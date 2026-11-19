# real_robot.py
import socket, json, itertools, time, threading
from robot_api import Robot, RobotError
from msg_log import MsgLog

class RealRobot(Robot):
    def __init__(self, port=9000, reply_timeout=2.0, log=None):
        self.reply_timeout = reply_timeout
        self.log = log or MsgLog()
        self._ids = itertools.count(1)      # 1, 2, 3, ... one number per request
        self._conn = None                   # the connected ESP32 (None = nobody yet)
        self._buf = b""                     # leftover bytes between messages

        # Open the "door" and listen on it
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("0.0.0.0", port))   # 0.0.0.0 = accept from any device on the network
        self._server.listen(1)

        self._safe = False              # starts unsafe until a robot connects and answers a ping
        self._closing = False
        self._lock = threading.RLock()  # one conversation with the robot at a time

        # ---------- background reconnect ----------
    def start(self):
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def wait_until_ready(self, timeout=30):
        """Block until a robot is connected AND answered a ping."""
        end = time.time() + timeout
        while time.time() < end:
            if self._safe:
                return
            time.sleep(0.1)
        raise RobotError("NO_ROBOT", "robot not ready in time")

    def _accept_loop(self):
        """Runs forever: whenever nobody is connected, wait for the ESP32 to (re)connect."""
        while not self._closing:
            if self._conn is not None:
                time.sleep(0.2)
                continue
            self._server.settimeout(1.0)
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break                       # server socket was closed
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._conn, self._buf = conn, b""
            self.log.write("event", {"what": "connected", "addr": addr[0]})
            try:
                self._raw_request("ping", timeout=1.0)   # prove it's really alive
                self._safe = True
                self.log.write("event", {"what": "safe_again"})
            except RobotError:
                self._drop_conn()

    def _drop_conn(self):
        with self._lock:
            if self._conn:
                try: self._conn.close()
                except OSError: pass
            self._conn, self._buf = None, b""

    # ---------- safety ----------
    MOVES = {"walk", "turn"}

    def _fail(self, err):
        self._safe = False
        self.log.write("event", {"what": "unsafe", "reason": err.code})
        if err.code == "TIMEOUT":
            # Link may still be alive: stop the robot, then check it answers.
            try:
                self._raw_request("stop", timeout=1.0)
                self._raw_request("ping", timeout=1.0)
                self._safe = True
                self.log.write("event", {"what": "recovered_after_timeout"})
                return
            except RobotError:
                pass
        self._drop_conn()                   # accept loop now waits for a reconnect

    def _request(self, cmd, timeout=None, **args):
        if cmd in self.MOVES and not self._safe:
            raise RobotError("UNSAFE", "link not healthy; move refused")
        try:
            return self._raw_request(cmd, timeout, **args)
        except RobotError as e:
            if e.code in ("TIMEOUT", "LINK_DOWN"):
                self._fail(e)
            raise                           # the mission loop still hears about it

    # ---------- low-level helpers ----------
    def _read_line(self, deadline):
        """Read bytes until we have one full line (ending in \\n)."""
        import time
        while b"\n" not in self._buf:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise socket.timeout()
            self._conn.settimeout(remaining)
            chunk = self._conn.recv(4096)
            if not chunk:                       # empty = the other side closed
                raise ConnectionError("robot closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8")

    def _raw_request(self, cmd, timeout=None, **args):
        import time
        if self._conn is None:
            raise RobotError("LINK_DOWN", "no robot connected")

        msg = {"id": next(self._ids), "cmd": cmd, **args}
        self.log.write("out", msg)
        deadline = time.time() + (timeout or self.reply_timeout)

        try:
            self._conn.sendall((json.dumps(msg) + "\n").encode("utf-8"))
            while True:
                reply = json.loads(self._read_line(deadline))
                self.log.write("in", reply)
                if reply.get("id") == msg["id"]:   # ignore late replies to OLD requests
                    break
        except socket.timeout:
            raise RobotError("TIMEOUT", f"no reply to {cmd} (id {msg['id']})")
        except (ConnectionError, OSError):
            raise RobotError("LINK_DOWN", "connection lost")

        if not reply.get("ok", False):
            raise RobotError(reply.get("err", "UNKNOWN"), reply.get("msg", ""))
        return reply

    # ---------- the Robot interface ----------
    def scan(self):        return self._request("scan")
    def walk(self, steps): return self._request("walk", timeout=10, steps=steps)
    def turn(self, deg):   return self._request("turn", timeout=10, deg=deg)
    def stop(self):        return self._request("stop")

    def close(self):
        if self._conn: self._conn.close()
        self._server.close()
        self.log.close()