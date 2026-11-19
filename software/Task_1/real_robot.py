# real_robot.py  -- laptop is the CLIENT: it connects to the robot (ESP32 or mock_robot.py)
import socket, json, itertools, time, threading
from robot_api import Robot, RobotError
from msg_log import MsgLog

class RealRobot(Robot):
    MOVES = {"walk", "turn"}

    def __init__(self, host="127.0.0.1", port=5000, reply_timeout=2.0,
                 move_timeout=10.0, retry_delay=1.0, log=None):
        self.host, self.port = host, port
        self.reply_timeout = reply_timeout     # scan / ping / stop / reset
        self.move_timeout = move_timeout       # walk / turn (they take real time)
        self.retry_delay = retry_delay         # pause between reconnect attempts
        self.log = log or MsgLog()
        self._ids = itertools.count(1)
        self._conn = None
        self._buf = b""
        self._safe = False                     # unsafe until connected AND ping answered
        self._closing = False
        self._lock = threading.RLock()         # one conversation with the robot at a time

    # ---------- connecting (and reconnecting) ----------
    def start(self):
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def wait_until_ready(self, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            if self._safe:
                return
            time.sleep(0.1)
        raise RobotError("NO_ROBOT", f"could not reach robot at {self.host}:{self.port}")

    def _connect_loop(self):
        """Runs forever: whenever we are not connected, try to connect."""
        while not self._closing:
            if self._conn is not None:
                time.sleep(0.2)
                continue
            try:
                conn = socket.create_connection((self.host, self.port), timeout=2.0)
            except OSError:
                time.sleep(self.retry_delay)   # robot not up yet, try again
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._conn, self._buf = conn, b""
            self.log.write("event", {"what": "connected", "addr": f"{self.host}:{self.port}"})
            try:
                self._raw_request("ping", timeout=1.0)   # prove it is really alive
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

    # ---------- low level: send one request, wait for the reply with the same id ----------
    def _read_line(self, deadline):
        while b"\n" not in self._buf:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise socket.timeout()
            self._conn.settimeout(remaining)
            chunk = self._conn.recv(4096)
            if not chunk:
                raise ConnectionError("robot closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8")

    def _raw_request(self, cmd, timeout=None, **args):
        with self._lock:
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
                    if reply.get("id") == msg["id"]:     # ignore late replies to old requests
                        break
            except socket.timeout:
                raise RobotError("TIMEOUT", f"no reply to {cmd} (id {msg['id']})")
            except (ConnectionError, OSError, ValueError):
                raise RobotError("LINK_DOWN", "connection lost")
            if not reply.get("ok", False):
                # protocol v1 uses "error"; accept "err" too just in case
                raise RobotError(reply.get("error", reply.get("err", "UNKNOWN")),
                                 reply.get("msg", ""))
            return reply

    # ---------- safety ----------
    def _fail(self, err):
        self._safe = False
        self.log.write("event", {"what": "unsafe", "reason": err.code})
        if err.code == "TIMEOUT":
            try:                                   # link may be alive: stop, then check
                self._raw_request("stop", timeout=1.0)
                self._raw_request("ping", timeout=1.0)
                self._safe = True
                self.log.write("event", {"what": "recovered_after_timeout"})
                return
            except RobotError:
                pass
        self._drop_conn()                          # connect loop will redial

    def _request(self, cmd, timeout=None, **args):
        if cmd in self.MOVES and not self._safe:
            raise RobotError("UNSAFE", "link not healthy; move refused")
        try:
            return self._raw_request(cmd, timeout, **args)
        except RobotError as e:
            if e.code in ("TIMEOUT", "LINK_DOWN"):
                self._fail(e)
            raise

    # ---------- the Robot interface ----------
    def scan(self):
        r = self._request("scan")
        r["dist_cm"] = [r.get("left"), r.get("front"), r.get("right")]   # None = out of range
        return r

    def walk(self, steps): return self._request("walk", timeout=self.move_timeout, n=steps)
    def turn(self, deg):   return self._request("turn", timeout=self.move_timeout, deg=deg)
    def stop(self):        return self._request("stop")
    def reset(self):       return self._request("reset")

    def close(self):
        self._closing = True
        self._drop_conn()
        self.log.close()
