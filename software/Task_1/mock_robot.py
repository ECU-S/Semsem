"""
mock_robot.py: fake ESP32 for the quadruped. Speaks the same protocol as the real firmware.

PROTOCOL v1
  Transport: TCP (default port 5000). One JSON object per line, "\n" terminated, UTF-8.
  Units: cm and degrees. Yaw is -180..180, 0 = start heading, positive = turning left (CCW).

  Request : {"id": 7, "cmd": "walk", "n": 5}
  Success : {"id": 7, "ok": true, "yaw": 12.5, "steps": 15, ...cmd-specific fields}
  Failure : {"id": 7, "ok": false, "error": "BAD_ARGS", "msg": "n must be an int 1..50"}

  The reply always echoes the request "id" so the client can match replies to requests.
  Every successful reply carries "yaw" (IMU heading) and "steps" (cumulative steps since reset).

  Commands
    ping                  -> {}
    scan                  -> {"left": cm|null, "front": cm|null, "right": cm|null}
                             null = no echo within 200 cm
    walk  n (int 1..50)   -> {"done": int, "blocked": bool}
                             blocked=true means it stopped early because of an obstacle.
                             That is NOT an error, the planner needs to know it happened.
    turn  deg (-180..180) -> {}
    stop                  -> {}
    status                -> {}
    reset                 -> {}   zero steps/yaw (new origin)
    truth                 -> {"x": cm, "y": cm}   MOCK ONLY, for testing. Real firmware answers UNKNOWN_CMD.

  Error codes: BAD_JSON, UNKNOWN_CMD, BAD_ARGS, SENSOR_TIMEOUT
"""
import argparse, json, math, random, socket, time

STEP_CM = 2.0                   # nominal distance per step
MAX_RANGE = 200                 # ultrasonic max range we trust (cm)
STOP_DIST = 8                   # walk halts if the front obstacle is closer than this (cm)
ROOM = (300, 300)               # room size (cm)
OBSTACLES = [(120, 100, 160, 140), (200, 200, 230, 260)]   # "books": (x1, y1, x2, y2)


class ProtoError(Exception):
    def __init__(self, code, msg):
        self.code, self.msg = code, msg


def wrap(a):
    return (a + 180) % 360 - 180


class World:
    def __init__(self, args):
        self.args = args
        self.reset()

    def reset(self):
        self.x, self.y, self.yaw, self.steps = 20.0, 20.0, 0.0, 0   # true state, robot never sees x, y

    def hit(self, x, y):
        w, h = ROOM
        if not (0 <= x <= w and 0 <= y <= h):
            return True
        return any(x1 <= x <= x2 and y1 <= y <= y2 for x1, y1, x2, y2 in OBSTACLES)

    def ray(self, deg):
        r = math.radians(deg)
        for d in range(1, MAX_RANGE + 1):
            if self.hit(self.x + d * math.cos(r), self.y + d * math.sin(r)):
                return d
        return None

    def imu_yaw(self):
        return round(wrap(self.yaw + random.gauss(0, 0.5)), 1)

    def scan(self):
        if random.random() < self.args.fail:
            raise ProtoError("SENSOR_TIMEOUT", "ultrasonic did not answer")
        out = {}
        for name, off in (("left", 90), ("front", 0), ("right", -90)):
            d = self.ray(self.yaw + off)
            out[name] = None if d is None else round(max(2.0, d + random.gauss(0, 1.0)), 1)
        return out

    def walk(self, n):
        done = 0
        for _ in range(n):
            d = self.ray(self.yaw)
            if d is not None and d < STOP_DIST:
                return {"done": done, "blocked": True}
            step = STEP_CM * random.uniform(0.9, 1.05)          # imperfect steps
            r = math.radians(self.yaw)
            self.x += step * math.cos(r)
            self.y += step * math.sin(r)
            self.yaw = wrap(self.yaw + random.gauss(0, 0.3))    # gait drift
            self.steps += 1
            done += 1
            time.sleep(self.args.step_time)
        return {"done": done, "blocked": False}

    def turn(self, deg):
        self.yaw = wrap(self.yaw + deg * random.uniform(0.95, 1.05))   # imperfect turns
        time.sleep(abs(deg) * self.args.step_time / 10)


def dispatch(world, req):
    cmd = req.get("cmd")
    if cmd in ("ping", "status", "stop"):
        return {}
    if cmd == "scan":
        return world.scan()
    if cmd == "walk":
        n = req.get("n")
        if not isinstance(n, int) or isinstance(n, bool) or not 1 <= n <= 50:
            raise ProtoError("BAD_ARGS", "n must be an int 1..50")
        return world.walk(n)
    if cmd == "turn":
        deg = req.get("deg")
        if not isinstance(deg, (int, float)) or isinstance(deg, bool) or not -180 <= deg <= 180:
            raise ProtoError("BAD_ARGS", "deg must be a number -180..180")
        world.turn(deg)
        return {}
    if cmd == "reset":
        world.reset()
        return {}
    if cmd == "truth":
        return {"x": round(world.x, 1), "y": round(world.y, 1)}
    raise ProtoError("UNKNOWN_CMD", f"unknown cmd: {cmd!r}")


def serve(conn, world, args):
    f = conn.makefile("rwb")
    for line in f:
        if args.latency:
            time.sleep(args.latency)
        rid = None
        try:
            try:
                req = json.loads(line)
            except ValueError:
                raise ProtoError("BAD_JSON", "not valid JSON")
            if not isinstance(req, dict):
                raise ProtoError("BAD_JSON", "expected a JSON object")
            rid = req.get("id")
            body = dispatch(world, req)
            reply = {"id": rid, "ok": True, **body, "yaw": world.imu_yaw(), "steps": world.steps}
        except ProtoError as e:
            reply = {"id": rid, "ok": False, "error": e.code, "msg": e.msg}
        if random.random() < args.drop:
            continue        # command ran, reply "lost": tests the client's timeout path
        f.write((json.dumps(reply) + "\n").encode())
        f.flush()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--latency", type=float, default=0.0, help="extra delay per command (s)")
    p.add_argument("--drop", type=float, default=0.0, help="probability a reply is lost")
    p.add_argument("--fail", type=float, default=0.0, help="probability a scan times out")
    p.add_argument("--step-time", type=float, default=0.05, help="seconds per simulated step")
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args()
    random.seed(args.seed)

    world = World(args)
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"mock robot listening on :{args.port}")
    while True:                         # one client at a time, like the real ESP32
        conn, addr = srv.accept()
        print("client connected:", addr)
        try:
            serve(conn, world, args)
        except (ConnectionError, OSError):
            pass
        print("client disconnected")   # a real robot would stop here (safety)


if __name__ == "__main__":
    main()