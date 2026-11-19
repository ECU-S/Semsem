# sim_robot.py  -- in-process fake robot, same interface and reply fields as the real one
import random
from robot_api import Robot, RobotError

def _wrap(a):
    return (a + 180) % 360 - 180

class SimRobot(Robot):
    def __init__(self, seed=None):
        self._rng = random.Random(seed)
        self.yaw = 0.0
        self.steps = 0

    def _state(self):
        return {"ok": True, "yaw": round(_wrap(self.yaw), 1), "steps": self.steps}

    def scan(self):
        noisy = lambda d: round(d + self._rng.uniform(-1.5, 1.5), 1)
        left, front, right = noisy(42), noisy(35.5), noisy(120)
        return {**self._state(), "left": left, "front": front, "right": right,
                "dist_cm": [left, front, right]}

    def walk(self, steps):
        if not isinstance(steps, int) or isinstance(steps, bool) or not 1 <= steps <= 50:
            raise RobotError("BAD_ARGS", "n must be an int 1..50")
        self.steps += steps
        self.yaw = _wrap(self.yaw + self._rng.uniform(-0.5, 0.5))   # small drift
        return {**self._state(), "done": steps, "blocked": False}

    def turn(self, deg):
        if not isinstance(deg, (int, float)) or isinstance(deg, bool) or not -180 <= deg <= 180:
            raise RobotError("BAD_ARGS", "deg must be a number -180..180")
        self.yaw = _wrap(self.yaw + deg + self._rng.uniform(-1, 1))
        return self._state()

    def stop(self):  return self._state()

    def reset(self):
        self.yaw, self.steps = 0.0, 0
        return self._state()

    def close(self): pass
