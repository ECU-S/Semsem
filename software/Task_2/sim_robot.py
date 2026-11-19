# sim_robot.py
import random
from robot_api import Robot

class SimRobot(Robot):
    """In-process fake robot: same interface as RealRobot, no network."""
    def __init__(self, seed=None):
        self._rng = random.Random(seed)   # pass a seed for repeatable runs
        self.yaw = 0.0
        self.steps = 0

    def _state(self):
        return {"ok": True, "yaw": round(self.yaw, 1), "steps": self.steps}

    def scan(self):
        noisy = lambda d: round(d + self._rng.uniform(-1.5, 1.5), 1)
        return {**self._state(), "dist_cm": [noisy(42), noisy(35.5), noisy(120)]}

    def walk(self, steps):
        self.steps += steps
        self.yaw += self._rng.uniform(-0.5, 0.5)     # small drift, feels real
        return {**self._state(), "steps_done": steps}

    def turn(self, deg):
        self.yaw += deg + self._rng.uniform(-1, 1)
        return self._state()

    def stop(self):
        return self._state()

    def close(self):
        pass