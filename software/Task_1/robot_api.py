# robot_api.py
from abc import ABC, abstractmethod

class RobotError(Exception):
    """Robot replied ok=false, or the link is unsafe."""
    def __init__(self, code, msg=""):
        super().__init__(f"{code}: {msg}")
        self.code = code

class Robot(ABC):
    """What every robot (real or simulated) must be able to do.

    Reply dicts always contain "yaw" and "steps". Extra fields:
      scan -> "left", "front", "right" (cm or None), plus "dist_cm" = [left, front, right]
      walk -> "done" (steps actually taken), "blocked" (True = stopped early by an obstacle)
    """
    @abstractmethod
    def scan(self) -> dict: ...
    @abstractmethod
    def walk(self, steps: int) -> dict: ...
    @abstractmethod
    def turn(self, deg: float) -> dict: ...   # positive = left (CCW)
    @abstractmethod
    def stop(self) -> dict: ...
    @abstractmethod
    def reset(self) -> dict: ...              # zero steps/yaw (new origin)
    @abstractmethod
    def close(self) -> None: ...
