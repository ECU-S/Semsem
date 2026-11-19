# robot_api.py
from abc import ABC, abstractmethod

class RobotError(Exception):
    """Robot replied ok=false, or the link is unsafe."""
    def __init__(self, code, msg=""):
        super().__init__(f"{code}: {msg}")
        self.code = code

class Robot(ABC):
    @abstractmethod
    def scan(self) -> dict: ...          # returns {"dist_cm":[l,f,r], "yaw":..., "steps":...}

    @abstractmethod
    def walk(self, steps: int) -> dict: ...

    @abstractmethod
    def turn(self, deg: float) -> dict: ...

    @abstractmethod
    def stop(self) -> dict: ...

    @abstractmethod
    def close(self) -> None: ...