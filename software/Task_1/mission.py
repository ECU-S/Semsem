# mission.py -- main mission loop, written against robot_api.Robot (sim or real)
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Optional

from robot_api import Robot, RobotError
from sim_robot import SimRobot

STEP_CM = 2.0   # nominal cm per step; must match mock/firmware, calibrate on the real robot


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def clear(d: Optional[float]) -> float:
    """Protocol: None = no echo within 200 cm = open space (NOT a failed scan)."""
    return float("inf") if d is None else d


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class MoveCommand:
    action: str            # "forward", "turn_left", "turn_right", "stop"
    value: float = 0.0     # cm for forward, degrees for turns
    is_detour: bool = False


# ==========================================
# Dead reckoning: robot only reports steps + yaw, never x/y
# ==========================================
class Odometry:
    def __init__(self) -> None:
        self.reset_to(0, 0.0)

    def reset_to(self, steps: int, yaw: float) -> None:
        self.x = self.y = 0.0
        self.steps, self.yaw = steps, yaw

    def update(self, reply: dict[str, Any]) -> None:
        steps, yaw = reply["steps"], reply["yaw"]
        d_steps = steps - self.steps
        if d_steps > 0:
            heading = self.yaw + wrap(yaw - self.yaw) / 2   # average heading during the walk
            r = math.radians(heading)
            self.x += d_steps * STEP_CM * math.cos(r)
            self.y += d_steps * STEP_CM * math.sin(r)
        self.steps, self.yaw = steps, yaw

    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.yaw)


# ==========================================
# Stubs for Map & Planner teams
# ==========================================
class StubMap:
    def __init__(self) -> None:
        self.last_scan: Optional[dict[str, Any]] = None
        self.last_pose: Optional[Pose] = None

    def update(self, scan: dict[str, Any], pose: Pose) -> None:
        self.last_scan, self.last_pose = scan, pose   # Map team replaces this


class StubPlanner:
    def __init__(
        self,
        obstacle_threshold_cm: float = 20.0,
        detour_angle_deg: float = 30.0,
        detour_step_cm: float = 20.0,
        target_yaw_deg: float = 0.0,
        yaw_tolerance_deg: float = 5.0,
    ) -> None:
        self.obstacle_threshold_cm = obstacle_threshold_cm
        self.detour_angle_deg = detour_angle_deg
        self.detour_step_cm = detour_step_cm
        self.target_yaw_deg = target_yaw_deg
        self.yaw_tolerance_deg = yaw_tolerance_deg
        self._stage = 0   # 0: normal, 1: turned away, 2: stepped around

    def plan(self, map_data: StubMap, pose: Pose, remaining: float, blocked: bool = False) -> MoveCommand:
        scan = map_data.last_scan or {}
        front = clear(scan.get("front"))

        # obstacle ahead (seen by scan, or the last walk was stopped early): turn to the more open side
        if front < self.obstacle_threshold_cm or blocked:
            self._stage = 1
            left, right = clear(scan.get("left")), clear(scan.get("right"))
            side = "turn_left" if left >= right else "turn_right"
            return MoveCommand(side, self.detour_angle_deg, is_detour=True)

        if self._stage == 1:
            self._stage = 2
            return MoveCommand("forward", self.detour_step_cm, is_detour=True)

        yaw_err = wrap(self.target_yaw_deg - pose.yaw_deg)
        if abs(yaw_err) > self.yaw_tolerance_deg:
            action = "turn_left" if yaw_err > 0 else "turn_right"
            return MoveCommand(action, min(abs(yaw_err), self.detour_angle_deg), is_detour=True)

        self._stage = 0
        return MoveCommand("forward", min(20.0, max(0.0, remaining)), is_detour=False)


# ==========================================
# Mission loop
# ==========================================
@dataclass
class MissionConfig:
    target_distance: float
    target_yaw_deg: float = 0.0
    max_stuck_cycles: int = 6      # consecutive cycles with no position change
    max_empty_scans: int = 3       # consecutive SENSOR_TIMEOUTs
    max_cycles: int = 500          # hard cap so the loop can never run forever
    loop_delay_sec: float = 0.02


class MissionRunner:
    def __init__(self, robot: Robot, mapper: StubMap, planner: StubPlanner, config: MissionConfig) -> None:
        self.robot, self.mapper, self.planner, self.cfg = robot, mapper, planner, config
        self.odo = Odometry()
        self.progress = 0.0      # distance made good toward the target
        self.detour = 0.0        # distance walked in detour sub-routines (never touches progress)
        self.stuck = 0
        self.empty_scans = 0

    def _execute(self, cmd: MoveCommand) -> dict[str, Any]:
        if cmd.action == "forward":
            n = min(50, max(1, round(cmd.value / STEP_CM)))   # protocol: walk n = 1..50 steps
            return self.robot.walk(n)
        if cmd.action == "turn_left":
            return self.robot.turn(min(180.0, cmd.value))
        if cmd.action == "turn_right":
            return self.robot.turn(-min(180.0, cmd.value))
        raise ValueError(f"unknown action {cmd.action}")

    def _safe_stop(self) -> None:
        try:
            self.robot.stop()
        except RobotError:
            pass   # link is already down; the firmware's own safety stop has to cover this

    def run(self) -> str:
        print(f"\n[Mission] Started | target {self.cfg.target_distance:.1f} cm | yaw {self.cfg.target_yaw_deg:.1f}")
        try:
            return self._run()
        except RobotError as e:
            return f"ABORTED: robot error {e.code} ({e})"
        finally:
            self._safe_stop()

    def _run(self) -> str:
        reply = self.robot.reset()                       # new origin: steps and yaw to zero
        self.odo.reset_to(reply["steps"], reply["yaw"])
        blocked = False
        th = math.radians(self.cfg.target_yaw_deg)

        for _ in range(self.cfg.max_cycles):
            if self.progress >= self.cfg.target_distance - STEP_CM / 2:
                return f"COMPLETED: reached {self.progress:.1f} (detour: {self.detour:.1f})"

            # 1. scan (SENSOR_TIMEOUT = failed scan; anything else is a real error and aborts)
            try:
                scan = self.robot.scan()
            except RobotError as e:
                if e.code != "SENSOR_TIMEOUT":
                    raise
                self.empty_scans += 1
                print(f"[Warn] scan failed ({self.empty_scans}/{self.cfg.max_empty_scans})")
                if self.empty_scans >= self.cfg.max_empty_scans:
                    return "ABORTED: scan failed repeatedly"
                time.sleep(self.cfg.loop_delay_sec)
                continue
            self.empty_scans = 0

            # 2. update map
            self.odo.update(scan)
            pose = self.odo.pose()
            self.mapper.update(scan, pose)

            # 3. plan
            remaining = self.cfg.target_distance - self.progress
            cmd = self.planner.plan(self.mapper, pose, remaining, blocked)
            if cmd.action == "stop" or cmd.value <= 0:
                return f"STOPPED by planner at {self.progress:.1f} (detour: {self.detour:.1f})"

            # 4. send move
            before = self.odo.pose()
            reply = self._execute(cmd)
            self.odo.update(reply)
            after = self.odo.pose()
            blocked = bool(reply.get("blocked", False))

            # 5. account: detours stay separate, progress only counts motion along the target heading
            dx, dy = after.x - before.x, after.y - before.y
            moved = math.hypot(dx, dy)
            if cmd.action == "forward":
                if cmd.is_detour:
                    self.detour += moved
                else:
                    self.progress += max(0.0, dx * math.cos(th) + dy * math.sin(th))

            # 6. stuck = position not changing for N cycles in a row (a normal detour never does that)
            self.stuck = self.stuck + 1 if moved < 0.5 else 0
            if self.stuck >= self.cfg.max_stuck_cycles:
                return f"ABORTED: stuck for {self.stuck} cycles"

            print(
                f"[Loop] {cmd.action:<10} detour={str(cmd.is_detour):<5} blocked={str(blocked):<5} | "
                f"progress={self.progress:6.1f}/{self.cfg.target_distance:.1f} | "
                f"detour={self.detour:5.1f} | yaw={after.yaw_deg:6.1f}"
            )
            time.sleep(self.cfg.loop_delay_sec)

        return "ABORTED: max cycles reached"


# ==========================================
# Offline tests: SimRobot plus scripted faults (no network)
# ==========================================
class ScriptedRobot(SimRobot):
    def __init__(self, book_at_scan=None, fail_scan_from=None, link_down_at_scan=None, blocked_from_walk=None):
        super().__init__(seed=1)
        self.book_at_scan, self.fail_scan_from = book_at_scan, fail_scan_from
        self.link_down_at_scan, self.blocked_from_walk = link_down_at_scan, blocked_from_walk
        self.n_scans = self.n_walks = 0

    def scan(self):
        self.n_scans += 1
        if self.link_down_at_scan and self.n_scans >= self.link_down_at_scan:
            raise RobotError("LINK_DOWN", "connection lost")
        if self.fail_scan_from and self.n_scans >= self.fail_scan_from:
            raise RobotError("SENSOR_TIMEOUT", "ultrasonic did not answer")
        front = 12.0 if self.n_scans == self.book_at_scan else None
        return {**self._state(), "left": None, "front": front, "right": None, "dist_cm": [None, front, None]}

    def walk(self, steps):
        self.n_walks += 1
        if self.blocked_from_walk and self.n_walks >= self.blocked_from_walk:
            return {**self._state(), "done": 0, "blocked": True}
        return super().walk(steps)


def run_test_suite() -> None:
    cfg = MissionConfig(target_distance=100.0, max_stuck_cycles=5)

    def go(label, robot):
        print("->", label, ":", MissionRunner(robot, StubMap(), StubPlanner(), cfg).run())

    go("Scenario 1 (normal + book detour)", ScriptedRobot(book_at_scan=3))
    go("Scenario 2 (scan keeps failing)", ScriptedRobot(fail_scan_from=2))
    go("Scenario 3 (link drops)", ScriptedRobot(link_down_at_scan=3))
    go("Scenario 4 (stuck, walk always blocked)", ScriptedRobot(blocked_from_walk=2))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "real":
        from robot_factory import make_robot
        robot = make_robot("real")          # needs mock_robot.py (or the ESP32) running
        try:
            print(MissionRunner(robot, StubMap(), StubPlanner(), MissionConfig(target_distance=100.0)).run())
            print("mock truth:", robot._request("truth"))   # mock only: compare with odometry
        finally:
            robot.close()
    else:
        run_test_suite()