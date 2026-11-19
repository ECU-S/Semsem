# robot_factory.py
from real_robot import RealRobot
from sim_robot import SimRobot

def make_robot(mode="sim", host="127.0.0.1", port=5000, timeout=30, **kwargs):
    """Return a ready-to-use robot. Same interface either way.
    mode="sim"  -> SimRobot (no network)
    mode="real" -> RealRobot connected to host:port (the ESP32, or mock_robot.py)
    Extra keyword args (reply_timeout, move_timeout, ...) go to RealRobot."""
    if mode == "sim":
        return SimRobot()
    if mode == "real":
        robot = RealRobot(host=host, port=port, **kwargs)
        robot.start()
        robot.wait_until_ready(timeout)
        return robot
    raise ValueError(f"unknown mode '{mode}' (use 'sim' or 'real')")
