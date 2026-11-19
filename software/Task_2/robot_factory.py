# robot_factory.py
from real_robot import RealRobot
from sim_robot import SimRobot

def make_robot(mode="sim", port=9000):
    """Return a ready-to-use robot. Same interface either way."""
    if mode == "sim":
        return SimRobot()
    if mode == "real":
        robot = RealRobot(port=port)
        robot.start()               # begin listening for the ESP32
        robot.wait_until_ready()    # block until it connects and answers a ping
        return robot
    raise ValueError(f"unknown mode '{mode}' (use 'sim' or 'real')")