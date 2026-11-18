import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)                                  # opens the 3D window
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")                          # the floor
p.loadURDF("cube_small.urdf", [0, 0, 1])          # a small cube, 1 m up

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)                           # PyBullet steps at 240 Hz by default

p.disconnect()