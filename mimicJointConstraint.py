#a mimic joint can act as a gear between two joints
#you can control the gear ratio in magnitude and sign (>0 reverses direction)

import pybullet as p
import pybullet_data
import time
import os
p.connect(p.GUI)
urdfRoot=pybullet_data.getDataPath()
p.loadURDF(os.path.join(urdfRoot, "plane.urdf"), 0, 0, -2)
gripper = p.loadURDF(os.path.join(urdfRoot, "franka_panda/panda_hand.urdf"), [0, 0, 0])
for i in range(p.getNumJoints(gripper)):
  print(p.getJointInfo(gripper, i))
  p.setJointMotorControl2(gripper, i, p.POSITION_CONTROL, targetPosition=0, force=0)

c = p.createConstraint(gripper,
                       0,
                       gripper,
                       1,
                       jointType=p.JOINT_GEAR,
                       jointAxis=[0, 1, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)


p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, -10)
  time.sleep(0.01)
#p.removeConstraint(c)
