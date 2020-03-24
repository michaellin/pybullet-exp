import pybullet as p
import numpy as np
import copy
import math
import pybullet_data
import os
import time

class FrankaHand:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.maxVelocity = .35
    self.maxForce = 200.
    self.fingerForce = 2
    self.fingerIndices = [0, 1]
    self.useSimulation = 1
    self.reset()

  def reset(self):
    objects = p.loadSDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand.sdf"))
    self.gripperUid = objects[0]
  
    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, [-0.100000, 0.000000, -0.150000],
                                      [0.000000, 0.000000, 0.000000, 1.000000])

    # set mass of base body to 0 so it is fixed to the world (-1 is index for # base)
    p.changeDynamics(self.gripperUid, -1, mass = 0.) 


    for jointIndex in self.fingerIndices:
        p.resetJointState(self.gripperUid, jointIndex, 0)
        p.setJointMotorControl2(self.gripperUid,
                                jointIndex,
                                p.POSITION_CONTROL,
                                targetPosition=0.2,
                                force=self.maxForce)

    #self.trayUid = p.loadURDF(os.path.join(self.urdfRootPath, "tray/tray.urdf"), 0.640000,
    #                          0.075000, -0.190000, 0.000000, 0.000000, 1.000000, 0.000000)

    self.motorName = "N/A"
    self.motorIndex = 0

    #for i in range(self.numJoints):
    #  jointInfo = p.getJointInfo(self.gripperUid, i)
    #  qIndex = jointInfo[3]
    #  if qIndex > -1:
    #    print("motorname: {}, index: {}".format(jointInfo[1], i))
    #    self.motorName = str(jointInfo[1])
    #    self.motorIndex = i

  def applyAction(self, motorCommand):
      for i, jointIndex in enumerate(self.fingerIndices):
          p.setJointMotorControl2(self.gripperUid,
                                  jointIndex,
                                  p.POSITION_CONTROL,
                                  targetPosition=motorCommand[i],
                                  force=self.maxForce)


if __name__ == "__main__":
  p.connect(p.GUI)
  h = FrankaHand()
  stepCount = 0
  while (stepCount < 100000):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      h.applyAction([fingerPos]*2)
      stepCount += 1
      p.stepSimulation()
      time.sleep(h.timeStep)
