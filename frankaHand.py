import pybullet as p
import numpy as np
import copy
import math
import pybullet_data
import os
import time

class FrankaHand:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), 
                initPos=[0.00000,0.00000,0.50000],timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.initPos = initPos
    self.maxVelocity = .35
    self.maxForce = 20.
    self.fingerForce = 2
    self.leftFingerIndex = 0
    self.rightFingerIndex = 1
    self.useSimulation = 1
    self.reset()

  def reset(self):
    objects = p.loadSDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand.sdf"))
    self.gripperUid = objects[0]
  
    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, self.initPos,
                                      [0.000000, 0.000000, 0.000000, 1.000000])

    # set mass of base body to 0 so it is fixed to the world (-1 is index for # base)
    p.changeDynamics(self.gripperUid, -1, mass = 0.) 

    p.resetJointState(self.gripperUid, self.leftFingerIndex, 0)
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.04,
                            force=self.maxForce)

    p.resetJointState(self.gripperUid, self.rightFingerIndex, 0)
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.04,
                            force=self.maxForce)

    # create the gearing contraint
    c = p.createConstraint(self.gripperUid,
                           self.leftFingerIndex,
                           self.gripperUid,
                           self.rightFingerIndex,
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    #self.trayUid = p.loadURDF(os.path.join(self.urdfRootPath, "tray/tray.urdf"), 0.640000,
    #                          0.075000, -0.190000, 0.000000, 0.000000, 1.000000, 0.000000)


  def applyAction(self, motorCommand):
      # only actuated one finger and mimic with the other one
      p.setJointMotorControl2(self.gripperUid,
                              self.leftFingerIndex,
                              p.POSITION_CONTROL,
                              targetPosition=motorCommand,
                              force=self.maxForce)

if __name__ == "__main__":
  p.connect(p.GUI)
  h = FrankaHand()
  stepCount = 0
  while (stepCount < 100000):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      #h.applyAction([fingerPos]*2)
      stepCount += 1
      p.stepSimulation()
      time.sleep(h.timeStep)
