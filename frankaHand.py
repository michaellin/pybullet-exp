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

    # reset fingers to original position
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.,
                            force=self.maxForce)

    # for gear constraint right finger must be in position control with no force limit
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.,
                            force=0)

    # create the gearing contraint. Make right finger
    # child of left finger.
    c = p.createConstraint(self.gripperUid,
                           self.leftFingerIndex,
                           self.gripperUid,
                           self.rightFingerIndex,
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)


  def applyAction(self, motorCommand):
      # only actuated one finger and mimic with the other one
      p.setJointMotorControl2(self.gripperUid,
                              self.leftFingerIndex,
                              p.POSITION_CONTROL,
                              targetPosition=motorCommand,
                              force=self.maxForce)

if __name__ == "__main__":
  p.connect(p.GUI)
  urdfRoot=pybullet_data.getDataPath()
  p.loadURDF(os.path.join(urdfRoot, "plane.urdf"), 0, 0, -2)
  h = FrankaHand()
  stepCount = 0
  p.setRealTimeSimulation(1)
  while (1):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      h.applyAction(fingerPos)
      stepCount += 1
      time.sleep(h.timeStep)
