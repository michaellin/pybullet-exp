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
    self.leftFingerIndex = 2
    self.rightFingerIndex = 4
    self.leftFingerPadIndex = 3
    self.rightFingerPadIndex = 5
    self.baseZIndex = 0
    self.baseXIndex = 1
    self.reset()

  def reset(self):
    temp = p.loadSDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand2.sdf"),
                        useMaximalCoordinates = False)
    self.gripperUid = temp[0]

    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, self.initPos,
                                      [0.000000, 0.000000, 0.000000, 1.000000])

    # set mass of base body to 0 so it is fixed to the world (-1 is index for # base)
    #p.changeDynamics(self.gripperUid, -1, mass = 0.) 

    # reset fingers to original position
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.04,
                            force=self.maxForce)

    # for gear constraint right finger must be in position control with no force limit
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.04,
                            force=0)

    p.setJointMotorControl2(self.gripperUid, self.baseZIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=0.0)

    # move the prismatic wrist to 0
    p.setJointMotorControl2(self.gripperUid, self.baseXIndex,
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition = 0,
                            force=100.0)

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

    self.enableFingerSensor()



  def enableFingerSensor(self):
      p.enableJointForceTorqueSensor(self.gripperUid, self.leftFingerPadIndex)
      p.enableJointForceTorqueSensor(self.gripperUid, self.rightFingerPadIndex)

  def getFingerForce(self):
      leftState = p.getJointState(self.gripperUid, self.leftFingerPadIndex)
      rightState = p.getJointState(self.gripperUid, self.rightFingerPadIndex)
      #print("left {}".format(leftState[2][:3]))
      #print("right {}".format(rightState[2][:3]))
      #print("left {}".format(leftState[2][:3]))
      print("right {}".format(rightState[2]))

  def applyAction(self, motorCommand):
      # only actuated one finger and mimic with the other one
      p.setJointMotorControl2(self.gripperUid,
                              self.leftFingerIndex,
                              p.POSITION_CONTROL,
                              targetPosition=motorCommand,
                              force=self.maxForce)

  def liftObject(self):
      # only actuated one finger and mimic with the other one
      p.setJointMotorControl2(self.gripperUid,
                              self.baseZIndex,
                              p.VELOCITY_CONTROL,
                              targetVelocity=0.2,
                              velocityGain=0.5,
                              force=500.0)
  def stopLiftObject(self):
      p.setJointMotorControl2(self.gripperUid,
                              self.baseZIndex,
                              p.VELOCITY_CONTROL,
                              targetVelocity=0.0,
                              velocityGain=0.5,
                              force=500.0)

      zState = p.getJointState(self.gripperUid, self.baseZIndex)
      p.setJointMotorControl2(self.gripperUid,
                              self.baseZIndex,
                              p.POSITION_CONTROL,
                              targetPosition=zState[0],
                              force=500.0)

  def resetPose(self):
      # move z prismatic back to 0
      p.setJointMotorControl2(self.gripperUid,
                              self.baseZIndex,
                              p.POSITION_CONTROL,
                              targetPosition=0.0,
                              force=500.0)

      # reset fingers to original position
      p.setJointMotorControl2(self.gripperUid,
                              self.leftFingerIndex,
                              p.POSITION_CONTROL,
                              targetPosition=0.04,
                              force=self.maxForce)

      # for gear constraint right finger must be in position control with no force limit
      p.setJointMotorControl2(self.gripperUid,
                              self.rightFingerIndex,
                              p.POSITION_CONTROL,
                              targetPosition=0.04,
                              force=0)

if __name__ == "__main__":
  p.connect(p.GUI)
  urdfRoot=pybullet_data.getDataPath()
  p.loadURDF(os.path.join(urdfRoot, "plane.urdf"), 0, 0, -2)
  h = FrankaHand()
  stepCount = 0
  p.setRealTimeSimulation(1)
  time.sleep(1)
  while (1):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      h.applyAction(fingerPos)
      stepCount += 1
      time.sleep(h.timeStep)

