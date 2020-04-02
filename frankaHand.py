import pybullet as p
import numpy as np
import scipy.signal
import copy
import math
import pybullet_data
import os
import time
from pdControllerExplicit import PDControllerExplicit
from filter import lowPassFilter

""" Class for Franka Gripper. Implements sdf loading. Constraints setup and 
    joint controls.
"""
class FrankaHand:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), 
                initPos=[0.00000,0.00000,0.50000],timeStep=0.01):
    self._p = p
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.initPos = initPos
    self.maxVelocity = .35

    # define joint indices
    self.baseIndex = 0
    self.wristIndex = 1
    self.leftFingerIndex = 2
    self.rightFingerIndex = 4
    self.leftFingerPadIndex = 3
    self.rightFingerPadIndex = 5

    # create pd Controller instance
    self.gripperPdController = PDControllerExplicit(self._p)
    # set default controller parameters
    self.gripperQDes = 0.04
    self.gripperMaxForce = 100.
    self.gripperPosGain = 2500.0
    self.gripperVelGain = 1.0

    # filter for wrist force sensor
    # 2nd order butterworth at fc=20 Hz and fs=240 Hz
    self.filt = lowPassFilter(2, 20, 240)

    self.reset()

  def reset(self):
    temp = p.loadSDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand2.sdf"),
                        useMaximalCoordinates = False)
    self.gripperUid = temp[0]


    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, self.initPos,
                                      [0.000000, 0.000000, 0.000000, 1.000000])

    # disable position control to use explicit position control torque Torque control
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.00,
                            force=0.0)

    # for gear constraint right finger must be in position control with no force limit
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.00,
                            force=0.0)

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

    # command all joint to their default pose
    self.resetPose()

    self.enableSensors()


  def enableSensors(self):
    """ Enables Force Torque reading from pinger pads and wrist joint. """
    p.enableJointForceTorqueSensor(self.gripperUid, self.leftFingerPadIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.rightFingerPadIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.wristIndex)

  def getFingerForces(self):
    """ Returns the force torque value defined at the fixed joint between
        finger pad and finger link.
    """
    self.leftFingerF = p.getJointState(self.gripperUid, self.leftFingerPadIndex)[2]
    self.rightFingerF = p.getJointState(self.gripperUid, self.rightFingerPadIndex)[2]
    return self.leftFingerF[2], self.rightFingerF[2]

  def getWristForce(self):
    wristForce = p.getJointState(self.gripperUid, self.wristIndex)[2][0]
    return self.filt.filter(wristForce)


  def stepAdmittanceControl(self):
    dxDes = 1.0*self.wristForce
    wristPos = p.getJointState(self.gripperUid, self.wristIndex)[0]
    wristPosDes = wristPos + 0.002*dxDes

    # move the prismatic wrist to new desired position
    p.setJointMotorControl2(self.gripperUid, self.wristIndex,
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition = wristPosDes,
                            targetVelocity = 0.,
                            #positionGain = 1.,
                            #velocityGain = .1,
                            force=100.0)
      

  def applyAction(self, motorCommand, maxForce=None):
    if (not maxForce): maxForce = self.gripperMaxForce
    self.setJointControl(qDes = motorCommand,
                          maxForce = maxForce)
    # only actuated one finger and mimic with the other one
    #p.setJointMotorControl2(self.gripperUid,
    #                        self.leftFingerIndex,
    #                        p.POSITION_CONTROL,
    #                        targetPosition=motorCommand,
    #                        force=maxForce)

  def liftObject(self):
    # only actuated one finger and mimic with the other one
    p.setJointMotorControl2(self.gripperUid,
                            self.baseIndex,
                            p.VELOCITY_CONTROL,
                            targetVelocity=0.2,
                            velocityGain=0.5,
                            force=500.0)

  def stopLiftObject(self):
    p.setJointMotorControl2(self.gripperUid,
                            self.baseIndex,
                            p.VELOCITY_CONTROL,
                            targetVelocity=0.0,
                            velocityGain=0.5,
                            force=500.0)

    zState = p.getJointState(self.gripperUid, self.baseIndex)
    p.setJointMotorControl2(self.gripperUid,
                            self.baseIndex,
                            p.POSITION_CONTROL,
                            targetPosition=zState[0],
                            force=500.0)

  def resetPose(self):
    # move z prismatic back to 0
    p.setJointMotorControl2(self.gripperUid,
                            self.baseIndex,
                            p.POSITION_CONTROL,
                            targetPosition=-0.05,
                            force=500.0)

    # move wrist prismatic back to 0
    p.setJointMotorControl2(self.gripperUid,
                            self.wristIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=100.0)

    # reset fingers to original position
    self.setJointControl(qDes = 0.04,
                          maxForce = self.gripperMaxForce)

  def setJointControl(self, qDes=0.04, maxForce=100.):
    self.gripperQDes = qDes
    self.gripperMaxForce = maxForce

  def step(self):
    # perform explicit position control
    force = self.gripperPdController.computePD(self.gripperUid,
                                                self.leftFingerIndex,
                                                self.gripperQDes,
                                                0.,
                                                self.gripperPosGain,
                                                self.gripperVelGain,
                                                self.gripperMaxForce)
    
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.TORQUE_CONTROL,
                            force=force)

    
    self.getFingerForces()
    self.wristForce = self.filt.filter(-self.leftFingerF[1]-self.rightFingerF[1])
    #self.wristForce = self.getWristForce()

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
