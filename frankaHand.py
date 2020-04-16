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

from utils import sleeper


""" Class for Franka Gripper. Implements sdf loading. Constraints setup and 
    joint controls.
"""
class FrankaHand:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), 
                initPos=[0.00000,0.00000,0.50000],timeStep=0.01):
    self._p = p
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.sleeper = sleeper(self._p, self.timeStep)
    self.initPos = initPos
    self.maxVelocity = .35

    # define joint indices
    self.baseIndex = 0
    self.wristXIndex = 1
    self.wristYIndex = 2
    self.wristWIndex = 3
    self.rightFingerIndex = 4
    self.leftFingerIndex = 6
    self.rightFingerPadIndex = 5
    self.leftFingerPadIndex = 7


    ### LOAD GRIPPER MODELS 
    self.gripperUid = p.loadURDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand2.urdf"))

    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, self.initPos,
                                      [0.000000, 0.000000, 0.000000, 1.000000])



    # finger tip jacobian
    Rskew = np.zeros((3,3))
    Rskew[2,0] = 0.007
    Rskew[0,2] = -0.007
    I = np.eye(3)
    self.Jtip = np.block([[I, Rskew],[np.zeros((3,3)), I]])

    # load some reference frames
    #self.loadDebugRefFrames()

    ### SETUP CONTROLS
    # create pd Controller instance for gripper
    self.gripperPdController = PDControllerExplicit(self._p)
    # set default controller parameters
    self.gripperQDes = 0.04
    self.gripperMaxForce = 100.
    self.gripperPosGain = 1500.0
    self.gripperVelGain = 10.0

    # filter for wrist force sensor
    # 2nd order butterworth at fc=20 Hz and fs=1000 Hz
    self.filt = lowPassFilter(2, 20, 1000)

    # disable position control to use explicit position control torque Torque control
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.00,
                            force=0.0)

    # for gear constraint left finger must be in position control with no force limit
    p.setJointMotorControl2(self.gripperUid,
                            self.leftFingerIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.00,
                            force=0.0)

    # create the gearing contraint. Make left finger
    # child of right finger.
    c = p.createConstraint(self.gripperUid,
                           self.rightFingerIndex,
                           self.gripperUid,
                           self.leftFingerIndex,
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    # command all joint to their default pose
    self.resetPose()
    self.rightFT = FTSensor(self.gripperUid, self.rightFingerPadIndex)
    self.leftFT = FTSensor(self.gripperUid, self.leftFingerPadIndex)

    # let system settle and zero the force sensor
    self.sleeper.sleepSim(1, self.step)

    self.rightFT.zeroSensorBias()
    self.leftFT.zeroSensorBias()
    print("got bias")

  def enableSensors(self):
    """ Enables Force Torque reading from pinger pads and wrist joint. """
    p.enableJointForceTorqueSensor(self.gripperUid, self.rightFingerPadIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.leftFingerPadIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.wristXIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.wristYIndex)
    p.enableJointForceTorqueSensor(self.gripperUid, self.wristWIndex)

  def getFingerForces(self):
    """ Returns the force torque value defined at the fixed joint between
        finger pad and finger link.
    """
    self.rightFingerF = p.getJointState(self.gripperUid, self.rightFingerPadIndex)[2]
    self.leftFingerF = p.getJointState(self.gripperUid, self.leftFingerPadIndex)[2]
    return self.rightFingerF[2], self.leftFingerF[2]

  def getWristForce(self):
    wristForce = p.getJointState(self.gripperUid, self.wristXIndex)[2][0]
    return self.filt.filter(wristForce)


  def stepAdmittanceControl(self):
    dxDes = 1.0*self.wristForce
    wristPos = p.getJointState(self.gripperUid, self.wristXIndex)[0]
    wristPosDes = wristPos + 0.002*dxDes

    # move the prismatic wrist to new desired position
    p.setJointMotorControl2(self.gripperUid, self.wristXIndex,
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition = wristPosDes,
                            targetVelocity = 0.,
                            force=100.0)
      

  def stepFingerComplianceControl(self):
    # get finger force vector (6x1)
    ffl = self.rightFingerF
    ffr = self.leftFingerF

    # set force desired
    f_des = np.zeros(len(ffl))

    # get jacobian to joints
    fingerPos = p.getJointState(self.gripperUid, self.rightFingerIndex)[0]
    Jqr = np.zeros((3,6))
    Jqr[0,1] = 1.
    Jqr[1,2] = 1.
    Jqr[2,2] = -0.1034
    Jqr[2,3] = fingerPos
    Jqr[2,4] = 1.

    Jql = np.zeros((3,6))
    Jql[0,1] = 1.
    Jql[1,2] = 1.
    Jql[2,2] = -0.1034
    Jql[2,3] = -fingerPos
    Jql[2,4] = 1.

    fl_err = f_des - ffl
    fr_err = f_des - ffr
    Kl = np.eye(6)
    Kl[1,1] = 0.1 
    Kl[3,3] = 5
    Kr = np.eye(6)
    Kr[1,1] = 0.1 
    Kr[3,3] = 5 
    ul = Jql.dot(Kl.dot(self.Jtip.dot(fl_err)))
    ur = Jqr.dot(Kr.dot(self.Jtip.dot(fr_err)))
    u = ur + ul

    # move the prismatic wrist to new desired position
    p.setJointMotorControlArray(self.gripperUid, 
                            [self.wristXIndex, 
                            self.wristYIndex,
                            self.wristWIndex],
                            controlMode=p.TORQUE_CONTROL, 
                            forces=[u[0], u[1], u[2]])

  def applyAction(self, motorCommand, maxForce=None):
    if (not maxForce): maxForce = self.gripperMaxForce
    self.setJointControl(qDes = motorCommand,
                         maxForce = maxForce)

    # only actuated one finger and mimic with the other one
    #p.setJointMotorControl2(self.gripperUid,
    #                        self.rightFingerIndex,
    #                        p.POSITION_CONTROL,
    #                        targetPosition=motorCommand,
    #                        force=maxForce)
    return

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

    p.resetJointState(self.gripperUid,
                      self.wristXIndex,
                      targetValue=0)

    p.resetJointState(self.gripperUid,
                      self.wristYIndex,
                      targetValue=0)

    p.resetJointState(self.gripperUid,
                      self.wristWIndex,
                      targetValue=0)

    # turn off position control on wrist joints
    p.setJointMotorControl2(self.gripperUid,
                            self.wristXIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=0.0)

    p.setJointMotorControl2(self.gripperUid,
                            self.wristYIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=0.0)

    p.setJointMotorControl2(self.gripperUid,
                            self.wristWIndex,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=0.0)

    # reset fingers to home position
    self.setJointControl(qDes = 0.04,
                          maxForce = self.gripperMaxForce)

  def setJointControl(self, qDes=0.04, maxForce=100.):
    self.gripperQDes = qDes
    self.gripperMaxForce = maxForce

  def step(self):
    # perform explicit position control
    force = self.gripperPdController.computePD(self.gripperUid,
                                                self.rightFingerIndex,
                                                self.gripperQDes,
                                                0.,
                                                self.gripperPosGain,
                                                self.gripperVelGain,
                                                self.gripperMaxForce)
    
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.TORQUE_CONTROL,
                            force=force)

    
    self.getFingerForces()
    self.wristForce = self.filt.filter(-self.rightFingerF[1]-self.leftFingerF[1])

    self.leftFT.step()
    self.rightFT.step()

  def loadDebugRefFrames(self):
    p.addUserDebugText("rightpad", [0,0,0.05],textColorRGB=[1,0,0],textSize=1.,
                        parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.rightFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0], parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.rightFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.rightFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.rightFingerPadIndex)

    p.addUserDebugText("leftpad", [0,0,0.05],textColorRGB=[1,0,0],textSize=1.,
                        parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.leftFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0], parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.leftFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.leftFingerPadIndex)
    p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.leftFingerPadIndex)

    p.addUserDebugText("wrist", [0,0,0.05],textColorRGB=[1,0,0],textSize=1.,
                        parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.wristWIndex)
    p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0], parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.wristWIndex)
    p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.wristWIndex)
    p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.wristWIndex)


""" Class for Force torque sensors mounted at the finger pads. Implements functions to
detemine if sensors are in contact, getting a filtered signal and displaying what the
contact forces are.
"""
class FTSensor(object):
  def __init__(self, gripperUid, linkIndex):
    self.gripperUid = gripperUid
    self.sensorIndex = linkIndex

    # enable force torque sensors
    p.enableJointForceTorqueSensor(self.gripperUid, self.sensorIndex)


    self.lineId = p.addUserDebugLine([0,0,0],[0,0.1,0],[1,1,1], 
                        parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.sensorIndex)

    self.bias = np.zeros(6)

    self.contactState = False

    # filter for wrist force sensor
    # 2nd order butterworth at fc=300 Hz and fs=1000 Hz
    #self.filt = lowPassFilter(2, 300, 1000, vecSize=6)


  def getForces(self):
    #return self.filt.filter(p.getJointState(self.gripperUid, self.sensorIndex)[2])
    self.forces = p.getJointState(self.gripperUid, self.sensorIndex)[2] - self.bias
    return self.forces


  def inContact(self, thresh=0.1):
    """ Return true or false depending on whether the sensor is in contact.
    Only using force, not moments.
    """
    f_mag = np.linalg.norm(self.forces[:3])
    return f_mag > thresh


  def zeroSensorBias(self):
    self.bias = self.getForces()

  def _displayForceVector(self):
    f = self.forces[:3]
    f_mag = np.linalg.norm(f)
    f_unit = f/(f_mag + 1e-8)
    if (f_mag > 0.1): f_mag = 0.1
    p.addUserDebugLine([0,0,0],f_mag*f_unit,[0,0,0], 
                        parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.sensorIndex,
                        replaceItemUniqueId=self.lineId)

  def step(self):
    self.getForces()
    self._displayForceVector()
    currContactState = self.inContact() 
    if (currContactState != self.contactState):
      if self.inContact():
        p.changeVisualShape(self.gripperUid,
                            self.sensorIndex,
                            rgbaColor=[1,0,0,1])
      else:
        p.changeVisualShape(self.gripperUid,
                            self.sensorIndex,
                            rgbaColor=[0,1,0,1])
    self.contactState = currContactState
    


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
