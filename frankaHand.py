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
    self.rightFingerPadIndex = 5
    self.leftFingerIndex = 6
    self.leftFingerPadIndex = 7


    ### LOAD GRIPPER MODELS 
    self.gripperUid = p.loadURDF(os.path.join(self.urdfRootPath, "franka_panda/panda_hand2.urdf"))

    for i in range(p.getNumJoints(self.gripperUid)):
      print(p.getJointInfo(self.gripperUid, i)[1])

    # load some reference frames
    #self.loadDebugRefFrames()

    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.gripperUid, self.initPos,
                                      [0.000000, 0.000000, 0.000000, 1.000000])


    # finger tip jacobian
    Rskew = np.zeros((3,3))
    Rskew[2,0] = 0.007
    Rskew[0,2] = -0.007
    I = np.eye(3)
    self.Jtip = np.block([[I, Rskew],[np.zeros((3,3)), I]])


    ### SETUP CONTROLS
    # create pd Controller instance for gripper
    self.fingerPDController = PDControllerExplicit(self._p,
                                                  self.gripperUid,
                                                  self.rightFingerIndex)

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
    p.changeConstraint(c, gearRatio=-1, maxForce=100000)


    # initialize wrist PD controller
    self.wristXPDController = PDControllerExplicit(self._p,
                                                  self.gripperUid,
                                                  self.wristXIndex)
    self.wristYPDController = PDControllerExplicit(self._p,
                                                  self.gripperUid,
                                                  self.wristYIndex)
    self.wristWPDController = PDControllerExplicit(self._p,
                                                  self.gripperUid,
                                                  self.wristWIndex)
    self.wristPosGain = 20.0
    self.wristVelGain = 0.3
    self.wristMaxForce = 50.

    # command all joint to their default pose
    self.resetPose()

    # load force torque sensors
    self.rightFT = FTSensor(self.gripperUid, self.rightFingerPadIndex)
    self.leftFT = FTSensor(self.gripperUid, self.leftFingerPadIndex)

    # let system settle and zero the force sensor
    self.sleeper.sleepSim(1, self.step)

    self.rightFT.zeroSensorBias()
    self.leftFT.zeroSensorBias()

    self.currMode = 0


  def getWristForce(self):
    wristForce = p.getJointState(self.gripperUid, self.wristXIndex)[2][0]
    return self.filt.filter(wristForce)


  def changeControlMode(self, mode):
    if (mode != self.currMode):
      # torque control is 1
      if (mode == 0):
        print("changed to position control")
        # turn off position control on wrist joints
        p.setJointMotorControl2(self.gripperUid,
                                self.wristXIndex,
                                p.POSITION_CONTROL,
                                force=10.0)

        p.setJointMotorControl2(self.gripperUid,
                                self.wristYIndex,
                                p.POSITION_CONTROL,
                                force=10.0)

        p.setJointMotorControl2(self.gripperUid,
                                self.wristWIndex,
                                p.POSITION_CONTROL,
                                force=10.0)
      elif (mode == 1):
        print("changed to force control")
        # get current reference pose
        self.desPos = [x[0] for x in p.getJointStates(self.gripperUid,
                                [self.wristXIndex,
                                self.wristYIndex,
                                self.wristWIndex])]

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
    self.currMode = mode

  def makeContactControl(self):
    self.changeControlMode(0)
    dxDes = 0.01
    wristPos = p.getJointState(self.gripperUid, self.wristXIndex)[0]
    wristPosDes = wristPos + 0.02*dxDes

    # move the prismatic wrist to new desired position
    p.setJointMotorControl2(self.gripperUid, self.wristXIndex,
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition = wristPosDes,
                            targetVelocity = 0.,
                            force=100.0)


  def stepAdmittanceControl(self):
    dxDes = 1.0*self.wristForce
    wristPos = p.getJointState(self.gripperUid, self.wristXIndex)[0]
    wristPosDes = wristPos + 0.002*dxDes

    # move the prismatic wrist to new desired position
    p.setJointMotorControl2(self.gripperUid, self.wristXIndex,
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition = wristPosDes,
                            targetVelocity = 0.,
                            force=0.5)
      

  def stepFingerComplianceControl(self):
    self.changeControlMode(1)

    u = np.zeros(3)
    # perform explicit position control
    u[0] = self.wristXPDController.computePD( self.desPos[0],
                                              0.,
                                              self.wristPosGain,
                                              self.wristVelGain,
                                              self.wristMaxForce)

    u[1] = self.wristYPDController.computePD( self.desPos[1],
                                              0.,
                                              self.wristPosGain,
                                              self.wristVelGain,
                                              self.wristMaxForce)

    u[2] = self.wristWPDController.computePD( self.desPos[2],
                                              0.,
                                              self.wristPosGain,
                                              self.wristVelGain,
                                              self.wristMaxForce)
    p.setJointMotorControlArray(self.gripperUid, 
                            [self.wristXIndex, 
                            self.wristYIndex,
                            self.wristWIndex],
                            controlMode=p.TORQUE_CONTROL, 
                            forces=u)


    # get finger force vector (6x1)
    ffl = self.leftFT.getForces()
    ffr = self.rightFT.getForces()

    # set force desired
    f_des = np.zeros(len(ffl))
    f_des[1] = 0.2

    # get jacobian to joints
    fingerPos = p.getJointState(self.gripperUid, self.rightFingerIndex)[0]
    Jqr = np.zeros((3,6))
    Jqr[0,1] = 1.
    Jqr[1,2] = 1.
    Jqr[2,2] = -0.1034
    Jqr[2,3] = fingerPos
    Jqr[2,4] = 1.

    Jql = np.zeros((3,3))
    Jql[0,0] = 1.
    Jql[1,1] = 1.
    Jql[1,2] = -fingerPos
    Jql[0,2] = -0.1034 - 0.025
    Jql[2,2] = 1.
    #Jql[0,1] = 1.
    #Jql[1,2] = 1.
    #Jql[2,2] = -0.1034
    #Jql[2,3] = -fingerPos
    #Jql[2,4] = 1.

    #fl_err = (f_des - ffl)
    fl_err = (f_des - ffl)[1:4]
    fr_err = (f_des - ffr)
    Kl = 0.0001*np.eye(3)
    Kl[1,1] = 0
    Kl[2,2] = 0.5
    Kr = 0.0001*np.eye(6)
    #ul = Jql.T.dot(Kl.dot(self.Jtip.dot(fl_err)))
    dx_des = Kl.dot(fl_err)
    ul = np.linalg.pinv(Jql).dot(dx_des)
    ur = Jqr.dot(Kr.dot(self.Jtip.dot(fr_err)))

    self.desPos += ul
    #print("ul {}".format(ul))
    #print("pos des {}".format(self.desPos))
    print("f {}".format(ffl))
    #print("u {}".format(u))
    # move the prismatic wrist to new desired position
    #p.setJointMotorControlArray(self.gripperUid, 
    #                        [self.wristXIndex, 
    #                        self.wristYIndex,
    #                        self.wristWIndex],
    #                        controlMode=p.TORQUE_CONTROL, 
    #                        forces=u)


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


    # reset fingers to home position
    self.setJointControl(qDes = 0.04,
                          maxForce = self.gripperMaxForce)

  def setJointControl(self, qDes=0.04, maxForce=100.):
    self.gripperQDes = qDes
    self.gripperMaxForce = maxForce

  def step(self):
    # perform explicit position control
    force = self.fingerPDController.computePD( self.gripperQDes,
                                              0.,
                                              self.gripperPosGain,
                                              self.gripperVelGain,
                                              self.gripperMaxForce)
    
    p.setJointMotorControl2(self.gripperUid,
                            self.rightFingerIndex,
                            p.TORQUE_CONTROL,
                            force=force)


    self.leftFT.step()
    self.rightFT.step()

    #self.wristForce = self.filt.filter(-self.rightFT.getForces()[1]-self.leftFT.getForces()[1])

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
                        parentLinkIndex=self.wristWIndex )
    p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0], parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.wristWIndex )
    p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.wristWIndex )
    p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1], parentObjectUniqueId=self.gripperUid,
                        parentLinkIndex=self.wristWIndex )


""" Class for Force torque sensors mounted at the finger pads. Implements functions to
detemine if sensors are in contact, getting a filtered signal and displaying what the
contact forces are.
"""
class FTSensor(object):
  def __init__(self, gripperUid, linkIndex):
    self.gripperUid = gripperUid
    self.sensorIndex = linkIndex
    self.loopCounter = 0

    # enable force torque sensors
    p.enableJointForceTorqueSensor(self.gripperUid, self.sensorIndex)


    # initialize line to visualize FT
    self.ftLineId = p.addUserDebugLine([0,0,0],[0,0.1,0],[0,0,0], 
                        parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.sensorIndex)

    # initialize line to visualize contact center of pressure
    self.cpLineId = p.addUserDebugLine([0,0,0],[0,0.01,0],[1,0,0], 
                        parentObjectUniqueId=self.gripperUid, 
                        parentLinkIndex=self.sensorIndex)

    self.bias = np.zeros(6)

    self.contactState = False
    self.contactStateCounter = 0

    self.sensor_thickness = 0.00929   # sensor thickness in m
    # filter for wrist force sensor
    # 2nd order butterworth at fc=300 Hz and fs=1000 Hz
    #self.filt = lowPassFilter(2, 300, 1000, vecSize=6)


  def getForces(self):
    #return self.filt.filter(p.getJointState(self.gripperUid, self.sensorIndex)[2])
    self.forces = p.getJointState(self.gripperUid, self.sensorIndex)[2] - self.bias
    return self.forces


  def getCP(self):
    """ Use f/t data and finger geometry to estimate the center of pressure through
        intrinsic tactile sensing. 
            r = h - alpha*f 
            h = (f_hat x m_hat)* ||f||/||m||

    Returns:
      vec3: location of contact center of pressure in the finger pad ref frame.

    """
    # get force and moment data
    f = self.forces[:3]
    m = self.forces[3:]
    f_mag = np.linalg.norm(f)
    m_mag = np.linalg.norm(m)
    # check that these are not zero
    if (f_mag == 0 or m_mag == 0):
      raise Exception("F/T data is not good for intrinsic tactile sensing.\
                       f: {}, m: {}".format(f_mag, m_mag))
    f_hat = -f/f_mag
    m_hat = -m/m_mag
    h = np.cross(f_hat, m_hat)*m_mag/f_mag
    alpha = (h[1] - self.sensor_thickness)/f[1]
    r = h - alpha*f
    return r


  def _getContactState(self, thresh=0.1):
    """ Return true or false depending on whether the sensor is in contact.
        Only using force, not moments.

    Returns:
      bool: indicates whether sensor is in contact.

    """
    f_mag = np.linalg.norm(self.forces[:3])
    in_contact  = f_mag > thresh
    if in_contact:
      self.contactStateCounter = min(self.contactStateCounter+1, 100)
    #else:
    #  self.contactStateCounter = max(self.contactStateCounter-1, 0)
    return self.contactStateCounter > 50

  def getContactState(self):
    return self.contactState

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
                        replaceItemUniqueId=self.ftLineId)

  def _displayCP(self):
    """ Display the estimated center of pressure """
    if (self._getContactState()):
      r = self.getCP()
      #r[1] = 0
      p.addUserDebugLine([0,0,0],r,[0,1,0], 
                          parentObjectUniqueId=self.gripperUid, 
                          parentLinkIndex=self.sensorIndex,
                          replaceItemUniqueId=self.cpLineId)
    else:
      p.addUserDebugLine([0, 0, 0],[0, 0, 0],[1,0,0], 
                          parentObjectUniqueId=self.gripperUid, 
                          parentLinkIndex=self.sensorIndex,
                          replaceItemUniqueId=self.cpLineId)
    

  def step(self):
    self.getForces()
    self.loopCounter += 1
    if (self.loopCounter % 20) == 0:
      self._displayForceVector()
      #self._displayCP()
    currContactState = self._getContactState()
    if (currContactState != self.contactState):
      if currContactState:
        p.changeVisualShape(self.gripperUid,
                            self.sensorIndex,
                            rgbaColor=[1,0,0,.1])
      else:
        p.changeVisualShape(self.gripperUid,
                            self.sensorIndex,
                            rgbaColor=[0,1,0,.1])
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
