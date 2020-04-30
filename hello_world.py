from frankaHandGymEnv import FrankaHandGymEnv
import pybullet as p
import time
import math
import numpy as np
import argparse

def main(args):

  #env = FrankaHandGymEnv(performGraspFunc = dummyControl)
  env = FrankaHandGymEnv(performGraspFunc = compliantGrasp)
  #env = FrankaHandGymEnv(performGraspFunc = admittanceControlGrasp)
  #env = FrankaHandGymEnv(performGraspFunc = simpleGrasp)
  env.fileCounter = 0
  while (True):
    total_attempts, total_success = env.performGrasp(args)
    print("lifted {}% trials".format(100.0*total_success/total_attempts))
    env.resetExp()


def dummyControl(self, args):
  while(True):
    self.gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)

def compliantGrasp(self, args):
  """ Function that controls gripper to close the gripper while performing
      compliance control on the finger pads.
  """
  #self.gripper.applyAction(0, maxForce=0.25)

  counter = 0
  done = False
  leftForces = []
  rightForces = []
  while (not done):
    leftFT = self.gripper.leftFT
    rightFT = self.gripper.rightFT
    
    if (leftFT.getContactState()) or (rightFT.getContactState()):
      # if either finger is in contact then we switch to the
      # next stage. Compliance control
      #self.gripper.applyAction(0, maxForce=0.0000001)
      self.gripper.stepFingerComplianceControl()
    else:
      # if not then just pick a direction left or right and move that way
      self.gripper.makeContactControl()

    self.gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)

    # if data logging flag is on
    if (args.log):
      leftForces += [leftFingerF]
      rightForces += [rightFingerF]

  if (args.log):
    np.save("left{}.npy".format(self.fileCounter), np.array(leftForces))
    np.save("right{}.npy".format(self.fileCounter), np.array(rightForces))
    self.fileCounter += 1

  print("out! {}".format(counter))
  self.gripper.applyAction(0, maxForce=100.)
  self.sleepSim(0.2) 
  self.gripper.liftObject()
  self.sleepSim(0.5) 
  self.gripper.stopLiftObject()
  self.sleepSim(0.2) 
  self.total_attempts += 1
  if (self.getBlockHeight() > -0.05):
    self.total_success += 1

  return self.total_attempts, self.total_success

def admittanceControlGrasp(self, args):
  """ Function that controls gripper to close the gripper while performing
      admittance control on the wrist X axis.
  """
  self.gripper.applyAction(0, maxForce=0.25)
  counter = 0
  done = False
  leftForces = []
  rightForces = []
  while (not done and counter < 5*240):
    counter += 1
    leftFingerF = self.gripper.leftFingerF
    rightFingerF = self.gripper.rightFingerF

    # if data logging flag is on
    if (args.log):
      leftForces += [leftFingerF]
      rightForces += [rightFingerF]
    
    if (abs(leftFingerF[1]) > 0.03) and \
      (abs(rightFingerF[1]) > 0.03):
      done = True
    
    #print("wirst force {}".format(self.gripper.getWristForce()))
    #print("L {} R {}".format(leftFingerF[1],rightFingerF[1]))
    if not done:
      if (abs(leftFingerF[1]) > 0.03) is not \
        (abs(rightFingerF[1]) > 0.03):
          self.gripper.stepAdmittanceControl()

    self.gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)

  if (args.log):
    np.save("left{}.npy".format(self.fileCounter), np.array(leftForces))
    np.save("right{}.npy".format(self.fileCounter), np.array(rightForces))
    self.fileCounter += 1

  print("out! {}".format(counter))
  self.gripper.applyAction(0, maxForce=100.)
  self.sleepSim(0.2) 
  self.gripper.liftObject()
  self.sleepSim(0.5) 
  self.gripper.stopLiftObject()
  self.sleepSim(0.2) 
  self.total_attempts += 1
  if (self.getBlockHeight() > -0.05):
    self.total_success += 1

  return self.total_attempts, self.total_success

def simpleGrasp(self, args):
  """ Function that controls gripper to simply close the gripper all
      the way without other feedback 
  """
  done = False
  counter = 0
  while not done:
    counter += 1
    self.gripper.applyAction(0, maxForce=0.25)
    self.gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)
    leftFingerF = self.gripper.leftFingerF
    rightFingerF = self.gripper.rightFingerF
    if ((abs(leftFingerF[1]) > 0.03) or \
      (abs(rightFingerF[1]) > 0.03) or \
      counter > 5*240):
      self.gripper.applyAction(0, maxForce=100.)
      done = True
  self.sleepSim(1) 
  self.gripper.liftObject()
  self.sleepSim(0.5) 
  self.gripper.stopLiftObject()
  self.sleepSim(0.2) 
  self.total_attempts += 1
  if (self.getBlockHeight() > -0.05):
    self.total_success += 1

  return self.total_attempts, self.total_success

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument(
        "--log",
        type=str,
        default=None,
        help="log contact force data")
  args = parser.parse_args()
  main(args)
