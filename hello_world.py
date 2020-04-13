from frankaHandGymEnv import FrankaHandGymEnv
import pybullet as p
import time
import math
import numpy as np


def main():

  env = FrankaHandGymEnv(performGraspFunc = contactFirstGrasp)
  #env = FrankaHandGymEnv(performGraspFunc = simpleGrasp)
  env.fileCounter = 0
  while (True):
    total_attempts, total_success = env.performGrasp()
    print("lifted {}% trials".format(100.0*total_success/total_attempts))
    env.resetExp()

def simpleGrasp(self):
  done = False
  counter = 0
  while not done:
    counter += 1
    self._gripper.applyAction(0, maxForce=0.25)
    self._gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)
    leftFingerF = self._gripper.leftFingerF
    rightFingerF = self._gripper.rightFingerF
    if ((abs(leftFingerF[1]) > 0.03) or \
      (abs(rightFingerF[1]) > 0.03) or \
      counter > 5*240):
      self._gripper.applyAction(0, maxForce=100.)
      done = True
  self.sleepSim(1) 
  self._gripper.liftObject()
  self.sleepSim(0.5) 
  self._gripper.stopLiftObject()
  self.sleepSim(0.2) 
  self.total_attempts += 1
  if (self.getBlockHeight() > -0.05):
    self.total_success += 1

  return self.total_attempts, self.total_success

def contactFirstGrasp(self):
  self._gripper.applyAction(0, maxForce=0.25)
  counter = 0
  done = False
  leftForces = []
  rightForces = []
  while (not done and counter < 5*240):
    counter += 1
    leftFingerF = self._gripper.leftFingerF
    rightFingerF = self._gripper.rightFingerF

    leftForces += [leftFingerF]
    rightForces += [rightFingerF]
    
    if (abs(leftFingerF[1]) > 0.03) and \
      (abs(rightFingerF[1]) > 0.03):
      done = True
    
    #print("wirst force {}".format(self._gripper.getWristForce()))
    #print("L {} R {}".format(leftFingerF[1],rightFingerF[1]))
    if not done:
      if (abs(leftFingerF[1]) > 0.03) is not \
        (abs(rightFingerF[1]) > 0.03):
          self._gripper.stepAdmittanceControl()

    self._gripper.step()
    p.stepSimulation()
    time.sleep(self._timeStep)

  np.save("left{}.npy".format(self.fileCounter), np.array(leftForces))
  np.save("right{}.npy".format(self.fileCounter), np.array(rightForces))
  self.fileCounter += 1

  print("out! {}".format(counter))
  self._gripper.applyAction(0, maxForce=100.)
  self.sleepSim(0.2) 
  self._gripper.liftObject()
  self.sleepSim(0.5) 
  self._gripper.stopLiftObject()
  self.sleepSim(0.2) 
  self.total_attempts += 1
  if (self.getBlockHeight() > -0.05):
    self.total_success += 1

  return self.total_attempts, self.total_success

if __name__ == "__main__":
  main()
