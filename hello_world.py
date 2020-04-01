from frankaHandGymEnv import FrankaHandGymEnv
import pybullet as p
import time
import math


def main():

  env = FrankaHandGymEnv(performGraspFunc = contactFirstGrasp)
  counter = 0
  while (counter < 10000):
      total_attempts, total_success = env.performGrasp()
      print("lifted {}% trials".format(100.0*total_success/total_attempts))
      env.resetExp()

def simpleGrasp(self):
      self._gripper.applyAction(0, maxForce=100.)
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
      while (not done and counter < 5*240):
          counter += 1
          leftFingerF = self._gripper.leftFingerF
          rightFingerF = self._gripper.rightFingerF
          if (abs(leftFingerF[1]) > 0.03) and \
            (abs(rightFingerF[1]) > 0.03):
              done = True
          
          #print("wirst force {}".format(self._gripper.getWristForce()))
          #print("L finger force {}".format(leftFingerF[1]))
          #print("R finger force {}".format(rightFingerF[1]))
          print("L {} R {}".format(leftFingerF[1],rightFingerF[1]))
          if not done:
              if (abs(leftFingerF[1]) > 0.03) is not \
                (abs(rightFingerF[1]) > 0.03):
                  self._gripper.stepAdmittanceControl()

          self._gripper.step()
          p.stepSimulation()
          time.sleep(self._timeStep)

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
