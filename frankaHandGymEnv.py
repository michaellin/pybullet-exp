import pybullet_data
import numpy as np
import pybullet as p
import gym
import os
import time
import random
from frankaHand import FrankaHand

class FrankaHandGymEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, urdfRoot=pybullet_data.getDataPath(), performGraspFunc=None):
    self._timeStep = 1. / 240.
    self._urdfRoot = urdfRoot
    self._performGrasp = performGraspFunc

    # camera parameters
    self._camDist = 0.25
    self._camYaw = -90
    self._camPitch = -89
    self._camTargetPosition = [0.1, 0., 0.]

    # experiment state variables
    self.total_attempts = 0
    self.total_success = 0

    self._p = p
    p.connect(p.GUI)
    self.reset()


  def reset(self):
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)

    # load plane 1 meter below 0
    p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -2])

    # load experiment table
    self.tableUid = p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), 
              0.5000000, 0.00000, -.820000,
               0.0000, 0.0000, 0.0, 1.0)

    # change the view point
    p.resetDebugVisualizerCamera(cameraDistance=self._camDist, cameraYaw=self._camYaw, 
                      cameraPitch=self._camPitch, cameraTargetPosition=self._camTargetPosition)

    p.setGravity(0, 0, -10)

    # instantiate the franka Gripper
    self._gripper = FrankaHand(urdfRootPath = self._urdfRoot, 
                      initPos = [0.00000, 0.0000, -0.2], timeStep = self._timeStep)

    # step once first to place the gripper in place
    self.sleepSim(1)

    self.placeRandBlock()

    self.sleepSim(1)

  def placeRandBlock(self):
    # randomly generate a 3d pose of the block to grasp
    # limit is at xpos = 0.215, center is xpos = 0.195
    xpos = 0.198 + 0.035 * (random.random()-0.5)
    ypos = 0 + 0.056 * (random.random()-0.5)
    ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    #xpos = 0.195 
    #ypos = -0.015
    #ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    orn = p.getQuaternionFromEuler([0, 1.57079632679, ang])
    self.blockUid = p.loadURDF("block.urdf", 
                                xpos, ypos, -0.0149,
                                orn[0], orn[1], orn[2], orn[3])

  def getBlockHeight(self):
      pose = p.getBasePositionAndOrientation(self.blockUid)
      return pose[0][2]
  
  def resetExp(self):
    # remove the block
    p.removeBody(self.blockUid)

    # reset gripper position
    self._gripper.resetPose()

    # step once first to place the gripper in place
    self.sleepSim(1)

    self.placeRandBlock()

    self.sleepSim(1)


  def __del__(self):
    p.disconnect()

  def sleepSim(self, sleepTime):
    """ Step through the simluation for some amount of time. """
    steps = int(round(sleepTime / self._timeStep))
    # step once first to place the gripper in place
    for i in range(steps):
        self._gripper.step()
        p.stepSimulation()
        time.sleep(self._timeStep)

  def performGrasp(self):
      return self._performGrasp(self)



if __name__ == "__main__":
  stepCount = 0
  fh_env = FrankaHandGymEnv()
  while (True):
      self.total_attempts += 1

      fh_env._gripper.applyAction(0)
      fh_env.sleepSim(1) 
      fh_env._gripper.liftObject()
      fh_env.sleepSim(0.5) 
      fh_env._gripper.stopLiftObject()
      fh_env.sleepSim(0.2) 
      if (fh_env.getBlockHeight() > -0.05):
          self.total_success += 1
      print("lifted {}% trials".format(100.0*self.total_success/self.total_attempts))
      fh_env.sleepSim(1) 
      fh_env.resetExp()
