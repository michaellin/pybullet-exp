import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import gym
import os
import time
import random
from frankaHand import FrankaHand

from utils import sleeper

class FrankaHandGymEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, urdfRoot=pybullet_data.getDataPath(), performGraspFunc=None):
    self._timeStep = 1. / 1000.
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
    self.sleeper = sleeper(self._p, self._timeStep)
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
    self.gripper = FrankaHand(urdfRootPath = self._urdfRoot, 
                      initPos = [0.00000, 0.0000, -0.2], timeStep = self._timeStep)

    # step once first to place the gripper in place
    self.sleeper.sleepSim(1, self.gripper.step)

    self.placeRandBlock()

    self.sleeper.sleepSim(1, self.gripper.step)

  def placeRandBlock(self):
    # randomly generate a 3d pose of the block to grasp
    # limit is at xpos = 0.215, center is xpos = 0.195
    #xpos = 0.198 + 0.035 * (random.random()-0.5)
    #ypos = 0 + 0.056 * (random.random()-0.5)
    #ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    #xpos = 0.195 
    #ypos = -0.015
    #ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    #orn = p.getQuaternionFromEuler([0, 1.57079632679, ang])
    #self.blockUid = p.loadURDF("data/block.urdf", 
    #                            xpos, ypos, -0.0149,
    #                            orn[0], orn[1], orn[2], orn[3])

    xpos = 0.195 + 0.065
    #xpos = 0.195 + 0.065 + 0.025 * (random.random()-0.5)
    ypos = 0.02
    #ypos = 0+ 0.055 * (random.random()-0.5)
    q = R.from_euler('xyz', [90, 0, 180], degrees=True).as_quat()
    #self.mugUid = p.loadURDF("data/mug.urdf", xpos, ypos, -0.18,
    #                    q[0], q[1], q[2], q[3])
    self.mugUid = p.loadSDF("data/mug.sdf",
                        useMaximalCoordinates = False)[0]

    # set the position of the base to be on the table
    p.resetBasePositionAndOrientation(self.mugUid, [xpos, ypos, -0.15], q)


  def getBlockHeight(self):
      #pose = p.getBasePositionAndOrientation(self.blockUid)
      pose = p.getBasePositionAndOrientation(self.mugUid)
      return pose[0][2]
  
  def resetExp(self):
    # remove the block
    #p.removeBody(self.blockUid)
    p.removeBody(self.mugUid)

    # reset gripper position
    self.gripper.resetPose()

    # step once first to place the gripper in place
    self.sleeper.sleepSim(1, self.gripper.step)

    self.placeRandBlock()

    self.sleeper.sleepSim(1, self.gripper.step)


  def __del__(self):
    p.disconnect()

  #def sleepSim(self, sleepTime):
  #  """ Step through the simluation for some amount of time. """
  #  steps = int(round(sleepTime / self._timeStep))
  #  # step once first to place the gripper in place
  #  for i in range(steps):
  #      self.gripper.step()
  #      p.stepSimulation()
  #      time.sleep(self._timeStep)

  def performGrasp(self, args):
      return self._performGrasp(self, args)



if __name__ == "__main__":
  stepCount = 0
  fh_env = FrankaHandGymEnv()
  while (True):
      self.total_attempts += 1

      fh_env.gripper.applyAction(0)
      fh_env.sleepSim(1) 
      fh_env.gripper.liftObject()
      fh_env.sleepSim(0.5) 
      fh_env.gripper.stopLiftObject()
      fh_env.sleepSim(0.2) 
      if (fh_env.getBlockHeight() > -0.05):
          self.total_success += 1
      print("lifted {}% trials".format(100.0*self.total_success/self.total_attempts))
      fh_env.sleepSim(1) 
      fh_env.resetExp()
