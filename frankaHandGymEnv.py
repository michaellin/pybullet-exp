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

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               actionRepeat=1,
               isEnableSelfCollision=True,
               maxSteps=1000):
    self._timeStep = 1. / 240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._envStepCounter = 0
    self._maxSteps = maxSteps
    self._cam_dist = 1.3
    self._cam_yaw = 180
    self._cam_pitch = -40

    self._p = p
    p.connect(p.GUI)
    self.reset()


  def reset(self):
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])

    p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
               0.000000, 0.000000, 0.0, 1.0)

    xpos = 0.55 + 0.12 * random.random()
    ypos = 0 + 0.2 * random.random()
    ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    orn = p.getQuaternionFromEuler([0, 0, ang])
    self.blockUid = p.loadURDF(os.path.join(self._urdfRoot, "block.urdf"), xpos, ypos, -0.15,
                               orn[0], orn[1], orn[2], orn[3])

    p.setGravity(0, 0, -10)
    self._gripper = FrankaHand(urdfRootPath = self._urdfRoot, timeStep = self._timeStep)

    p.stepSimulation()

  def __del__(self):
    p.disconnect()

  def step(self, action):
    dv = 0.005
    dx = action[0] * dv
    realAction = [dx]
    return realAction


if __name__ == "__main__":
  fh_env = FrankaHandGymEnv(maxSteps=10000)
  stepCount = 0
  while (stepCount < fh_env._maxSteps):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      fh_env._gripper.applyAction([fingerPos]*2)
      stepCount += 1
      p.stepSimulation()
      time.sleep(fh_env._timeStep)
