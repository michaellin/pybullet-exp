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

    p.setGravity(0, 0, -10)
    self._gripper = FrankaHand(urdfRootPath = self._urdfRoot, 
                      initPos = [0.00000, 0.0000, -0.2], timeStep = self._timeStep)
    # step once first to place the gripper in place
    for i in range(100):
        p.stepSimulation()
        time.sleep(self._timeStep)

    xpos = 0.15 - np.abs(0.03 * random.random())
    ypos = 0 + 0.04 * random.random()
    ang = 3.14 * 0.5 + 3.1415925438 * random.random()
    orn = p.getQuaternionFromEuler([0, 1.57079632679, ang])
    self.blockUid = p.loadURDF(os.path.join(self._urdfRoot, "block.urdf"), 
                                xpos, ypos, -0.15,
                                orn[0], orn[1], orn[2], orn[3])


    p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw = -90., cameraPitch = -89.,
    #p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw = 45., cameraPitch = 0.,
                      cameraTargetPosition=[0.1, 0., 0.])
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
  closed = False
  while (stepCount < fh_env._maxSteps):
      fingerPos = 0.02*(np.sin(2*np.pi*stepCount/100)+1)
      if (stepCount > 1000):
          if (not closed):
              fh_env._gripper.applyAction(fingerPos)
          if (fingerPos == 0):
              closed = True
      stepCount += 1
      p.stepSimulation()
      time.sleep(fh_env._timeStep)
