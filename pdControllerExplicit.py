import numpy as np


class PDControllerExplicit(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndex, qdes, qdotdes, kps, kds, maxForce):
      jointState = self._pb.getJointState(bodyUniqueId, jointIndex)
      q = jointState[0]
      qdot = jointState[1]

      qError = qdes - q
      qdotError = qdotdes - qdot
      Kp = kps
      Kd = kds
      force = Kp*qError + Kd*qdotError
      force = np.clip(force, -maxForce, maxForce)                                                            
      return force
