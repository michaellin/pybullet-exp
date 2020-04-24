import numpy as np


class PDControllerExplicit(object):

  def __init__(self, pb, bodyUniqueId, jointIndex):
    self._pb = pb
    self.bodyId = bodyUniqueId
    self.jointIndex = jointIndex

  def computePD(self, qdes, qdotdes, kps, kds, maxForce):
      jointState = self._pb.getJointState(self.bodyId, 
                                          self.jointIndex)
      q = jointState[0]
      qdot = jointState[1]

      qError = qdes - q
      qdotError = qdotdes - qdot
      Kp = kps
      Kd = kds
      force = Kp*qError + Kd*qdotError
      force = np.clip(force, -maxForce, maxForce)                                                            
      return force
