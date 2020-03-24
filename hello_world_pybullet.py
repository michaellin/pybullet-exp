import pybullet as p
import time
import pybullet_data
from franka_hand import FrankaHand

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
#planeId = p.loadURDF("table/table.urdf")
cubeStartPos = [0,0,0.8]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadSDF("franka_panda/panda_hand2.sdf")[0]
print("=== Robot id {} ===".format(robotId))

numJoints = p.getNumJoints(robotId)
print("=== Number of joints {} ===".format(numJoints))

print("=== Link state {} ===".format(p.getDynamicsInfo(robotId, -1)))

mode = p.POSITION_CONTROL
for i in range(numJoints):
    print("=== Info of joint {}: {} ===".format(i, p.getJointInfo(robotId,i)))
    #p.setJointMotorControl2(robotId, i, controlMode=mode, targetPosition=3.14,
    #                    positionGain = 1, velocityGain = 0.1)

#p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0,
#                      controlMode=mode, targetPosition=0.01, force=100)
#p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1,
#                      controlMode=mode, targetPosition=0.01, force=100)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()


