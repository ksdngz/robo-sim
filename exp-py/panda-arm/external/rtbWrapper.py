import roboticstoolbox as rtb
import numpy as np
from common.pose6d import Pose6d

robot = rtb.models.DH.Panda()
#print(robot)
tau = robot.rne(robot.q, np.zeros((7,)), np.zeros((7,)))
i = robot.inertia(robot.q)
c = robot.coriolis(robot.q, robot.qd)
g = robot.gravload(robot.q)
#print("tau:", tau)
#print("inertia:", i)
#print("coriolis:", c)
#print("grav:", g)

def calcGravComp(q : np.ndarray) -> np.ndarray:
    robot = rtb.models.DH.Panda()
    g = robot.gravload(q)
    return g

def inverseKin(
    p : Pose6d, #[rad]
    q0 : np.ndarray) -> np.ndarray: #[rad] -> [rad]
    robot = rtb.models.DH.Panda()
    ret = robot.ik_LM(p.se3(), q0=q0) # tuple (q, success, iterations, searches, residual)
    q : np.ndarray = ret[0]
    success : bool = ret[1]
    if success == 0:
        print('rtb ik solution was found but in error.')
#    print('rtb ik solution::')
#    print(np.rad2deg(q))
    return q

def forwardKin(q : np.ndarray) -> Pose6d:
    robot = rtb.models.DH.Panda()
    return Pose6d(robot.fkine(q))

# sample inverse dynamics
#puma = rtb.models.DH.Puma560()
#tau = puma.rne(puma.qn, np.zeros((6,)), np.zeros((6,)))
#puma.inertia(q)
#puma.coriolis(q, qd)
#puma.gravload(q)
#print(tau)