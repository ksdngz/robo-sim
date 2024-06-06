import roboticstoolbox as rtb
import numpy as np

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

def calcGravComp(q):
    robot = rtb.models.DH.Panda()
    g = robot.gravload(q)
    return g

# sample inverse dynamics
#puma = rtb.models.DH.Puma560()
#tau = puma.rne(puma.qn, np.zeros((6,)), np.zeros((6,)))
#puma.inertia(q)
#puma.coriolis(q, qd)
#puma.gravload(q)
#print(tau)