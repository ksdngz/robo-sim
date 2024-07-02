import numpy as np
from common.pose6d import Pose6d
from external import rtbWrapper as rtb

class Kinematics:
    def forwardKin(q : np.ndarray) -> Pose6d:
        return rtb.forwardKin(q)

    def inverseKin(
        p : Pose6d, #[rad]
        q0 : np.ndarray) -> np.ndarray: #[rad] -> [rad]
        return rtb.inverseKin(p, q0)

class Dynamics:
    def calcGravComp(q : np.ndarray) -> np.ndarray:
        return rtb.calcGravComp(q)

