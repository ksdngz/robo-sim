from typing import Union
import numpy as np
from motioncon import motionControllerService as mcs
from common.pose3d import Pose3d

class MultiSignedProfiler:
    @classmethod
    def generateTraj(cls,
                     startPos : Union[np.ndarray, Pose3d],
                     targetPos : Union[np.ndarray, Pose3d],
                     steps : int
                     ) -> Union[mcs.mr.mo.Trajectory, None] :
        # check type
        if type(startPos) != type(targetPos):
            print("generateTraj: input type error", "s:", type(startPos), "t:", type(targetPos))
            return None
        
        if type(startPos) == Pose3d:
            s = startPos.eul()
            t = targetPos.eul()
        elif type(startPos) == np.ndarray:
            s = startPos
            t = targetPos
        else:
            print("generateTraj: unknown type", "s:", type(startPos))
            return None
            
        traj = mcs.mr.mo.Trajectory()
        for i in range(steps):
            rate : float = float(i/steps)
            p : np.ndarray = s + (t-s)*(1 - np.cos(np.pi*rate))/2
            point = mcs.mr.mo.Point(i, p) # [t, p]
            traj.push(point)
        return traj
