import numpy as np
from motioncon import motionControllerService as mcs

class MultiSignedProfiler:
    @classmethod
    def generateTraj(cls,
                     startPos : np.ndarray,  
                     targetPos : np.ndarray,
                     steps : int
                     ) -> mcs.mr.mo.Trajectory :
        traj = mcs.mr.mo.Trajectory()
        for i in range(steps):
            rate : float = float(i/steps)
            p : np.ndarray = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
            point = mcs.mr.mo.Point(i, p) # [t, p]
            traj.push(point)
        return traj
