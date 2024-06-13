import numpy as np
from motioncon import motionControllerService as mcs

class MultiSignedProfiler:
    @classmethod
    def generateTraj(cls,
                     startPos : np.ndarray,  # [deg]
                     targetPos : np.ndarray,  # [deg]
                     steps : int
                     ) -> mcs.mr.mo.Trajectory :   # points[rad]
        traj = mcs.mr.mo.Trajectory()
        for i in range(steps):
            rate : float = float(i/steps)
            p : np.ndarray = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
            point = mcs.mr.mo.Point(i, np.deg2rad(p)) # [t, p[rad]]
            traj.push(point)
        return traj
