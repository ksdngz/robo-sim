from typing import Union
import numpy as np
from spatialmath import SE3
from spatialmath.base import tr2rpy, rpy2tr
from common import common_constants as const

def rot_deg2rad(pose : np.ndarray):
    out = pose
    for i in range(3,6):
        out[i] = np.deg2rad(pose[i])
    return out

class Pose6d:
    # In Pose6d, all of units are defined as radian.
                
    def __init__(self, 
                 pose : Union[np.ndarray, SE3, list[float], None] = None):
        if type(pose) == np.ndarray or type(pose) == list:
            if len(pose) is not const.CARTESIAN_POSE_SIZE:
                print("Pose6d: Warning, size of pose is not correct.")
                print("type:", type(pose), "size:", len(pose))
                self.__pose = SE3() # identity
            else:                
                trans = SE3.Trans(pose[0:3])
                rot = SE3.RPY(pose[3:6], order="zyx")
                self.__pose = trans * rot
        elif type(pose) == SE3:
            self.__pose = pose
        else:
            self.__pose = SE3() # identity
   
    def eul(self, unit="rad") -> np.ndarray: # zyx euler [rad]
        t : np.ndarray = self.__pose.t
        eul : np.ndarray = tr2rpy(self.__pose.R, unit='rad', order='zyx')
        eul = rot_deg2rad(eul) if unit == "deg" else eul
        return np.concatenate([t, eul])

    def se3(self) -> SE3:
        return self.__pose
    
    def __add__(self, other):
        if isinstance(other, Pose6d):        
            return self.__pose + other.__pose
        raise TypeError()    

    def __sub__(self, other):
        if isinstance(other, Pose6d):        
            return self.__pose - other.__pose
        raise TypeError()    

    def __mul__(self, other):
        if isinstance(other, Pose6d):        
            return self.__pose * other.__pose
        raise TypeError()