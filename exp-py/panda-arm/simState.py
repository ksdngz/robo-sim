from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation
import debugger.dataLogger as dl
from external.kinetics import Kinematics as kin
from spatialmath import SE3
from spatialmath.base import tr2rpy, rpy2tr
from common import time_recorder as tr
from common.pose6d import Pose6d

class MotionState(Enum):
    IDLE    = 0
    MOVING  = 1

class ControllerState:
    def __init__(self):
        self.__motionState = MotionState.IDLE

    @property
    def motionState(self):
        return self.__motionState
     
    @motionState.setter
    def motionState(self, value):
        self.__motionState = value


class JointState:
    def __init__(self):
        self.q_ = 0 # [deg]
        self.dq_ = 0 # [deg/s]
        self.qcmd_ = 0 # [deg]
        self.dqcmd_ = 0 # [deg]
        self.qtarget_ = 0 # [deg]

    def update(self, q, dq, qcmd, dqcmd):
        self.q_ = np.rad2deg(q)
        self.dq_ = np.rad2deg(dq)
        self.qcmd_ = np.rad2deg(qcmd)
        self.dqcmd_ = np.rad2deg(dqcmd)
            
class TcpState:
    def __init__(self):
        self.__pose = Pose6d()
        self.__cmdPose = Pose6d()

    def update(self, 
               pose: Pose6d,
               cmdPose: Pose6d) -> None:
        self.__pose = pose
        self.__cmdPose = cmdPose
    
    def pose(self) -> Pose6d:
#        pose : list[float] = []
#        pose.extend(self.__pos)
#        pose.extend(self.__rot)
        return self.__pose

    def cmdPose(self) -> Pose6d:
#        pose : list[float] = []
#        pose.extend(self.__pos)
#        pose.extend(self.__rot)
        return self.__cmdPose

class SimState:
    def __init__(self, qSize):
        self.joints_ = [JointState() for i in range(qSize)]
        self.__tcp = TcpState()
        self.qsize_ = qSize
        self.__time = 0
        self.dataLogger = dl.DataLogger()
        self.controllerState = ControllerState()

    def time(self):
        return self.__time
        
    def qs(self):
        return [self.joints_[i].q_ for i in range(self.qsize_)]

    def qdots(self):
        return [self.joints_[i].dq_ for i in range(self.qsize_)]

    def qcmds(self):
        return [self.joints_[i].qcmd_ for i in range(self.qsize_)]

    def qdotcmds(self):
        return [self.joints_[i].dqcmd_ for i in range(self.qsize_)]

    def tcpPose(self) -> Pose6d:
        return self.__tcp.pose()

    def tcpCmdPose(self) -> Pose6d:
        return self.__tcp.cmdPose()

    @tr.time_recorder(__qualname__)
    def update(self, data, qcmd, dqcmd):
        self.__time = self.__time+1
        for i, jnt in enumerate(self.joints_):
            jnt.update(data.qpos[i], data.qvel[i], qcmd[i], dqcmd[i])
        #index_tcp = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'panda_link7')
        index_tcp = 7 # todo to be refactored
        qs : np.ndarray = np.deg2rad(np.array(self.qs())) #[rad]
        p : SE3 = kin.forwardKin(qs) #[rad]
        cmdPose : SE3 = kin.forwardKin(qcmd) #[rad]

#        p_eulzyx : np.ndarray = tr2rpy(p.R, unit='rad', order='zyx')
        #print("p_eulzyx", p_eulzyx)
#        trans = [p.x, p.y, p.z]
#       self.__tcp.update(trans, np.rad2deg(p_eulzyx)) # eul(zyx) [deg]        
        self.__tcp.update(p, cmdPose)

        # to do change from quat to rpy
#        rpy = [0]*3        
#        xmat = np.array(data.xmat[index_tcp])
#        quat = np.array(data.xquat[index_tcp]) # q = (w, x, y, z).
#        q_w = quat[0]
#        q_x = quat[1]
#        q_y = quat[2]
#        q_z = quat[3]
#        newQ = np.array([q_x, q_y, q_z, q_w])
##        rot = Rotation.from_matrix(xmat)
#        rot = Rotation.from_quat(newQ) # x, y, z, w
#        angle = rot.as_euler('zyx', degrees=True)        
#        angle = rot.as_euler('ZYX', degrees=True)        
#        self.__tcp.update(data.xpos[index_tcp], angle)        
        self.dataLogger.log(
            self.time(), 
            self.qs(), self.qdots(), self.qcmds(), self.qdotcmds(), 
            self.tcpPose().eul(), self.tcpPose().eul(), self.tcpCmdPose().eul(), self.tcpCmdPose().eul() ) #todo tcp cmd and vel        

    def startDataLog(self):
        DEFAULT_SIZE = 100
        self.dataLogger.startLog(DEFAULT_SIZE)

    def endDataLog(self):
        self.dataLogger.endLog()
        
    def showDataLog(self):
        self.dataLogger.showLog()
