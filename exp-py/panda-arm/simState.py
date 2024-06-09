#import mujoco as mj
#from mujoco.glfw import glfw
#import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
#import os
#import queue
#import control
#import threading
#import tkinter as tk
#import tkinter.ttk as ttk
#import rtbWrapper as rtb
#from task import taskRequest as taskReq

import debugger.dataLogger as dl

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
        self.__pos : list[float] = [0.]*3
        self.__rot : list[float] = [0.]*3        

    def update(self, 
               pos: list[float], 
               rot: list[float]):
        self.__pos = pos
        self.__rot = rot
    
    def pose(self) -> list[float]:
        pose : list[float] = []
        pose.extend(self.__pos)
        pose.extend(self.__rot)
        return pose

class SimState:
    def __init__(self, qSize):
        self.joints_ = [JointState() for i in range(qSize)]
        self.__tcp = TcpState()
        self.qsize_ = qSize
        self.__time = 0
        self.dataLogger = dl.DataLogger()

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

    def tcpPose(self):
        return self.__tcp.pose()

    def update(self, data, qcmd, dqcmd):
        self.__time = self.__time+1
        for i, jnt in enumerate(self.joints_):
            jnt.update(data.qpos[i], data.qvel[i], qcmd[i], dqcmd[i])
        
        #index_tcp = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'panda_link7')
        index_tcp = 7 # todo to be refactored
        # print('panda_link7 pose', data.xpos[i], data.xquat[i])
        # to do change from quat to rpy
        rpy = [0]*3
        #data.xquat[index_tcp]
        
        quat = np.array(data.xquat[index_tcp])
        rot = Rotation.from_quat(quat)
        angle = rot.as_euler('ZYX', degrees=True)

#        angle2 = rot.as_euler('XYZ', degrees=True)
#        print('angle')
#        print(angle)
#        print('angle2')
#        print(angle2)
        
        
        self.__tcp.update(data.xpos[index_tcp], angle)
        
        self.dataLogger.log(self.time(), self.qs(), self.qdots(), self.qcmds(), self.qdotcmds())        
        #            self.__data.timeBuf.add(state.time())    
        #            self.__data.qBuf.add(state.qs())    
        #            self.__data.qdotBuf.add(state.qdots())            

    def startDataLog(self):
        DEFAULT_SIZE = 100
        self.dataLogger.startLog(DEFAULT_SIZE)

    def endDataLog(self):
        self.dataLogger.endLog()
        
    def showDataLog(self):
        self.dataLogger.showLog()
