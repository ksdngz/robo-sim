import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import queue
import control
import threading
import tkinter as tk
import tkinter.ttk as ttk
import rtbWrapper as rtb
import taskRequest as taskReq

import dataLogger as dl

class JointState:
    def __init__(self):
        self.q_ = 0 # [deg]
        self.dq_ = 0 # [deg/s]
        self.qtarget_ = 0 # [deg]

    def updateBySimulation(self, q, dq):
        self.q_ = np.rad2deg(q)
        self.dq_ = np.rad2deg(dq)
            
class SimState:
    def __init__(self, qSize):
        self.joints_ = [JointState() for i in range(qSize)]
        self.qsize_ = qSize
        self.__time = 0
        self.dataLogger = dl.DataLogger()

    def time(self):
        return self.__time
        
    def qs(self):
        return [self.joints_[i].q_ for i in range(self.qsize_)]

    def qdots(self):
        return [self.joints_[i].dq_ for i in range(self.qsize_)]

    def updateBySimulation(self, data):
        self.__time = self.__time+1
        for i, jnt in enumerate(self.joints_):
            jnt.updateBySimulation(data.qpos[i], data.qvel[i])
        
        self.dataLogger.log(self.time(), self.qs(), self.qdots())        
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
