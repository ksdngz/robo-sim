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

import simState as ss
import common as cm

class LogData:
    def __init__(self):
        MAX_LOGGING_SIZE = 100000
        self.timeBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qdotBuf = cm.RingBuffer(MAX_LOGGING_SIZE)

class Graph:
    def __init__(self):
        return
    
    def __addPlot(self, fig, pos, xs, ys, sName, 
                  sColor='blue', xLbl ='x', yLbl='y'):
        ax = fig.add_subplot(pos[0], pos[1], pos[2])
        ax.plot(xs, ys, color=sColor, label=sName)
        ax.set_xlabel(xLbl)
        ax.set_ylabel(yLbl)
        ax.legend(loc = 'upper right') 
        return ax
    
    def show(self, data : LogData):
        t = data.timeBuf.getList()
        qs = cm.transpose(data.qBuf.getList())
        qdots = cm.transpose(data.qdotBuf.getList())
        
        # Figure construction        
        fig = plt.figure(figsize = (22,5), facecolor='lightblue')
        row = 2
        col = max(len(qs), len(qdots))
        plotCount = 0
        # plot: q
        axs_q = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qs[i], 'J'+str(i+1)+' pos',
                                sColor='blue', xLbl ='[cyc]', yLbl='[deg]') for i in range(len(qs))]
        plotCount = plotCount + len(qs)
        # plot: qdot
        axs_qdots = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qdots[i], 'J'+str(i+1)+' vel',
                                    sColor='green', xLbl ='[cyc]', yLbl='[deg/s]') for i in range(len(qdots))]
        plotCount = plotCount + len(qdots)

        fig.tight_layout()
        plt.show()
    
    def quickShow(xSeries,ySeries):
        plt.plot(xSeries, ySeries)
        plt.show()        

## DataLogger
class DataLogger:
    def __init__(self):
#        self.__state = state
        self.__data = LogData()
        self.__enabled = False
        self.__size = 0

    def startLog(self, size):
        if self.__enabled:
            print('startLog Error: logging is already enabled.')
            return
        # temp
        size = 100000
        self.__data.timeBuf = cm.RingBuffer(size)
        self.__data.qBuf = cm.RingBuffer(size)
        self.__data.qdotBuf = cm.RingBuffer(size)
        self.__size = size
        self.__enabled = True

#    def log(self, state : ss.SimState):
#        if self.__enabled:
#            self.__data.timeBuf.add(state.time())    
#            self.__data.qBuf.add(state.qs())    
#            self.__data.qdotBuf.add(state.qdots())    

    def log(self, time, qs, qdots):
        if self.__enabled:
            self.__data.timeBuf.add(time)    
            self.__data.qBuf.add(qs)    
            self.__data.qdotBuf.add(qdots)    

    def endLog(self):
        self.__enabled = False

    def getLog_q(self):
        return self.__data.qBuf.getList()

    def getLog_qdot(self):
        return self.__data.qdotBuf.getList()

    def showLog(self):
        g = Graph()
        g.show(self.__data)

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
        self.dataLogger = DataLogger(self)

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
        
        self.dataLogger.log()

    def startDataLog(self):
        DEFAULT_SIZE = 100
        self.dataLogger.startLog(DEFAULT_SIZE)

    def endDataLog(self):
        self.dataLogger.endLog()
        
    def showDataLog(self):
        self.dataLogger.showLog()
