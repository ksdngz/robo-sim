import matplotlib.pyplot as plt
import numpy as np
from common import common as cm

class LogData:
    def __init__(self):
        MAX_LOGGING_SIZE = 100000
        self.timeBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qcmdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.qdcmdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)

        self.pBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.pdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.pcmdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)
        self.pdcmdBuf = cm.RingBuffer(MAX_LOGGING_SIZE)

class Graph:
    def __init__(self):
        return
    
    def __addPlot(self, fig, pos, xs, ys, ys2, lblName1, lblName2,
                  sColor='blue', xLbl ='x', yLbl='y'):
        ax = fig.add_subplot(pos[0], pos[1], pos[2])
        ax.plot(xs, ys, color=sColor, label=lblName1)
        ax.plot(xs, ys2, color='red', label=lblName2)
        ax.set_xlabel(xLbl)
        ax.set_ylabel(yLbl)
        ax.legend(loc = 'upper right') 
        return ax
    
    def jointShow(self, data : LogData):
        t = data.timeBuf.getList()
        qs = cm.transpose(data.qBuf.getList())
        qdots = cm.transpose(data.qdBuf.getList())
        qcmds = cm.transpose(data.qcmdBuf.getList())
        qdcmds = cm.transpose(data.qdcmdBuf.getList())
        
        # Figure construction        
        fig = plt.figure(figsize = (22,5), facecolor='lightblue')
        row = 2
        col = max(len(qs), len(qdots))
        plotCount = 0
        # plot: q
        axs_q = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qs[i], qcmds[i], 'J'+str(i+1)+'act pos', 'J'+str(i+1)+'cmd pos',
                                sColor='blue', xLbl ='[cyc]', yLbl='[deg]') for i in range(len(qs))]
        plotCount = plotCount + len(qs)
        # plot: qdot
        axs_qdots = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qdots[i], qdcmds[i], 'J'+str(i+1)+'act vel', 'J'+str(i+1)+'cmd vel',
                                    sColor='green', xLbl ='[cyc]', yLbl='[deg/s]') for i in range(len(qdots))]
        plotCount = plotCount + len(qdots)

        fig.tight_layout()
#        plt.show()

    def tcpShow(self, data : LogData):
        t = data.timeBuf.getList()
        ps = cm.transpose(data.pBuf.getList())
        pdots = cm.transpose(data.pdBuf.getList())
        pcmds = cm.transpose(data.pcmdBuf.getList())
        pdcmds = cm.transpose(data.pdcmdBuf.getList())
        
        # Figure construction        
        fig = plt.figure(figsize = (22,5), facecolor='lightblue')
        row = 2
        col = max(len(ps), len(pdots))
        plotCount = 0
        # plot: p
        cartesian_str = ['x','y','z','ro','pi','yo']
        axs_q = [self.__addPlot(fig, [row,col, plotCount+i+1], t, ps[i], pcmds[i], c_str+' act pos', c_str+' cmd pos',
                                sColor='blue', xLbl ='[cyc]', yLbl='[deg]') for i,c_str in enumerate(cartesian_str)]
        plotCount = plotCount + len(ps)
        # plot: pdot
        axs_qdots = [self.__addPlot(fig, [row,col, plotCount+i+1], t, pdots[i], pdcmds[i], c_str+'act vel', c_str+'cmd vel',
                                    sColor='green', xLbl ='[cyc]', yLbl='[deg/s]') for i,c_str in enumerate(cartesian_str)]
        plotCount = plotCount + len(pdots)

        fig.tight_layout()
#        plt.show()


    
    def quickShow(xSeries,ySeries):
        plt.plot(xSeries, ySeries)
        plt.show()        

## DataLogger
class DataLogger:
    def __init__(self):
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
        self.__size = size
        self.__enabled = True
        # configuration space
        self.__data.qBuf = cm.RingBuffer(size)
        self.__data.qdBuf = cm.RingBuffer(size)
        self.__data.qcmdBuf = cm.RingBuffer(size)
        self.__data.qdcmdBuf = cm.RingBuffer(size)
        # task space
        self.__data.pBuf = cm.RingBuffer(size)
        self.__data.pdBuf = cm.RingBuffer(size)
        self.__data.pcmdBuf = cm.RingBuffer(size)
        self.__data.pdcmdBuf = cm.RingBuffer(size)

    def log(self, time,
            qs, qdots, qcmds, qdcmds,
            ps, pdots, pcmds, pdcmds):
        if self.__enabled:
            self.__data.timeBuf.add(time)    
            # configuration space
            self.__data.qBuf.add(qs)    
            self.__data.qdBuf.add(qdots)    
            self.__data.qcmdBuf.add(qcmds)    
            self.__data.qdcmdBuf.add(qdcmds)    
            # task space
            self.__data.pBuf.add(ps)    
            self.__data.pdBuf.add(pdots)    
            self.__data.pcmdBuf.add(pcmds)    
            self.__data.pdcmdBuf.add(pdcmds)    

    def endLog(self):
        self.__enabled = False

#    def getLog_q(self):
#        return self.__data.qBuf.getList()

#    def getLog_qdot(self):
#        return self.__data.qdotBuf.getList()

    def showLog(self):
        g = Graph()
        g.jointShow(self.__data)
        
        #debug
#        g1 = Graph()
#        g1.tcpShow(self.__data)

        plt.show()


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
