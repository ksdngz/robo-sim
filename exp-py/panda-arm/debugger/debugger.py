import queue
import numpy as np
import tkinter as tk
from common import common as com
from common import common_constants as const
import simState as ss
from task import taskRequest as tr
from task import taskManagerService as tms

import copy

def updateEntryValueFloat(entry : tk.Entry,
                     val : float) -> None:
    orgState = entry.cget("state")
    entry.configure(state='normal')
    entry.delete(0, tk.END)
    entry.insert(tk.END, "{:.2f}".format(val))
    entry.configure(state=orgState)

def getEntryValue(entry : tk.Entry) -> float:
    s = entry.get()
    if not com.isNum(s):
        return 0. # todo to returns error in case of not number
    return float(s)

class JointView:
    def __init__(self, frame, name, rowNum, state, index):
        ENTRY_WIDTH = 8
        self.label          = tk.Label(frame, text=name)
        self.entry_q        = tk.Entry(frame, state='readonly', width=ENTRY_WIDTH)
        self.entry_dq       = tk.Entry(frame, state='readonly', width=ENTRY_WIDTH)
        self.entry_cq       = tk.Entry(frame, width=ENTRY_WIDTH)
        self.btn_apply      = tk.Button(frame, text="apply", command=self.__onbtn_apply)
        self.__state        = state
        self.__index        = index
        self.__jno          = index + 1 # to be refactored 
        self.__requests     = queue.Queue()
        # placement
        self.label.grid(column=0, row=rowNum)
        self.entry_q.grid(column=1, row=rowNum)
        self.entry_dq.grid(column=2, row=rowNum)
        self.entry_cq.grid(column=3, row=rowNum)
        self.btn_apply.grid(column=4, row=rowNum)

    @property
    def cq(self) -> float:
        return getEntryValue(self.entry_cq)

    @property
    def jno(self) -> int:
        return self.__jno

    def __onbtn_apply(self):
        cq = getEntryValue(self.entry_cq) # [deg]
        self.__state.joints_[self.__index].qtarget_ = cq # todo to update index of qtarget_
        targets : list[tuple[int, float]] = []
        targets.append((self.__jno, cq))
        self.__requests.put(tr.MultiJointMoveRequest(targets))
    
    def updateActValues(self, 
                        q : float,
                        dq : float) -> None:
        updateEntryValueFloat(self.entry_q, q)
        updateEntryValueFloat(self.entry_dq, dq)
    
    def copyq2cq(self):
        q = getEntryValue(self.entry_q)
        updateEntryValueFloat(self.entry_cq, q)
    
    def getRequests(self):
        requests = []
        while self.__requests.not_empty():
            requests.append(self.__requests.get())
        return requests
        
    def getRequest(self):
        if not self.__requests.empty():
            return self.__requests.get()
        return None

class PoseView:
    def __init__(self, 
                 frame, 
                 name : str,
                 rowNum : int,
                 optState : str ='normal'): # 'normal' 'readonly'
        ENTRY_WIDTH = 7
        CARTESIAN_POSE_SIZE = 6
        self.label          = tk.Label(frame, text=name)
        self.entries        = [tk.Entry(frame, state=optState, width=ENTRY_WIDTH) for i in range(CARTESIAN_POSE_SIZE) ]
        self.label.grid(column= 0, row=rowNum)
        for i, entry in enumerate(self.entries):
            entry.grid(column=i+1, row=rowNum)

    def updateValues(self, 
                     pose: list[float]) -> None:
        for i,entry in enumerate(self.entries):
            updateEntryValueFloat(entry, pose[i])

    def getPose(self) -> list[float]:
        return [getEntryValue(entry) for entry in self.entries]

class Debugger:
    def __init__(self, 
                 state: ss.SimState, 
                 taskManagerService : tms.TaskManagerService):
        self.state_ = state
        self.__taskManagerService = taskManagerService
        self.__allJointsRequest = queue.Queue()

    def update(self):
        # joint
        for i, jntView in enumerate(self.jntViews):
            jntView.updateActValues(self.state_.joints_[i].q_, self.state_.joints_[i].dq_)
            request = jntView.getRequest()
            if request is not None:
                self.__taskManagerService.pushRequest(request)

        # all joints
        if not self.__allJointsRequest.empty():
            req = self.__allJointsRequest.get()
            self.__taskManagerService.pushRequest(req)

        # tcp view
        self.tcpView.updateValues(self.state_.tcpPose())

        self.window.after(1000, self.update)

    def __onbtn_startDataLog(self):
        self.state_.startDataLog()
        
    def __onbtn_endDataLog(self):
        self.state_.endDataLog()        

    def __onbtn_showDataLog(self):
        self.state_.showDataLog()        
    
    def __onbtn_copyJnt(self):
        for jntView in self.jntViews:
            jntView.copyq2cq()

    def __onbtn_moveJntAll(self):
        targets : list[tuple[int, float]] = [(jnt.jno, jnt.cq) for jnt in self.jntViews]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_moveZero(self):
        targets : list[tuple[int, float]] = [(i+1, np.rad2deg(j)) for i,j in enumerate(const.Constants.ZERO_JOINTS)]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_moveHome(self):
        hjoint = copy.copy(const.Constants.HOME_JOINTS)  
        #hjoint = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]  
        targets : list[tuple[int, float]] = [(i+1, np.rad2deg(j)) for i,j in enumerate(hjoint)]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_copyTcp(self):
        tcp = self.tcpView.getPose()
        self.tcpCmdView.updateValues(tcp)

    def __onbtn_moveTcp(self):
        tcp = self.tcpCmdView.getPose()
        self.__allJointsRequest.put(tr.MultiJointMoveTcpRequest(tcp))

    def start(self):
        self.running = True
        self.window = tk.Tk()
        self.window.title("Debugger")
        self.window.geometry('540x340')

        # frame
        self.jntFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.jntFrame.propagate(False)
        self.tcpFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.tcpFrame.propagate(False)
        self.datalogFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.datalogFrame.propagate(False)

        # frame layout
        self.jntFrame.grid(column=1, row=1)
        self.tcpFrame.grid(column=1, row=2)
        self.datalogFrame.grid(column=2, row=1)

        # Joint Widgets
        #JOINT_NUM = 7
        LABEL_WIDTH = 7
        jntValueLabels = [tk.Label(self.jntFrame, text=name, width=LABEL_WIDTH) for name in ['q[deg]','qd[deg]','cq[deg]']]
        TITLE_ROW = 0
        for i,label in enumerate(jntValueLabels):
            label.grid(column=i+1, row=TITLE_ROW)

        self.jntViews = [JointView(self.jntFrame, 'J'+str(i+1), i+1, self.state_, i) for i in range(self.state_.qsize_)]
        for jntView in self.jntViews:
            jntView.updateActValues(0, 0)
        
        self.btn_jntcpy      = tk.Button(self.jntFrame, text="copy", command=self.__onbtn_copyJnt)
        self.btn_jntcpy.grid(column=1, row=8)
        self.btn_moveJntAll      = tk.Button(self.jntFrame, text="movJAll", command=self.__onbtn_moveJntAll)
        self.btn_moveJntAll.grid(column=2, row=8)
        self.btn_moveZero      = tk.Button(self.jntFrame, text="zero", command=self.__onbtn_moveZero)
        self.btn_moveZero.grid(column=3, row=8)
        self.btn_moveHome      = tk.Button(self.jntFrame, text="home", command=self.__onbtn_moveHome)
        self.btn_moveHome.grid(column=4, row=8)

        # tcp Widgets
        tcpValueLabel = [tk.Label(self.tcpFrame, text=name) for name in ['x[m]', 'y[m]', 'z[m]', 'r[deg]', 'p[deg]', 'y[deg]']]
        titleRow = 0
        for i,label in enumerate(tcpValueLabel):
            label.grid(column=i+1, row=titleRow)

        # Datalog Widgets
        self.btn_startDataLog = tk.Button(self.datalogFrame, text="start", command=self.__onbtn_startDataLog)
        self.btn_endDataLog = tk.Button(self.datalogFrame, text="end", command=self.__onbtn_endDataLog)
        self.btn_showDataLog = tk.Button(self.datalogFrame, text="show", command=self.__onbtn_showDataLog)
        rowNum = 0
        self.btn_startDataLog.grid(column=1, row=rowNum)
        self.btn_endDataLog.grid(column=2, row=rowNum)
        self.btn_showDataLog.grid(column=3, row=rowNum)

        rowNum : int = 1
        self.tcpView = PoseView(self.tcpFrame, 'tcp', rowNum, 'readonly')
        pose = [0]*6
        self.tcpView.updateValues(pose)

        rowNum = rowNum + 1
        self.tcpCmdView = PoseView(self.tcpFrame, 'tcpcmd', rowNum)
        
        rowNum = rowNum + 1
        self.btn_tcpcpy      = tk.Button(self.tcpFrame, text="copy", command=self.__onbtn_copyTcp)
        self.btn_tcpcpy.grid(column=1, row=rowNum)
        self.btn_movetcp      = tk.Button(self.tcpFrame, text="moveJntAll", command=self.__onbtn_moveTcp)
        self.btn_movetcp.grid(column=2, row=rowNum)

        # Widget
        self.update()
        self.window.mainloop()
        # need to delete variables that reference tkinter objects in the thread
        del self.window

    def _check_to_quit(self):
        if self.running:
            self.window.after(1000, self._check_to_quit)
        else:
            self.window.destroy()

    def quit(self):
        self.running = False