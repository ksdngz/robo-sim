import queue
import numpy as np
import tkinter as tk
from common import common as com
from common import common_constants as const
import simState as ss
from task import taskRequest as tr
from task import taskManagerService as tms
from common.pose6d import Pose6d

import copy

def updateEntryValueFloat(entry : tk.Entry,
                     val : float) -> None:
    orgState = entry.cget("state")
    entry.configure(state='normal')
    entry.delete(0, tk.END)
    entry.insert(tk.END, "{:.2f}".format(val))
    entry.configure(state=orgState)

def updateEntryValueString(entry : tk.Entry,
                     val : str) -> None:
    orgState = entry.cget("state")
    entry.configure(state='normal')
    entry.delete(0, tk.END)
    entry.insert(tk.END, val)
    entry.configure(state=orgState)

def getEntryValue(entry : tk.Entry) -> float:
    s = entry.get()
    if not com.isNum(s):
        return 0. # todo to returns error in case of not number
    return float(s)

class DataLogView:
    def __init__(self, window: tk.Tk, state: ss.SimState):
        self.__frame = tk.Frame(window, relief=tk.GROOVE, bd=2)
        self.__state = state
        self.btn_startDataLog = tk.Button(self.__frame, text="start", command=self.__onbtn_startDataLog)
        self.btn_endDataLog = tk.Button(self.__frame, text="end", command=self.__onbtn_endDataLog)
        self.btn_showDataLog = tk.Button(self.__frame, text="show", command=self.__onbtn_showDataLog)
        self.__enableJointLog = tk.BooleanVar(value = True) # set True as initial value
        self.check_jointDataLog = tk.Checkbutton(self.__frame, 
                           text = "joint",
                           command = self.__checkJointLog,
                           variable = self.__enableJointLog)

    def grid(self, col: int , row: int) -> None:
        self.__frame.grid(column=col, row=row)
        rowNum = 0
#        self.btn_startDataLog.grid(column=1, row=rowNum)
#        self.btn_endDataLog.grid(column=2, row=rowNum)
#        self.btn_showDataLog.grid(column=3, row=rowNum)
        rowNum = 1
#        self.btn_startDataLog.grid(column=1, row=2)
        self.check_jointDataLog.pack()
        self.btn_startDataLog.pack()
        self.btn_endDataLog.pack()
        self.btn_showDataLog.pack()

    def __onbtn_startDataLog(self):
        isEnableJointLog: bool = self.__enableJointLog.get()
        print('joint log enable:', isEnableJointLog)
        self.__state.startDataLog()
        
    def __onbtn_endDataLog(self):
        self.__state.endDataLog()        

    def __onbtn_showDataLog(self):
        self.__state.showDataLog()        
    
    def __checkJointLog(self) -> None:
        return
        



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
                 optState : str ='normal'): # 'normal', 'readonly'
        ENTRY_WIDTH = 7
        self.label          = tk.Label(frame, text=name)
        self.entries        = [tk.Entry(frame, state=optState, width=ENTRY_WIDTH) for i in range(const.CARTESIAN_POSE_SIZE) ]
        self.label.grid(column= 0, row=rowNum)
        for i, entry in enumerate(self.entries):
            entry.grid(column=i+1, row=rowNum)

    def updateValues(self, 
                     pose: Pose6d) -> None:
        p : np.ndarray = pose.eul()
        for i,entry in enumerate(self.entries):
            updateEntryValueFloat(entry, p[i])

    def getPose(self) -> Pose6d:
        ps : list[float] = [getEntryValue(entry) for entry in self.entries]
        return Pose6d(ps)

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

        # status view
        updateEntryValueString(self.entry_movingState, self.state_.controllerState.motionState.name)
        
        # continuous gui update 
        self.window.after(1000, self.update)
        
        
    def __onbtn_copyJnt(self):
        for jntView in self.jntViews:
            jntView.copyq2cq()

    def __onbtn_moveJntAll(self):
        targets : list[tuple[int, float]] = [(jnt.jno, jnt.cq) for jnt in self.jntViews]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_moveZero(self):
        targets : list[tuple[int, float]] = [(i+1, np.rad2deg(j)) for i,j in enumerate(const.ZERO_JOINTS)]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_moveHome(self):
        hjoint = copy.copy(const.HOME_JOINTS)  
        #hjoint = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]  
        targets : list[tuple[int, float]] = [(i+1, np.rad2deg(j)) for i,j in enumerate(hjoint)]
        self.__allJointsRequest.put(tr.MultiJointMoveRequest(targets))

    def __onbtn_copyTcp(self):
        tcp : Pose6d = self.tcpView.getPose()
        self.tcpCmdView.updateValues(tcp)

    def __onbtn_moveJointAllTcpBase(self):
        tcp = self.tcpCmdView.getPose()
        self.__allJointsRequest.put(tr.MultiJointMoveTcpRequest(tcp))

    def __onbtn_moveTcpStraight(self):
        tcp = self.tcpCmdView.getPose()
        self.__allJointsRequest.put(tr.TCPMoveStraightRequest(tcp))


    def start(self):
        self.running = True
        self.window = tk.Tk()
        self.window.title("Debugger")
        self.window.geometry('740x340')

        # frame
        self.jntFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.jntFrame.propagate(False)
        self.tcpFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.tcpFrame.propagate(False)
        self.datalogFrame = DataLogView(self.window, self.state_)
#        self.datalogFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
#        self.datalogFrame.propagate(False)
        self.statusFrame = tk.Frame(self.window, relief=tk.GROOVE, bd=2)
        self.statusFrame.propagate(False)

        # frame layout
        self.jntFrame.grid(column=1, row=1)
        self.tcpFrame.grid(column=2, row=1)
        self.statusFrame.grid(column=1, row=2)
#        self.datalogFrame.grid(column=2, row=2)
        self.datalogFrame.grid(2,2)
        
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
#        self.btn_startDataLog = tk.Button(self.datalogFrame, text="start", command=self.__onbtn_startDataLog)
#        self.btn_endDataLog = tk.Button(self.datalogFrame, text="end", command=self.__onbtn_endDataLog)
#        self.btn_showDataLog = tk.Button(self.datalogFrame, text="show", command=self.__onbtn_showDataLog)
#        rowNum = 0
#        self.btn_startDataLog.grid(column=1, row=rowNum)
#        self.btn_endDataLog.grid(column=2, row=rowNum)
#        self.btn_showDataLog.grid(column=3, row=rowNum)
#        init_check_value = tk.BooleanVar(value = True) # set True as initial value
#        rowNum = 1
#        self.check_jointDataLog = tk.Checkbutton(self.datalogFrame, 
#                           text = "joint",
#                           command = self.__check_jointDataLog,
#                           variable = init_check_value)
#        self.btn_startDataLog.grid(column=1, row=rowNum)
        
        # PoseView
        rowNum : int = 1
        self.tcpView = PoseView(self.tcpFrame, 'tcp', rowNum, 'readonly')
        pose = Pose6d()
        self.tcpView.updateValues(pose)

        rowNum = rowNum + 1
        self.tcpCmdView = PoseView(self.tcpFrame, 'tcpcmd', rowNum)
        
        rowNum = rowNum + 1
        self.btn_tcpcpy      = tk.Button(self.tcpFrame, text="copy", command=self.__onbtn_copyTcp)
        self.btn_tcpcpy.grid(column=1, row=rowNum)
        self.btn_movetcp      = tk.Button(self.tcpFrame, text="moveJntAll", command=self.__onbtn_moveJointAllTcpBase)
        self.btn_movetcp.grid(column=2, row=rowNum)
        self.btn_movetcpStraight      = tk.Button(self.tcpFrame, text="moveStraight", command=self.__onbtn_moveTcpStraight)
        self.btn_movetcpStraight.grid(column=3, row=rowNum)

        # Status Widgets
        STATE_ENTRY_WIDTH = 12
        self.label_movingState = tk.Label(self.statusFrame, text="movingState", width=STATE_ENTRY_WIDTH)
        self.entry_movingState = tk.Entry(self.statusFrame, state="readonly", width=STATE_ENTRY_WIDTH)
        self.label_movingState.grid(column=1, row=1)
        self.entry_movingState.grid(column=2, row=1)
        
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