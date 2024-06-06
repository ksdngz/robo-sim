import queue
import tkinter as tk
import tkinter.ttk as ttk
import common as com
import simState as ss
from task import taskRequest as tr
from task import taskManagerService as tms

class JointView:
    def __init__(self, frame, name, rowNum, state, index):
        ENTRY_WIDTH = 7
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
        self.label.grid(column= 0, row=rowNum)
        self.entry_q.grid(column=1, row=rowNum)
        self.entry_dq.grid(column=2, row=rowNum)
        self.entry_cq.grid(column=3, row=rowNum)
        self.btn_apply.grid(column=4, row=rowNum)

    def __getEntryValue(self, entry) -> float:
        s = entry.get()
        if not com.isNum(s):
            return 0. # todo to returns error in case of not number
        return float(s)

    def __onbtn_apply(self):
        cq = self.__getEntryValue(self.entry_cq) # [deg]
        self.__state.joints_[self.__index].qtarget_ = cq # todo to update index of qtarget_
        self.__requests.put(tr.SingleJointMoveRequest(self.__jno, cq))
    
    def updateActValues(self, q, dq):
        self.entry_q.configure(state='normal')
        self.entry_q.delete(0, tk.END)
        self.entry_q.insert(tk.END, "{:.2f}".format(q))
        self.entry_q.configure(state='readonly')

        self.entry_dq.configure(state='normal')
        self.entry_dq.delete(0, tk.END)
        self.entry_dq.insert(tk.END, "{:.2f}".format(dq))
        self.entry_dq.configure(state='readonly')
    
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
    def __init__(self, frame, name):
        ENTRY_WIDTH = 7
        CARTESIAN_POSE_SIZE = 6
        ROW_NUM = 1
        self.label          = tk.Label(frame, text=name)
        self.entries        = [tk.Entry(frame, state='readonly', width=ENTRY_WIDTH) for i in range(CARTESIAN_POSE_SIZE) ]
        # self.btn_apply      = tk.Button(frame, text="apply", command=self.__onbtn_apply)
        #self.__state        = state
        #self.__index        = index
        #self.__jno          = index + 1 # to be refactored 
        #self.__requests     = queue.Queue()
        # placement
        self.label.grid(column= 0, row=ROW_NUM)
        for i, entry in enumerate(self.entries):
            entry.grid(column=i+1, row=ROW_NUM)

    def updateActValues(self, 
                        pose: list[float]) -> None:
        for i,entry in enumerate(self.entries):
            entry.configure(state='normal')
            entry.delete(0, tk.END)
            entry.insert(tk.END, "{:.2f}".format(pose[i]))
            entry.configure(state='readonly')

class Debugger:
    def __init__(self, state: ss.SimState, taskManagerService : tms.TaskManagerService):
        self.state_ = state
        self.__taskManagerService = taskManagerService

    def update(self):
        for i, jntView in enumerate(self.jntViews):
            jntView.updateActValues(self.state_.joints_[i].q_, self.state_.joints_[i].dq_)
            request = jntView.getRequest()
            if request is not None:
                self.__taskManagerService.pushRequest(request)

        self.tcpViews.updateActValues(self.state_.tcpPose())

        self.window.after(1000, self.update)

    def __onbtn_startDataLog(self):
        self.state_.startDataLog()
        
    def __onbtn_endDataLog(self):
        self.state_.endDataLog()        

    def __onbtn_showDataLog(self):
        self.state_.showDataLog()        
    
    def start(self):
        self.running = True
        self.window = tk.Tk()
        self.window.title("Debugger")
        self.window.geometry('540x240')

        # frame
        self.jntFrame = tk.Frame(self.window)
        self.tcpFrame = tk.Frame(self.window)
        self.datalogFrame = tk.Frame(self.window)
        # frame layout
        self.jntFrame.grid(column=1, row=0)
        self.tcpFrame.grid(column=1, row=2)
        self.datalogFrame.grid(column=1, row=1)

        # Joint Widgets
        titleRow = 0
        label_q = tk.Label(self.jntFrame, text='q[deg]')
        label_dq = tk.Label(self.jntFrame, text='dq[deg/s]')
        label_cq = tk.Label(self.jntFrame, text='cq[deg]')
        label_q.grid(column=1, row=titleRow)
        label_dq.grid(column=2, row=titleRow)
        label_cq.grid(column=3, row=titleRow)

        # tcp Widgets
        titleRow = 0
        label_x = tk.Label(self.tcpFrame, text='x[m]')
        label_y = tk.Label(self.tcpFrame, text='y[m]')
        label_z = tk.Label(self.tcpFrame, text='z[m]')
        label_r = tk.Label(self.tcpFrame, text='r[deg]')
        label_p = tk.Label(self.tcpFrame, text='p[deg]')
        label_y = tk.Label(self.tcpFrame, text='y[deg]')
        label_x.grid(column=1, row=titleRow)
        label_y.grid(column=2, row=titleRow)
        label_z.grid(column=3, row=titleRow)
        label_r.grid(column=4, row=titleRow)
        label_p.grid(column=5, row=titleRow)
        label_y.grid(column=6, row=titleRow)

        # Datalog Widgets
        self.btn_startDataLog = tk.Button(self.datalogFrame, text="start", command=self.__onbtn_startDataLog)
        self.btn_endDataLog = tk.Button(self.datalogFrame, text="end", command=self.__onbtn_endDataLog)
        self.btn_showDataLog = tk.Button(self.datalogFrame, text="show", command=self.__onbtn_showDataLog)
        rowNum = 0
        #self.label.grid(column= 0, row=rowNum)
        self.btn_startDataLog.grid(column=1, row=rowNum)
        self.btn_endDataLog.grid(column=2, row=rowNum)
        self.btn_showDataLog.grid(column=3, row=rowNum)

        self.jntViews = [JointView(self.jntFrame, 'J'+str(i+1), i+1, self.state_, i) for i in range(self.state_.qsize_)]
        for jntView in self.jntViews:
            jntView.updateActValues(0, 0)
        
        self.tcpViews = PoseView(self.tcpFrame, 'tcp')
        pose = [0]*6
        self.tcpViews.updateActValues(pose)
        
        
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