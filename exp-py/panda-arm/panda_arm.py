import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import control
import threading
import tkinter as tk
import tkinter.ttk as ttk
import rtbWrapper as rtb



# commmon method
# 
# check if the string s is a number or not.
# Note that the string including decimals returns true. 
# 
def isNum(s): 
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

def transpose(list_org): # doubly list e.g. [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    list_transposed = [list(x) for x in zip(*list_org)]
    return list_transposed

# global settings
np.set_printoptions(precision=2)

class JointState:
    def __init__(self):
        self.q_ = 0 # [deg]
        self.dq_ = 0 # [deg/s]
        self.qtarget_ = 0 # [deg]

    def updateBySimulation(self, q, dq):
        self.q_ = np.rad2deg(q)
        self.dq_ = np.rad2deg(dq)
#        self.q_ = "{:.2f}".format(np.rad2deg(q))
#        self.dq_ = "{:.2f}".format(np.rad2deg(dq))
            
class InState:
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

class RingBuffer:
    def __init__(self, size):
        self.buffer = [None for i in range(0, size)]
        self.top = 0
        self.bottom = 0
        self.size = size
        self.isFull = False

    def __len__(self):
        return self.bottom - self.top

    def add(self, value):
        self.buffer[self.bottom] = value
        self.bottom = (self.bottom + 1) % len(self.buffer)
        if(self.top == self.bottom):
            self.isFull = True
            


    def getVal(self, index=None):
        if index is not None:
            return self.buffer[index]

        value = self.buffer[self.top]
        self.top =(self.top + 1) % len(self.buffer)
        return value
    
    def getList(self):
        l = []
        if(self.isFull): # todo test
            l = self.buffer[self.bottom:]
            l.extend(self.buffer[:self.bottom])
        else:
            l = self.buffer[:self.bottom]
        return l

# rBuf = RingBuffer(10)
# rBuf.add(1)
# rBuf.add(2)
# rBuf.add(3)
# rBuf.add(4)
# rBuf.add(5)
# print(rBuf.getList())

class LogData:
    def __init__(self):
        MAX_LOGGING_SIZE = 100000
        self.timeBuf = RingBuffer(MAX_LOGGING_SIZE)
        self.qBuf = RingBuffer(MAX_LOGGING_SIZE)
        self.qdotBuf = RingBuffer(MAX_LOGGING_SIZE)

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
        qs = transpose(data.qBuf.getList())
        qdots = transpose(data.qdotBuf.getList())
        
#        x = np.linspace(0, 10, 1000)
#        y = np.sin(x)
#        c1, c2 = 'blue', 'green'
#        l1, l2 = 'sin', 'cos'
#        xl1, xl2 = 'x', 'x'
#        yl1, yl2 = 'sin', 'cos'
        #グラフを表示する領域を，figオブジェクトとして作成。
        fig = plt.figure(figsize = (22,5), facecolor='lightblue')
        row = 2
        col = max(len(qs), len(qdots))
        plotCount = 0
        # q
        axs_q = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qs[i], 'J'+str(i+1)+' pos',
                                sColor='blue', xLbl ='[cyc]', yLbl='[deg]') for i in range(len(qs))]
        plotCount = plotCount + len(qs)
        # qdot
        axs_qdots = [self.__addPlot(fig, [row,col, plotCount+i+1], t, qdots[i], 'J'+str(i+1)+' vel',
                                    sColor='green', xLbl ='[cyc]', yLbl='[deg/s]') for i in range(len(qdots))]
        plotCount = plotCount + len(qdots)

#        ax1 = fig.add_subplot(2, 4, 1)
#        ax2 = fig.add_subplot(2, 4, 2)
#        ax3 = fig.add_subplot(2, 4, 3)
#        ax4 = fig.add_subplot(2, 4, 4)
#        ax5 = fig.add_subplot(2, 4, 5)
#        ax6 = fig.add_subplot(2, 4, 6)
#        ax7 = fig.add_subplot(2, 4, 7)
#        ax8 = fig.add_subplot(2, 4, 8)
        #各subplot領域にデータを渡す
#        ax1.plot(x, y1, color=c1, label=l1)
#        ax2.plot(x, y2, color=c2, label=l2)
#        #各subplotにxラベルを追加
#        ax1.set_xlabel(xl1)
#        ax2.set_xlabel(xl2)
#        #各subplotにyラベルを追加
#        ax1.set_ylabel(yl1)
#        ax2.set_ylabel(yl2)
#        # 凡例表示
#        ax1.legend(loc = 'upper right') 
#        ax2.legend(loc = 'upper right') 
        fig.tight_layout()
        plt.show()
    


    
## DataLogger
class DataLogger:
    def __init__(self, state : InState):
        self.__state = state
        self.__data = LogData()
#        self.__timeBuf = RingBuffer(MAX_LOGGING_SIZE)
#        self.__qBuf = RingBuffer(MAX_LOGGING_SIZE)
#        self.__qdotBuf = RingBuffer(MAX_LOGGING_SIZE)
        self.__enabled = False
        self.__size = 0

    def startLog(self, size):
        if self.__enabled:
            print('startLog Error: logging is already enabled.')
            return
        # temp
        size = 100000
        self.__data.timeBuf = RingBuffer(size)
        self.__data.qBuf = RingBuffer(size)
        self.__data.qdotBuf = RingBuffer(size)
        self.__size = size
        self.__enabled = True

    def log(self):
        if self.__enabled:
            self.__data.timeBuf.add(state.time())    
            self.__data.qBuf.add(state.qs())    
            self.__data.qdotBuf.add(state.qdots())    

    def endLog(self):
        self.__enabled = False

    def getLog_q(self):
        return self.__data.qBuf.getList()

    def getLog_qdot(self):
        return self.__data.qdotBuf.getList()

    def showLog(self):
        g = Graph()
        g.show(self.__data)

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
        # placement
        self.label.grid(column= 0, row=rowNum)
        self.entry_q.grid(column=1, row=rowNum)
        self.entry_dq.grid(column=2, row=rowNum)
        self.entry_cq.grid(column=3, row=rowNum)
        self.btn_apply.grid(column=4, row=rowNum)

    def __getEntryValue(self, entry) -> float:
        s = entry.get()
        if not isNum(s):
            return 0. # todo to returns error in case of not number
        return float(s)

    def __onbtn_apply(self):
        self.__state.joints_[self.__index].qtarget_ = np.deg2rad(self.__getEntryValue(self.entry_cq)) # todo to update index of qtarget_
    
    def updateActValues(self, q, dq):
        self.entry_q.configure(state='normal')
        self.entry_q.delete(0, tk.END)
        
        #        self.q_ = "{:.2f}".format(np.rad2deg(q))
#        self.dq_ = "{:.2f}".format(np.rad2deg(dq))

        
        self.entry_q.insert(tk.END, "{:.2f}".format(q))
        self.entry_q.configure(state='readonly')

        self.entry_dq.configure(state='normal')
        self.entry_dq.delete(0, tk.END)
        self.entry_dq.insert(tk.END, "{:.2f}".format(dq))
        self.entry_dq.configure(state='readonly')

class Debugger:
    def __init__(self, state: InState):
        self.state_ = state

    def update(self):
        for i, jntView in enumerate(self.jntViews):
            jntView.updateActValues(self.state_.joints_[i].q_, self.state_.joints_[i].dq_)

        self.window.after(1000, self.update)

    def __onbtn_startDataLog(self):
        state.startDataLog()
        
    def __onbtn_endDataLog(self):
        state.endDataLog()        

    def __onbtn_showDataLog(self):
        state.showDataLog()        
    
    def start(self):
        self.running = True
        self.window = tk.Tk()
        self.window.title("Debugger")
        self.window.geometry('540x240')

        # frame
        self.jntFrame = tk.Frame(self.window)
        self.datalogFrame = tk.Frame(self.window)
        # frame layout
        self.jntFrame.grid(column=1, row=0)
        self.datalogFrame.grid(column=2, row=0)

        # Joint Widgets
        titleRow = 0
        label_q = tk.Label(self.jntFrame, text='q[deg]')
        label_dq = tk.Label(self.jntFrame, text='dq[deg/s]')
        label_cq = tk.Label(self.jntFrame, text='cq[deg]')
        label_q.grid(column=1, row=titleRow)
        label_dq.grid(column=2, row=titleRow)
        label_cq.grid(column=3, row=titleRow)

        # Datalog Widgets
        self.btn_startDataLog = tk.Button(self.datalogFrame, text="start", command=self.__onbtn_startDataLog)
        self.btn_endDataLog = tk.Button(self.datalogFrame, text="end", command=self.__onbtn_endDataLog)
        self.btn_showDataLog = tk.Button(self.datalogFrame, text="show", command=self.__onbtn_showDataLog)
        rowNum = 0
        #self.label.grid(column= 0, row=rowNum)
        self.btn_startDataLog.grid(column=1, row=rowNum)
        self.btn_endDataLog.grid(column=2, row=rowNum)
        self.btn_showDataLog.grid(column=3, row=rowNum)

        self.jntViews = [JointView(self.jntFrame, 'J'+str(i+1), i+1, state, i) for i in range(state.qsize_)]
        for jntView in self.jntViews:
            jntView.updateActValues(0, 0)
                
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



#xml_path = '2D_double_pendulum.xml' #xml file (assumes this is in the same folder as this file)
# xml_path = 'urdf/robot/panda_arm.urdf' #xml file (assumes this is in the same folder as this file)
xml_path = 'urdf/robot/panda_arm_mjcf.xml' #xml file (assumes this is in the same folder as this file)

simend = 100 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

#create the function xdot = f(x,u)
def f(x,u):
    #x=q0,q1,qdot0,qdot1
    #u=torque

    data.qpos[0] = x[0]
    data.qpos[1] = x[1]
    data.qvel[0] = x[2]
    data.qvel[1] = x[3]
    data.ctrl[0] = u[0]
    mj.mj_forward(model,data)

    #qddot = inv(M)*(data_ctrl-frc_bias)
    M = np.zeros((2,2))
    mj.mj_fullM(model,M,data.qM)
    invM = np.linalg.inv(M)
    frc_bias = np.array([data.qfrc_bias[0],data.qfrc_bias[1]])
    tau = np.array([u[0],0])
    qddot = np.matmul(invM,np.subtract(tau,frc_bias))

    xdot = np.array([data.qvel[0],data.qvel[1],qddot[0],qddot[1]])
    return xdot


def linearize():

    n = 4
    m = 1
    A = np.zeros((n,n))
    B = np.zeros((n,m))

    x0 = np.array([0,0,0,0])
    u0 = np.array([0])
    xdot0 = f(x0,u0)
    #print(xdot0)

    pert = 1e-2
    #get A matrix
    for i in range(0,n):
        x = [0]*n
        u = u0
        for j in range(0,n):
            x[j] = x0[j]
        x[i] = x[i]+pert
        xdot = f(x,u)
        for k in range(0,n):
            A[k,i] = (xdot[k]-xdot0[k])/pert

    #get B matrix
    for i in range(0,m):
        x = x0
        u = [0]*m
        for j in range(0,m):
            u[j] = u0[j]
        u[i] = u[i]+pert
        xdot = f(x,u)
        for k in range(0,n):
            B[k,i] = (xdot[k]-xdot0[k])/pert

    return A,B


def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    #pass
    global K

    n = 4
    m = 1

    #1. linearization
    A,B = linearize()
    # print(A)
    # print(B)

    #2. linear quadratic regulator
    Q = np.eye((n))
    R = 1e-2*np.eye((m))
    K,S,E = control.lqr(A,B,Q,R)
    # print("K = ",K)


class pidController:
    def __init__(self, model, data, kp, kd, ki):
        # temp
        #kp = 20
        #kd = 2
        #ki = 0.1
        # temp end

        self.model = model
        self.data = data
        self.kp_ = kp
        self.kd_ = kd
        self.ki_ = ki
        self.epre_ = [0]*self.model.nu
        self.ie_ = [0]*self.model.nu
        self.target = [0]*self.model.nu
        self.T = 1

    def controller(self, model, data):
        gc = rtb.calcGravComp(data.qpos)
        e = self.target - data.qpos
        de = (e - self.epre_)/self.T
        self.ie_ = self.ie_ + (e+de)*self.T/2
        u = self.kp_*e + self.kd_*de + self.ki_*self.ie_ + gc
        ## set ctrl in mujoco
        data.ctrl = u

    def update(self, target):
        self.target = target
    
def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    # pass
    # global K
    K =10000
    body_id = model.jnt('panda_joint1').id    

    #1. apply congtrol u = -K*x
#    x = np.array([data.qpos[0],data.qpos[1],data.qvel[0],data.qvel[1]])
#    u = -K.dot(x)
#    u = -K * data.qpos[0]
#    global gCount
#    if gCount % 1000 == 0:
#        q = np.array(data.qpos)
#        ctrl = np.array(data.ctrl)
#        qfrc = np.array(data.qfrc_bias)
#        print("q", q, "ctrl", ctrl, "qfrc", qfrc)
#    gCount = gCount+1     

    for i in range(model.nu):
        data.ctrl[i] = data.qfrc_bias[i] *1

    #2 apply disturbance torque
#   tau_disturb_mean = 0
#   tau_disturb_dev = 20
#   tau_d0 = np.random.normal(tau_disturb_mean,tau_disturb_dev)
#   tau_d1 = np.random.normal(tau_disturb_mean,0.25*tau_disturb_dev)
#   data.qfrc_applied[0] = tau_d0
#   data.qfrc_applied[1] = tau_d1

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

# definition
global gCount
gCount = 0

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
#model = mj.MjModel.from_xml_string(xml_path)
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init Internal State
state = InState(model.nu)

# Init Debugger
debugger = Debugger(state)
debuggerThread = threading.Thread(target=debugger.start)
debuggerThread.start()

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Visualizer", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
#opt.flags[mj.mjtVisFlag.mjVIS_JOINT] = True
#opt.frame = mj.mjtFrame.mjFRAME_GEOM

scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 90 ; cam.elevation = 5 ; cam.distance =  6
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
## temporary disabled
#init_controller(model,data)
initq = [0., 1.3, 0., 0., 0., 0., 0.]
data.qpos = initq
# state.qtarget_ = initq

for i, jnt in enumerate(state.joints_):
    jnt.qtarget_ = initq[i]


#set the controller
kp = 20
kd = 0.5
ki = 0.1
controller = pidController(model, data, kp, kd, ki)
controller.update(initq)
mj.set_mjcb_control(controller.controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)
        state.updateBySimulation(data)
        qtargets = [state.joints_[i].qtarget_ for i in range(model.nu)]
        controller.update(qtargets)

    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
