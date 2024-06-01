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
import taskRequest as tr

import lowLevelController as llc

import motionController as mc

import taskManagerService as tms
import taskManager as tm



import common as com
import simState as ss

# global settings
np.set_printoptions(precision=2)

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
        cq = np.deg2rad(self.__getEntryValue(self.entry_cq))
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
        return self.__requests.get()


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

        self.jntViews = [JointView(self.jntFrame, 'J'+str(i+1), i+1, self.state_, i) for i in range(self.state_.qsize_)]
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
initq = [0., 1.3, 0., 0., 0., 0., 0.]

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
simState = ss.SimState(model.nu)

# Init lowLevelcontroller
kp = 20
kd = 0.5
ki = 0.1
lowLevelCon = llc.PIDController(model, data, kp, kd, ki)
lowLevelCon.update(initq)
mj.set_mjcb_control(lowLevelCon.controller)

# Init MotionController
motionCon = mc.MotionController()

# Init TaskManager
taskMgr = tm.TaskManager(simState, motionCon.getService())

# Init Debugger
debugger = Debugger(simState, taskMgr.getService())
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

# state.qtarget_ = initq
data.qpos = initq

for i, jnt in enumerate(simState.joints_):
    jnt.qtarget_ = initq[i]

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)
        simState.updateBySimulation(data)
        
        taskMgr.tick()
        motionCon.tick()
        
        # todo to change below
        #qtargets = [simState.joints_[i].qtarget_ for i in range(model.nu)]
        #lowLevelCon.update(qtargets)

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
