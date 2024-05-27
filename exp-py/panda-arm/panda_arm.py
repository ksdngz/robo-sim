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

# global settings
np.set_printoptions(precision=2)

class InState:
    def __init__(self, qSize):
        self.q_ = [0]*qSize
        self.dq_ = [0]*qSize
        self.qtarget_ = [0]*qSize
        self.qsize_ = qSize

    def updateBySimulation(self, data):
        self.q_ = ["{:.2f}".format(q) for q in np.rad2deg(data.qpos)]
        self.dq_ = ["{:.2f}".format(dq) for dq in np.rad2deg(data.qvel)]

class JointView:
    def __init__(self, frame, name, rowNum, state):
        ENTRY_WIDTH = 7
        self.label          = tk.Label(frame, text=name)
        self.entry_q        = tk.Entry(frame, state='readonly', width=ENTRY_WIDTH)
        self.entry_dq       = tk.Entry(frame, state='readonly', width=ENTRY_WIDTH)
        self.entry_cq       = tk.Entry(frame, width=ENTRY_WIDTH)
        self.btn_apply      = tk.Button(frame, text="apply", command=self.__onbtn_apply)
        self.__state        = state
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
        self.__state.qtarget_[0] = np.deg2rad(self.__getEntryValue(self.entry_cq)) # todo to update index of qtarget_
    
    def updateActValues(self, q, dq):
        self.entry_q.configure(state='normal')
        self.entry_q.delete(0, tk.END)
        self.entry_q.insert(tk.END, str(q))
        self.entry_q.configure(state='readonly')

        self.entry_dq.configure(state='normal')
        self.entry_dq.delete(0, tk.END)
        self.entry_dq.insert(tk.END, str(dq))
        self.entry_dq.configure(state='readonly')

class Debugger:
    def __init__(self, state: InState):
        self.state_ = state

    def update(self):
        for i, jntView in enumerate(self.jntViews):
            jntView.updateActValues(self.state_.q_[i], self.state_.dq_[i])

        self.window.after(1000, self.update)

    def start(self):
        self.running = True
        self.window = tk.Tk()
        self.window.title("Debugger")
        self.window.geometry('540x240')

        # Frame
        self.jntFrame = tk.Frame(self.window)
        self.jntFrame.pack(fill = tk.BOTH, pady=10)

        # JointView
        titleRow = 0
        label_q = tk.Label(self.jntFrame, text='q[deg]')
        label_dq = tk.Label(self.jntFrame, text='dq[deg/s]')
        label_cq = tk.Label(self.jntFrame, text='cq[deg]')
        label_q.grid(column=1, row=titleRow)
        label_dq.grid(column=2, row=titleRow)
        label_cq.grid(column=3, row=titleRow)

        self.jntViews = [JointView(self.jntFrame, 'J'+str(i), i, state) for i in range(1, state.qsize_+1)]
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
state.qtarget_ = initq

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
        controller.update(state.qtarget_)

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
