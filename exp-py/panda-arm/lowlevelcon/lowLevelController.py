import rtbWrapper as rtb

class LowLevelController:
    def tick(self, model, data):
        print('Error: superClass method is called.')
        assert()

    def update(self, targetPos): # todo
        print('Error: superClass method is called.')
        assert()
    
    def getCmdPos(self):
        print('Error: superClass method is called.')
        assert()

class PIDController(LowLevelController):
    def __init__(self, model, data, kp, kd, ki):
        self.model = model
        self.data = data
        self.kp_ = kp
        self.kd_ = kd
        self.ki_ = ki
        self.epre_ = [0]*self.model.nu
        self.ie_ = [0]*self.model.nu
        self.qcmd_ = [0]*self.model.nu #[rad]
        self.qdcmd_ = [0]*self.model.nu #[rad/s]
        self.T = 1

    def tick(self, model, data):
        gc = rtb.calcGravComp(data.qpos)
        e = self.qcmd_ - data.qpos
        de = (e - self.epre_)/self.T
        self.ie_ = self.ie_ + (e+de)*self.T/2
        u = self.kp_*e + self.kd_*de + self.ki_*self.ie_ + gc
        ## set ctrl in mujoco
        data.ctrl = u

    def update(self, qcmd):
        for i in range(len(self.qcmd_)):
            self.qdcmd_[i] = (qcmd[i] - self.qcmd_[i])/self.T 
        self.qcmd_ = qcmd

    def getCmdPos(self):
        return self.qcmd_

    def getCmdVel(self):
        return self.qdcmd_

    
#def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    # pass
    # global K
#    K =10000
#    body_id = model.jnt('panda_joint1').id    

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

#    for i in range(model.nu):
#        data.ctrl[i] = data.qfrc_bias[i] *1

    #2 apply disturbance torque
#   tau_disturb_mean = 0
#   tau_disturb_dev = 20
#   tau_d0 = np.random.normal(tau_disturb_mean,tau_disturb_dev)
#   tau_d1 = np.random.normal(tau_disturb_mean,0.25*tau_disturb_dev)
#   data.qfrc_applied[0] = tau_d0
#   data.qfrc_applied[1] = tau_d1