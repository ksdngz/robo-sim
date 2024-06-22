import copy

import mujoco as mj

from common import common_constants as const 
from lowlevelcon import lowLevelController as llc
from motioncon import motionController as mc
from task import taskManager as tm
import simState as ss

class RobotController:
    def tick(self, model, data):
        print('Error: superClass method is called.')
        assert()

    def load(self, targetPos): # todo
        print('Error: superClass method is called.')
        assert()
    
    def unload(self):
        print('Error: superClass method is called.')
        assert()
    
    #to be refactored
    def getTaskService(self) -> tm.tms.TaskManagerService: 
        print('Error: superClass method is called.')
        assert()

    #to be refactored
    def getCmdPos(self) -> list[float]:
        print('Error: superClass method is called.')
        assert()

    #to be refactored
    def getCmdVel(self) -> list[float]:
        print('Error: superClass method is called.')
        assert()

class simpleRobotController(RobotController):
    def __init__(self, model, data, simState):
        initq = copy.copy(const.HOME_JOINTS)
        kp = 100; kd = 0; ki = 0 # only K_p
        self.__lowLevelCon = llc.PIDController(model, data, kp, kd, ki)
        self.__lowLevelCon.update(initq)
        self.__motionCon = mc.MotionController(self.__lowLevelCon)
        self.__taskMgr = tm.TaskManager(simState, self.__motionCon.getService())

    def tick(self, model, data, simState):
        self.__taskMgr.tick()
        self.__motionCon.tick(simState.controllerState)
        self.__lowLevelCon.tick(model, data)

    def load(self, model, data):
        return 

    def unload(self, qcmd):
        return 

    #to be refactored
    def getTaskService(self) -> tm.tms.TaskManagerService: 
        return self.__taskMgr.getService()

    #to be refactored
    def getCmdPos(self) -> list[float]:
        return self.__lowLevelCon.getCmdPos()

    #to be refactored
    def getCmdVel(self) -> list[float]:
        return self.__lowLevelCon.getCmdVel()
