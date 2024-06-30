import copy
import toml
import mujoco as mj

from common import common_constants as const 
from robotcon import robotControllerService as rcs
from lowlevelcon import lowLevelController as llc
from motioncon import motionController as mc
from task import taskManager as tm
import simState as ss

class RobotController:
    def tick(self, model, data, simState):
        print('Error: superClass method is called.')
        assert()

    def load(self, configPath: str, model, data): # todo
        print('Error: superClass method is called.')
        assert()
    
    def unload(self):
        print('Error: superClass method is called.')
        assert()
    
    def getRobotControllerService(self) -> rcs.RobotControllerService: 
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
    def __init__(self, configPath: str, model, data, simState):
        self.__configPath = configPath
        self.__robotControllerService = rcs.RobotControllerService()
        initq = copy.copy(const.HOME_JOINTS)
        kp = 100; kd = 0; ki = 0 # only K_p
        self.__lowLevelCon = llc.PIDController(model, data, kp, kd, ki)
        self.__lowLevelCon.update(initq)
        self.__motionCon = mc.MotionController(self.__lowLevelCon)
        self.__taskMgr = tm.TaskManager(simState, self.__motionCon.getService())

    def processRequest(self, model, data):
        if self.__robotControllerService.hasRequest():
            req : rcs.rcr.RobotControllerRequest = self.__robotControllerService.popRequest()
            type : rcs.rcr.RobotControllerRequestType =  req.getType()
            args = req.getArgs()
            if type ==rcs.rcr.RobotControllerRequestType.LOAD:
                config: str = args.get() # not used at this moment
                self.load(model, data)                
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()

    def tick(self, model, data, simState):
        self.processRequest(model, data)
        self.__taskMgr.tick()
        self.__motionCon.tick(simState.controllerState)
        self.__lowLevelCon.tick(model, data)

    def load(self,
             model, 
             data,
             isInitCmdPos: bool = False):
        with open(self.__configPath) as f:
            obj = toml.load(f)
            #print(obj)

        # lowlevelcon
        kp: float = obj['lowlevelcon']['pid']['pgain']
        kd: float = obj['lowlevelcon']['pid']['igain']
        ki: float = obj['lowlevelcon']['pid']['dgain']

        self.__lowLevelCon.load(kp, kd, ki)
        if isInitCmdPos is True:
            initq = copy.copy(const.HOME_JOINTS)
            self.__lowLevelCon.update(initq)

    def unload(self):
        return 

    def getRobotControllerService(self) -> rcs.RobotControllerService: 
        return self.__robotControllerService

    #to be refactored
    def getTaskService(self) -> tm.tms.TaskManagerService: 
        return self.__taskMgr.getService()

    #to be refactored
    def getCmdPos(self) -> list[float]:
        return self.__lowLevelCon.getCmdPos()

    #to be refactored
    def getCmdVel(self) -> list[float]:
        return self.__lowLevelCon.getCmdVel()
