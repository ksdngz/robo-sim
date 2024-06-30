import queue
from enum import Enum

class RobotControllerRequestType(Enum):
    NONE                    = 0
    LOAD                    = 1

class RobotControllerRequest:
    def __init__(self, requstType: RobotControllerRequestType):
        self.__type = requstType
        self.__args = queue.Queue()

    def getType(self):
        return self.__type

    def getArgs(self):
        return self.__args
        
class LoadRequest(RobotControllerRequest):
    def __init__(self, config: str = None):
        super().__init__(RobotControllerRequestType.LOAD)
        self.getArgs().put(config)
