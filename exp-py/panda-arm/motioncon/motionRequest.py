import queue
from enum import Enum
from motioncon import motion as mo

class MotionRequestType(Enum):
    NONE = 0
    SINGLE_JOINT_MOTION = 1
    MULTI_JOINT_MOTION = 2

class MotionRequest:
    def __init__(self, requstType: MotionRequestType):
        self.__type = requstType
        self.__args = queue.Queue()

    def getType(self):
        return self.__type

    def getArgs(self):
        return self.__args
    
class SingleJointMotionRequest(MotionRequest):
    def __init__(self, qno: int, motion : mo.Motion):
        super().__init__(MotionRequestType.SINGLE_JOINT_MOTION)
        self.getArgs().put(qno)
        self.getArgs().put(motion)

class MultiJointMotionRequest(MotionRequest):
    def __init__(self, motion : mo.Motion):
        super().__init__(MotionRequestType.MULTI_JOINT_MOTION)
        self.getArgs().put(motion)
    

