import queue
from enum import Enum

class TaskRequestType(Enum):
    NONE                = 0
    SINGLE_JOINT_MOVE   = 1
    MULTI_JOINT_MOVE    = 2

class TaskRequest:
    def __init__(self, requstType: TaskRequestType):
        self.__type = requstType
        self.__args = queue.Queue()

    def getType(self):
        return self.__type

    def getArgs(self):
        return self.__args

class SingleJointMoveRequest(TaskRequest):
    def __init__(self, jno:int, targetPos:float):
        super().__init__(TaskRequestType.SINGLE_JOINT_MOVE)
        self.getArgs().put(jno)
        self.getArgs().put(targetPos)
        
class MultiJointMoveRequest(TaskRequest):
    def __init__(self, targets : list[tuple[int, float]]): # targets: list[(qno, targetPos)]
        super().__init__(TaskRequestType.MULTI_JOINT_MOVE)
        self.getArgs().put(targets)
