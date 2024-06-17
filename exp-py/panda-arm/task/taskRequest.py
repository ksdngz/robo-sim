import queue
from enum import Enum
from common.pose3d import Pose3d

class TaskRequestType(Enum):
    NONE                    = 0
#   SINGLE_JOINT_MOVE       = 1
    MULTI_JOINT_MOVE        = 2
    MULTI_JOINT_MOVE_TCP    = 3
    TCP_MOVE_STRAIGHT       = 4

class TaskRequest:
    def __init__(self, requstType: TaskRequestType):
        self.__type = requstType
        self.__args = queue.Queue()

    def getType(self):
        return self.__type

    def getArgs(self):
        return self.__args
        
class MultiJointMoveRequest(TaskRequest):
    def __init__(self, targets : list[tuple[int, float]]): # targets: list[(qno, targetPos)] [deg]
        super().__init__(TaskRequestType.MULTI_JOINT_MOVE)
        self.getArgs().put(targets)

class MultiJointMoveTcpRequest(TaskRequest):
    def __init__(self, tcp : Pose3d):
        super().__init__(TaskRequestType.MULTI_JOINT_MOVE_TCP)
        self.getArgs().put(tcp)

class TCPMoveStraightRequest(TaskRequest):
    def __init__(self, tcp : Pose3d):
        super().__init__(TaskRequestType.TCP_MOVE_STRAIGHT)
        self.getArgs().put(tcp)
