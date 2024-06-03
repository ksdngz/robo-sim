import queue
from motioncon import motionRequest as mr

class MotionControllerService:
    def __init__(self):
        self.__requests = queue.Queue()
    
    def pushRequest(self, request : mr.MotionRequest):
        self.__requests.put(request)

    def hasRequest(self):
        return not self.__requests.empty()

    def popRequest(self):
        return self.__requests.get()
