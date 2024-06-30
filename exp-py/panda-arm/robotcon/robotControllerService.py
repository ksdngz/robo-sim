import queue
from robotcon import robotControllerRequest as rcr

class RobotControllerService:
    def __init__(self):
        self.__requests = queue.Queue()
    
    def pushRequest(self, request : rcr.RobotControllerRequest):
        self.__requests.put(request)
    
    def hasRequest(self):
        return not self.__requests.empty()
    
    def popRequest(self):
        return self.__requests.get()
