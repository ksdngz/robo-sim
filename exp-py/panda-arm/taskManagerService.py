import queue
import taskRequest as tr

class TaskManagerService:
    def __init__(self):
        self.__requests = queue.Queue()
    
    def pushRequest(self, request : tr.TaskRequest):
        self.__requests.put(request)
    
    def hasRequest(self):
        return not self.__requests.empty()
    
    def popRequest(self):
        return self.__requests.get()
