import queue

class Point:
    def __init__(self, t, pos):        
        self.time = t
        self.pos = pos

class Trajectory:

    def __init__(self):
        self.__points = queue.Queue()
    
    def isEmpty(self):
        return self.__points.empty()

    def push(self, point):
        self.__points.put(point)

    def pop(self):
        return self.__points.get()
    
class Motion:
    def __init__(self, traj : Trajectory):
        self.__traj = traj
    def traj(self): # todo
        return self.__traj
    
