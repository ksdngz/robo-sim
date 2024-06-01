import queue

class Trajectory:
    class Point:
        def __init__(self, t, pos):        
            self.time = t
            self.pos = pos

    def __init__(self):
        self.__points = queue.Queue()
    
    def isEmpty(self):
        return self.__points.empty()

    def push(self, point : Point):
        self.__points.put(point)

    def pop(self):
        self.__points.get()
    
class Motion:
    def __init__(self, traj : Trajectory):
        self.__traj = traj
    
    
