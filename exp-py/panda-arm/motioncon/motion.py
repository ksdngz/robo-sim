import queue
import numpy as np
import copy

class Point:
    def __init__(self, 
                 t : int,
                 ps : np.ndarray):        
        self.time = t
        self.ps = ps

class Trajectory:
    def __init__(self) -> None:
        self.__points = queue.Queue()
    
    def isEmpty(self) -> bool:
        return self.__points.empty()

    def push(self, point : list[float]):
        self.__points.put(point)

    def pop(self) -> Point:
        return self.__points.get()
    
    def pointList(self) -> list[tuple[int, float]]:
        plist : list[Point] = copy.deepcopy(list(self.__points.queue))
        print(repr(plist[0]), repr(self.__points.queue[0])) 
        # points = copy.deepcopy(self.__points)
        # print(repr(points), repr(self.__points)) 
        # return [(i, points.get()) for i in points.qsize()]
        return plist
    
class Motion:
    def __init__(self, 
                 qnos : list[int], 
                 traj : Trajectory):
        self.__qnos = qnos
        self.__traj = traj

    @property
    def traj(self) -> Trajectory:
        return self.__traj
    
    @property
    def qnos(self) -> list[int]:
        return self.__qnos
