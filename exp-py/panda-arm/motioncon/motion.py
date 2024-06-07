import queue
import numpy as np

class Point:
    def __init__(self, 
                 t, 
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

    def pop(self):
        return self.__points.get()
    
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
