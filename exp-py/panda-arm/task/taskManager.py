import numpy as np
from task import taskManagerService as tms
from motioncon import motionControllerService as mcs
import simState as ss

import dataLogger as dl

class SingleSignedProfiler:
    @classmethod
    def generateTraj(cls, 
                     startPos : float,  # deg
                     targetPos : float,  # deg
                     steps : int
                     ) -> mcs.mr.mo.Trajectory :   # points[rad]
        traj = mcs.mr.mo.Trajectory()
        for i in range(steps):
            rate : float = float(i/steps)
            p = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
            point = mcs.mr.mo.Point(i, np.deg2rad(p)) # [t, p[rad]]
            traj.push(point)
        return traj

class MultiSignedProfiler:
    @classmethod
    def generateTraj(cls,
                     startPos : np.ndarray,  # [deg]
                     targetPos : np.ndarray,  # [deg]
                     steps : int
                     ) -> mcs.mr.mo.Trajectory :   # points[rad]
        traj = mcs.mr.mo.Trajectory()
        for i in range(steps):
            rate : float = float(i/steps)
            p : np.ndarray = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
            point = mcs.mr.mo.Point(i, np.deg2rad(p)) # [t, p[rad]]
            traj.push(point)
        return traj


class TaskManager:
    def __init__(self,
                 simState : ss.SimState,
                 motionControlService : mcs.MotionControllerService):
        self.__service = tms.TaskManagerService()
        self.__motionControlService = motionControlService
        self.__simState = simState

    def getService(self):
        return self.__service

    def tick(self):
        if self.__service.hasRequest():
            req : tms.tr.TaskRequest = self.__service.popRequest()
            type : tms.tr.TaskRequestType =  req.getType()
            args = req.getArgs()
            # parse arguments
            if type == tms.tr.TaskRequestType.SINGLE_JOINT_MOVE: 
                # parse arguments
                qno : int = args.get() # qno
                index = qno - 1
                startPos : float = self.__simState.qs()[index] # [deg]
                targetPos : float = args.get() # [deg]
                T = 1000 # points num                
                traj = SingleSignedProfiler.generateTraj(startPos, targetPos, T)
                motion = mcs.mr.mo.Motion(traj)

                # generate Traj
                #traj = mcs.mr.mo.Trajectory()
                ## temporal trajectory generation ##
                # for debugging
                #plistDbg =[]
                #tlistDbg =[]

#                for i in range(T):
#                    rate : float = float(i/T)
#                    p = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
#                    # for debugging
#                    #tlistDbg.append(i)
#                    #plistDbg.append(p)                    
#                    point = mcs.mr.mo.Point(i, np.deg2rad(p)) # [t, p[rad]]
#                    traj.push(point)               
#                motion = mcs.mr.mo.Motion(traj)
                # for debugging
                #dl.Graph.quickShow(tlistDbg, plistDbg)

                # create motionRequest
                request : mcs.mr.MotionRequest = mcs.mr.SingleJointMotionRequest(qno, motion)
                self.__motionControlService.pushRequest(request)
           
            elif type == tms.tr.TaskRequestType.MULTI_JOINT_MOVE:
                targets : list[tuple[int, float]] = args.get() # [deg]
                qnos : list[int] = []
                s : list[float] = []
                t : list[float] = []
                for target in targets:
                    qno : int = target[0]
                    targetPos : float = target[1]
                    index = qno - 1
                    startPos : float = self.__simState.qs()[index] # [deg]
                    qnos.append(qno)
                    s.append(startPos)
                    t.append(targetPos)

                T = 1000 # points num                
                traj = MultiSignedProfiler.generateTraj(np.array(s), np.array(t), T)
                motion = mcs.mr.mo.Motion(qnos, traj)
                # create motionRequest
                request : mcs.mr.MotionRequest = mcs.mr.MultiJointMotionRequest(motion)
                self.__motionControlService.pushRequest(request)
                    
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()
                

