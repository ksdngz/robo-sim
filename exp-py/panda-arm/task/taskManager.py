import numpy as np
from task import taskManagerService as tms
from motioncon import motionControllerService as mcs
import simState as ss

import dataLogger as dl

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
                targetPos : float = args.get() # [deg]

                # generate Traj
                traj = mcs.mr.mo.Trajectory()
                index = qno - 1
                startPos : float = self.__simState.qs()[index] # [deg]
                ## temporal trajectory generation ##
                T = 1000 # points num                
                # for debugging
                #plistDbg =[]
                #tlistDbg =[]

                for i in range(T):
                    rate : float = float(i/T)
                    p = startPos + (targetPos-startPos)*(1 - np.cos(np.pi*rate))/2 # [deg]
                    # for debugging
                    #tlistDbg.append(i)
                    #plistDbg.append(p)                    
                    point = mcs.mr.mo.Point(i, np.deg2rad(p)) # [t, p[rad]]
                    traj.push(point)
                
                motion = mcs.mr.mo.Motion(traj)
                # for debugging
                #dl.Graph.quickShow(tlistDbg, plistDbg)

                # create motionRequest
                request : mcs.mr.MotionRequest = mcs.mr.SingleJointMotionRequest(qno, motion)
                self.__motionControlService.pushRequest(request)
                
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()
                

