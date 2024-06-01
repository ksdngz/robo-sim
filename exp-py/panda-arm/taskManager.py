import numpy as np
import taskManagerService as tms
import motionControllerService as mcs
import simState as ss

class TaskManager:
    def __init__(self,
                 simState : ss.SimState,
                 motionControlService : mcs.MotionControllerService):
        self.__service = tms.taskManagerService()
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
                targetPos : float = args.get() # targetPos

                # generate Traj
                traj = mcs.mr.mo.Trajectory
                startPos : float = self.__simState.qs()[qno]
                ## temporal trajectory generation ##
                T = 1000. # points num
                for i in range(T):            
                    p = startPos + (1 - np.cos(i/T))
                    traj.push(i, p)
                motion = mcs.mr.mo.Motion(traj)

                # create motionRequest
                request : mcs.mr.MotionRequest = mcs.mr.SingleJointMotionRequest(qno, motion)
                self.__motionControlService.pushRequest(request)
                
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()
                

