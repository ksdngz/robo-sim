from motioncon import motionControllerService as mcs
from lowlevelcon import lowLevelController as llc

class MotionController:
    def __init__(self, lowLevelCon : llc.LowLevelController):
        self.__service = mcs.MotionControllerService()
        self.__execMotion : mcs.mr.mo.Motion = None
        self.__llc = lowLevelCon
    
    def getService(self):
        return self.__service

    def tick(self):
        if self.__execMotion == None:
            if self.__service.hasRequest():
                req : mcs.mr.MotionRequest = self.__service.popRequest()
                type : mcs.mr.MotionRequestType =  req.getType()
                args = req.getArgs()
                # parse arguments
                if type == mcs.mr.MotionRequestType.SINGLE_JOINT_MOTION: 
                    # parse arguments
                    qno : int = args.get() # qno
                    motion : mcs.mr.mo.Motion = args.get() # motion
                    
                    # set executed motion
                    self.__execMotion = motion

                elif type == mcs.mr.MotionRequestType.MULTI_JOINT_MOTION: 
                    # parse arguments
                    motion : mcs.mr.mo.Motion = args.get() # motion
                    
                    # set executed motion
                    self.__execMotion = motion

        # tick
        if self.__execMotion is not None:
            motion = self.__execMotion
            if not motion.traj.isEmpty():
                point = motion.traj.pop()
                cmdPos = self.__llc.getCmdPos()
                for i, qno in enumerate(motion.qnos):
                    index = qno-1
                    cmdPos[index] = point.ps[i]
                self.__llc.update(cmdPos)
            else:
                self.__execMotion = None
