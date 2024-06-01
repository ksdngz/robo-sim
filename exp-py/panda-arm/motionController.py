import motionControllerService as mcs
import lowLevelController as llc

class MotionController:
    def __init__(self, lowLevelCon : llc.LowLevelController):
        self.__service = mcs.MotionControllerService()
        self.__execMotion : tuple[int, mcs.mr.mo.Motion] = None
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
                    self.__execMotion = (qno, motion)
        
        # tick
        if self.__execMotion is not None:
            qno, motion = self.__execMotion
            if not motion.traj().isEmpty():
                point = motion.traj().pop() 
                cmdPos = self.__llc.getCmdPos()
                index = qno-1
                cmdPos[index] = point.pos            
                self.__llc.update(cmdPos)
            else:
                self.__execMotion = None
