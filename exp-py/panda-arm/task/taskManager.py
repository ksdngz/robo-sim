import numpy as np
#from scipy.spatial.transform import Rotation
from spatialmath import SE3
from task import taskManagerService as tms
from motioncon import motionControllerService as mcs
import simState as ss
from external import rtbWrapper as rtb

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

    def __pushMultiJointMoveRequest(self, targets : list[tuple[int, float]]):
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
                # create motionRequest
                request : mcs.mr.MotionRequest = mcs.mr.SingleJointMotionRequest(qno, motion)
                self.__motionControlService.pushRequest(request)
           
            elif type == tms.tr.TaskRequestType.MULTI_JOINT_MOVE:
                targets : list[tuple[int, float]] = args.get() # [deg]
                self.__pushMultiJointMoveRequest(targets)

            elif type == tms.tr.TaskRequestType.MULTI_JOINT_MOVE_TCP:
                # tcpTarget : list[float] = args.get() # [deg]
                tcpTarget : np.ndarray = np.array(args.get())# [deg]
                x = tcpTarget[0]
                y = tcpTarget[1]
                z = tcpTarget[2]
                ro = np.deg2rad(tcpTarget[3])
                pi = np.deg2rad(tcpTarget[4])
                ya = np.deg2rad(tcpTarget[5])
                rot = SE3.RPY(ro,pi,ya)                
                trans = SE3.Trans(x,y,z)
                target = trans * rot
                q0 : np.ndarray = np.array(np.deg2rad(self.__simState.qs())) # [rad]
                q : np.ndarray = rtb.inverseKin(target, q0) # [rad]
                jntTarget : list[float] = np.rad2deg(q) #[deg]
                jnos = list(range(1,8)) # todo
                targets : list[tuple[int, float]] = []
                for i,jno in enumerate(jnos):
                    targets.append((jno, jntTarget[i]))
                self.__pushMultiJointMoveRequest(targets)
                
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()
                

