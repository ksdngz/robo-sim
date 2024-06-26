import numpy as np
from spatialmath import SE3
from spatialmath.base import tr2rpy, rpy2tr
from task import taskManagerService as tms
from motioncon import motionControllerService as mcs
from planner import trajectoryProfiler as trajprof
import simState as ss
from external.kinetics import Kinematics as kin
from common import common_constants
from common.pose6d import Pose6d

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
            s.append(np.deg2rad(startPos)) # [rad]
            t.append(np.deg2rad(targetPos)) # [rad]

        T = 1000 # points num                
        traj = trajprof.MultiSignedProfiler.generateTraj(np.array(s), np.array(t), T)
        motion = mcs.mr.mo.Motion(qnos, traj)
        request : mcs.mr.MotionRequest = mcs.mr.MultiJointMotionRequest(motion)
        self.__motionControlService.pushRequest(request)
    
    def tick(self):
        if self.__service.hasRequest():
            req : tms.tr.TaskRequest = self.__service.popRequest()
            type : tms.tr.TaskRequestType =  req.getType()
            args = req.getArgs()
            if type == tms.tr.TaskRequestType.MULTI_JOINT_MOVE:
                targets : list[tuple[int, float]] = args.get() # [deg]
                self.__pushMultiJointMoveRequest(targets)

            elif type == tms.tr.TaskRequestType.MULTI_JOINT_MOVE_TCP:
                target : Pose6d = args.get()
                q0 : np.ndarray = np.array(np.deg2rad(self.__simState.qs())) # [rad]
                q : np.ndarray = kin.inverseKin(target, q0) # [rad]
                jntTarget : list[float] = np.rad2deg(q) #[deg]
                jnos = list(range(1,8)) # todo
                targets : list[tuple[int, float]] = []
                for i,jno in enumerate(jnos):
                    targets.append((jno, jntTarget[i]))
                self.__pushMultiJointMoveRequest(targets)
            
            elif type == tms.tr.TaskRequestType.TCP_MOVE_STRAIGHT:
                # tcpTarget : np.ndarray = np.array(args.get())# [deg]
                tcpTarget : Pose6d = args.get()
                tcp0 : Pose6d = self.__simState.tcpPose()
                q0 : np.ndarray = np.array(np.deg2rad(self.__simState.qs())) # [rad]
                T = 1000
                tcpTraj : mcs.mr.mo.Trajectory = trajprof.MultiSignedProfiler.generateTraj(tcp0, tcpTarget, T)
                qtraj  = mcs.mr.mo.Trajectory()
                print('start: generating trajectory')
                print('start: q0', np.rad2deg(q0))
                while not tcpTraj.isEmpty():
                    tcpPoint : mcs.mr.mo.Point = tcpTraj.pop()
                    ps = Pose6d(tcpPoint.ps) # tcpPoint.ps[rad]
                    q : np.ndarray = kin.inverseKin(ps, q0) # [rad]                   
                    qtraj.push(mcs.mr.mo.Point(tcpPoint.time, q))
                    q0 = q
                    
                    print('ik: time, q', tcpPoint.time, np.rad2deg(q))

                # todo debug
                print('end: generating trajectory')
                qnos : list[int] = list(range(1, common_constants.JOINT_NUM+1))
                motion = mcs.mr.mo.Motion(qnos, qtraj)
                request : mcs.mr.MotionRequest = mcs.mr.MultiJointMotionRequest(motion)
                self.__motionControlService.pushRequest(request)
                
            else:
                print('Error: Not defined taskRequest was pushed.')
                assert()
                

