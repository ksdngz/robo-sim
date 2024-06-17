import numpy as np
#from scipy.spatial.transform import Rotation
from spatialmath import SE3
from spatialmath.base import tr2rpy, rpy2tr
from task import taskManagerService as tms
from motioncon import motionControllerService as mcs
from planner import trajectoryProfiler as trajprof
import simState as ss
from external import rtbWrapper as rtb
from common import common_constants
from common.pose3d import Pose3d

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
        # create motionRequest
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
                # tcpTarget : list[float] = args.get() # [deg]
                target : Pose3d = args.get()
#                tcpTarget : np.ndarray = np.array(args.get())# [deg]
#                target : SE3 = se3(rot_deg2rad(tcpTarget)) # SE3 [rad]
#                x = tcpTarget[0]
#                y = tcpTarget[1]
#                z = tcpTarget[2]
#                ro = np.deg2rad(tcpTarget[3])
#                pi = np.deg2rad(tcpTarget[4])
#                ya = np.deg2rad(tcpTarget[5])
##                rot = SE3.RPY(ro,pi,ya)
#                gamma = [ro,pi,ya]
#                rot = SE3.RPY(gamma, order="xyz")
##                rot = SE3.RPY(gamma, order="zyx")
#                          
#                trans = SE3.Trans(x,y,z)
#                target = trans * rot
#                print('tcpTarget', target)
                q0 : np.ndarray = np.array(np.deg2rad(self.__simState.qs())) # [rad]
                q : np.ndarray = rtb.inverseKin(target, q0) # [rad]
                jntTarget : list[float] = np.rad2deg(q) #[deg]
                jnos = list(range(1,8)) # todo
                targets : list[tuple[int, float]] = []
                for i,jno in enumerate(jnos):
                    targets.append((jno, jntTarget[i]))
                self.__pushMultiJointMoveRequest(targets)
            
            elif type == tms.tr.TaskRequestType.TCP_MOVE_STRAIGHT:
                # tcpTarget : np.ndarray = np.array(args.get())# [deg]
                tcpTarget : Pose3d = args.get()
                tcp0 : Pose3d = self.__simState.tcpPose()
                #tcp0 : np.ndarray = np.array(self.__simState.tcpPose())# [deg]
                # todo to change tcp rotation from deg to rad
                # rot_deg2rad = lambda pose: pose[0:3] + [np.deg2rad(r) for r in pose[3:6]]
#                tcpTarget = rot_deg2rad(tcpTarget) # eul(zyx) [rad]
                #tcp0 = rot_deg2rad(tcp0) # eul(zyx) [rad]
                q0 : np.ndarray = np.array(np.deg2rad(self.__simState.qs())) # [rad]               
#                p0 : SE3 = rtb.forwardKin(q0)                
#                print("p0", p0)
#                print("t0", se3(tcp0))
#                print("tt", se3(tcpTarget))
#                p0_eulzyx = tr2rpy(p0.R, unit='deg', order='zyx')
#                print("p0_eulzyx", p0_eulzyx)
                
                T = 1000
                tcpTraj : mcs.mr.mo.Trajectory = trajprof.MultiSignedProfiler.generateTraj(tcp0, tcpTarget, T)
                qtraj  = mcs.mr.mo.Trajectory()
                print('start: generating trajectory')
                print('start: q0', np.rad2deg(q0))
                while not tcpTraj.isEmpty():
                    tcpPoint : mcs.mr.mo.Point = tcpTraj.pop()
                    ps = Pose3d(tcpPoint.ps) # tcpPoint.ps[rad]
                    q : np.ndarray = rtb.inverseKin(ps, q0) # [rad]                   
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
                

