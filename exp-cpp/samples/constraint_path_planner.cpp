#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

#include "./mj_sim.hpp"
#include "./constraint_path_planner.hpp"
#include "./path_planner.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;
constexpr int EE_BODY_ID = 10;       // MuJoCoモデル内のEEリンクIDに変更

ConstraintPathPlanner::ConstraintPathPlanner(MjSim& mj)
 : mj_(mj){}

// ---- FK: q -> EE位置 ----
Eigen::Vector3d ConstraintPathPlanner::fkEE(const Eigen::VectorXd& q) {
    for(int i=0;i<DOF && i<q.size();++i) mj_.d->qpos[i] = q[i];
    mj_forward(mj_.m, mj_.d);
    return Eigen::Map<Eigen::Vector3d>(mj_.d->xpos + 3*EE_BODY_ID);
}

// ---- 数値IK（EE位置一致のみ、ヤコビ行列逆を利用）----
bool ConstraintPathPlanner::ikSolve(const Eigen::Vector3d& target, Eigen::VectorXd& q_out, int maxIter, double tol) {
    q_out = Eigen::Map<Eigen::VectorXd>(mj_.d->qpos, DOF); // 現在値から開始
    for(int iter=0; iter<maxIter; ++iter) {
        Eigen::Vector3d p = fkEE(q_out);
        Eigen::Vector3d err = target - p;
        if(err.norm() < tol) return true;
        // 位置ヤコビ取得
        Eigen::Matrix<double,3,DOF> Jpos;
        mj_jacBodyCom(mj_.m, mj_.d, Jpos.data(), nullptr, EE_BODY_ID);
        // 疑似逆行列で更新
        Eigen::MatrixXd Jpinv = Jpos.transpose() * ( (Jpos * Jpos.transpose()).inverse() );
        q_out += Jpinv * err;
    }
    return false;
}

// ---- 直線近似 ----
std::vector<Eigen::Vector3d> ConstraintPathPlanner::fitLine(const std::vector<Eigen::Vector3d>& pts) {
    Eigen::Vector3d p0=pts.front(), p1=pts.back();
    std::vector<Eigen::Vector3d> out;
    size_t N = pts.size();
    for(size_t i=0;i<N;++i){
        double t = double(i)/(N-1);
        out.push_back((1-t)*p0 + t*p1);
    }
    return out;
}

// ---- 円弧近似 ----
std::vector<Eigen::Vector3d> ConstraintPathPlanner::fitArc(const std::vector<Eigen::Vector3d>& pts) {
    Eigen::Vector3d p0 = pts.front();
    Eigen::Vector3d pm = pts[pts.size()/2];
    Eigen::Vector3d p1 = pts.back();
    Eigen::Vector3d normal = (pm - p0).cross(p1 - pm).normalized();
    Eigen::Vector3d center = (p0 + pm + p1) / 3.0;
    double radius = (p0 - center).norm();
    Eigen::Vector3d ux = (p0 - center).normalized();
    Eigen::Vector3d uy = normal.cross(ux);
    auto angleOf = [&](const Eigen::Vector3d& p){
        Eigen::Vector3d v = (p - center) / radius;
        return std::atan2(v.dot(uy), v.dot(ux));
    };
    double th0 = angleOf(p0);
    double th1 = angleOf(p1);
    if(th1<th0) th1 += 2*M_PI;
    std::vector<Eigen::Vector3d> out;
    size_t N = pts.size();
    for(size_t i=0;i<N;++i){
        double t = double(i)/(N-1);
        double th = th0 + t*(th1-th0);
        out.push_back(center + radius*(std::cos(th)*ux + std::sin(th)*uy));
    }
    return out;
}

int ConstraintPathPlanner::plan()
{
	auto pathPlanner = std::make_shared<PathPlanner>();
	PathPoints joint_path;
	int ret = pathPlanner->plan(joint_path);

    // FKでEE位置列に変換
	std::vector<Eigen::Vector3d> ee_pts;
	for(std::size_t k=0;k<joint_path.point.size();++k){
		const auto& q = joint_path.point[k];
        // PathPoint(std::array<double,DOF>) -> Eigen::VectorXd
        Eigen::VectorXd qvec(DOF);
        for(int i=0;i<DOF;++i) qvec[i] = q[i];
        ee_pts.push_back(fkEE(qvec));
	}	
	std::cout<<"Original joint path: "<<joint_path.point.size()<<"  FK EE path: "<<ee_pts.size()<<"\n";

	// 円弧フィット（直線は fitLine に差し替え）
	auto fitted = fitArc(ee_pts);

	// IKで関節軌道に戻す
    std::vector<Eigen::VectorXd> joint_traj;
	for(auto& p : fitted){
		Eigen::VectorXd qsol;
		if(ikSolve(p,qsol)) joint_traj.push_back(qsol);
	}
	std::cout<<"Original: "<<ee_pts.size()<<"  Arc fitted: "<<joint_traj.size()<<"\n";
    return 0;
}
