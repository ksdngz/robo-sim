#include <mujoco/mujoco.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

constexpr int DOF = 7;               // ロボットの関節自由度
constexpr int EE_BODY_ID = 10;       // MuJoCoモデル内のEEリンクIDに変更
mjModel* m = nullptr;
mjData*  d = nullptr;

// ---- FK: q -> EE位置 ----
Eigen::Vector3d fkEE(const Eigen::VectorXd& q) {
    for(int i=0;i<DOF;++i) d->qpos[i] = q[i];
    mj_forward(m, d);
    return Eigen::Map<Eigen::Vector3d>(d->xpos + 3*EE_BODY_ID);
}

// ---- 数値IK（EE位置一致のみ、ヤコビ行列逆を利用）----
bool ikSolve(const Eigen::Vector3d& target, Eigen::VectorXd& q_out, int maxIter=100, double tol=1e-4) {
    q_out = Eigen::Map<Eigen::VectorXd>(d->qpos, DOF); // 現在値から開始
    for(int iter=0; iter<maxIter; ++iter) {
        Eigen::Vector3d p = fkEE(q_out);
        Eigen::Vector3d err = target - p;
        if(err.norm() < tol) return true;
        // 位置ヤコビ取得
        Eigen::Matrix<double,3,DOF> Jpos;
        mj_jacBodyCom(m, d, Jpos.data(), nullptr, EE_BODY_ID);
        // 疑似逆行列で更新
        Eigen::MatrixXd Jpinv = Jpos.transpose() * ( (Jpos * Jpos.transpose()).inverse() );
        q_out += Jpinv * err;
    }
    return false;
}

// ---- 直線近似 ----
std::vector<Eigen::Vector3d> fitLine(const std::vector<Eigen::Vector3d>& pts) {
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
std::vector<Eigen::Vector3d> fitArc(const std::vector<Eigen::Vector3d>& pts) {
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

int main(int argc,char** argv){
    if(argc<2){ std::cerr<<"Usage: "<<argv[0]<<" model.xml\n"; return 1;}
//    mj_activate("mjkey.txt");
    m = mj_loadXML(argv[1], nullptr, nullptr, 0);
    if(!m){ std::cerr<<"Model load failed\n"; return 1;}
    d = mj_makeData(m);

    // --- OMPLセットアップ ---
    auto space = std::make_shared<ob::RealVectorStateSpace>(DOF);
    ob::RealVectorBounds bounds(DOF);
    for(int i=0;i<DOF;++i){ bounds.setLow(i,-M_PI); bounds.setHigh(i,M_PI);}
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State*){ return true; });
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    ob::ScopedState<> start(space), goal(space);
    for(int i=0;i<DOF;++i){ start[i]=0.0; goal[i]=0.5; }
    ss.setStartAndGoalStates(start, goal);

    if(ss.solve(1.0)) {
        const auto& path = ss.getSolutionPath();

        // FKでEE位置列に変換
        std::vector<Eigen::Vector3d> ee_pts;
        for(std::size_t k=0;k<path.getStateCount();++k){
            const auto* st = path.getState(k)->as<ob::RealVectorStateSpace::StateType>();
            Eigen::VectorXd q(DOF);
            for(int i=0;i<DOF;++i) q[i]=st->values[i];
            ee_pts.push_back(fkEE(q));
        }

        // 円弧フィット（直線は fitLine に差し替え）
        auto fitted = fitArc(ee_pts);

        // IKで関節軌道に戻す
        std::vector<Eigen::VectorXd> joint_traj;
        for(auto& p : fitted){
            Eigen::VectorXd qsol;
            if(ikSolve(p,qsol)) joint_traj.push_back(qsol);
        }
        std::cout<<"Original: "<<ee_pts.size()<<"  Arc fitted: "<<joint_traj.size()<<"\n";
    }

    mj_deleteData(d);
    mj_deleteModel(m);
//    mj_deactivate();
    return 0;
}
