#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "./path_planner.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

PathPlanner::PathPlanner()
{
    // コンストラクタの実装
}

int PathPlanner::plan(PathPoints& points)
{
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
		for(std::size_t k=0;k<path.getStateCount();++k){
			const auto* st = path.getState(k)->as<ob::RealVectorStateSpace::StateType>();
			PathPoint q;
			for(int i=0;i<DOF;++i) q[i]=st->values[i];
			points.point.push_back(q);
		}
		return 0;
    }
    return -1;
}
