#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

int PathPlanner::plan(const PathPlanningInput& input, WayPoints& points)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(DOF);
    ob::RealVectorBounds bounds(DOF);
    for(int i=0;i<DOF;++i){ bounds.setLow(i,-M_PI); bounds.setHigh(i,M_PI);}
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State*){ return true; });
    ss.setPlanner(std::make_shared<og::FMT>(ss.getSpaceInformation()));
//    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
	ss.getPlanner()->as<og::FMT>()->setNumSamples(20);
	ss.getPlanner()->as<og::FMT>()->setRadiusMultiplier(0.8);
    ob::ScopedState<> start(space), goal(space);
	for(unsigned i =0; i< DOF; ++i){
		start[i] = input.start[i];
		goal[i] = input.goal[i];
	}
    ss.setStartAndGoalStates(start, goal);

    if(ss.solve(1.0)) {
        const auto& path = ss.getSolutionPath();
		for(std::size_t k=0;k<path.getStateCount();++k){
			const auto* st = path.getState(k)->as<ob::RealVectorStateSpace::StateType>();
			WayPoint q;
			for(int i=0;i<DOF;++i) q[i]=st->values[i];
			points.push_back(q);
		}
		return 0;
    }
    return -1;
}
