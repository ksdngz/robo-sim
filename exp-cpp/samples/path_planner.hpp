#pragma once

//#include <mujoco/mujoco.h>
//#include <ompl/base/spaces/RealVectorStateSpace.h>
//#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <Eigen/Dense>
//#include <vector>
//#include <iostream>
//#include <cmath>
//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
#include "robot_def.hpp"

using WayPoint = std::array<double, DOF>;
using WayPoints = std::vector<WayPoint>;

struct PathPlanningInput
{
	WayPoint start;
	WayPoint goal;
};

class PathPlanner{
public:
	PathPlanner();
	int plan(const PathPlanningInput& input, WayPoints& points);

};
