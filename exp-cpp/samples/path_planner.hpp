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

constexpr int DOF = 7;               // ロボットの関節自由度
using PathPoint = std::array<double, DOF>;
struct PathPoints{
	std::vector<PathPoint> point;
};

class PathPlanner{
public:
	PathPlanner();
	int plan(PathPoints& points);

};
