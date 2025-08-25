#pragma once
#include <Eigen/Dense>
#include <vector>

class MjSim;

class ConstraintPathPlanner
{
public:
	ConstraintPathPlanner(MjSim& mj);
	int plan();

private:
	Eigen::Vector3d fkEE(const Eigen::VectorXd& q);
	bool ikSolve(const Eigen::Vector3d& target, Eigen::VectorXd& q_out, int maxIter=100, double tol=1e-4);
	std::vector<Eigen::Vector3d> fitLine(const std::vector<Eigen::Vector3d>& pts);
	std::vector<Eigen::Vector3d> fitArc(const std::vector<Eigen::Vector3d>& pts);
	MjSim& mj_;
};

