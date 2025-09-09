#pragma once
#include <Eigen/Dense>

class Position
{
public:
	double x;
	double y;
	double z;

	// constructors (rule of five minimal set)
	Position() = default;
	Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
	Position(const Position&) = default;
	Position(Position&&) noexcept = default;
	Position(const Eigen::Vector3d& vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}
	Position(Eigen::Vector3d& vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}

	Position operator+(const Position& rhs) const {return {x + rhs.x, y + rhs.y, z + rhs.z};}
	Position operator-(const Position& rhs) const {return {x - rhs.x, y - rhs.y, z - rhs.z};}
	operator std::array<double,3>() const { return {x, y, z};}
	double norm2() const { return std::sqrt(x * x + y * y + z * z);}

	// -------- assignment operators --------
	Position& operator=(const Position& other) = default;
	Position& operator=(Position&& other) noexcept = default;
	Position& operator=(const std::array<double,3>& arr) {
		x = arr[0]; y = arr[1]; z = arr[2];
		return *this;
	}
	Position& operator=(const Eigen::Vector3d& vec) {
		x = vec[0]; y = vec[1]; z = vec[2];
		return *this;
	}
	Position& operator=(Eigen::Vector3d& vec) {
		x = vec[0]; y = vec[1]; z = vec[2];
		return *this;
	}
	// assign from initializer_list<double> of size 3: {x,y,z}
	Position& operator=(std::initializer_list<double> list) {
		if (list.size() == 3) {
			auto it = list.begin();
			x = *it++; y = *it++; z = *it;
		}
		return *this;
	}

	// -------- compound assignment --------
	Position& operator+=(const Position& rhs) { x+=rhs.x; y+=rhs.y; z+=rhs.z; return *this; }
	Position& operator-=(const Position& rhs) { x-=rhs.x; y-=rhs.y; z-=rhs.z; return *this; }
	Position& operator*=(double s) { x*=s; y*=s; z*=s; return *this; }
	Position& operator/=(double s) { x/=s; y/=s; z/=s; return *this; }

	// non-const returning scalar ops
	friend Position operator*(Position lhs, double s){ lhs*=s; return lhs; }
	friend Position operator*(double s, Position rhs){ rhs*=s; return rhs; }
	friend Position operator/(Position lhs, double s){ lhs/=s; return lhs; }
};
