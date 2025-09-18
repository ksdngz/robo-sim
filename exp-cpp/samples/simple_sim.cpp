// Simple MuJoCo simulation sample: visualize panda_arm_mjcf.xml
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <initializer_list>
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mjui.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <Python.h>
#include "robot_def.hpp"
#include "common/Position.hpp"
#include "common/SecondOrderDynamics.hpp"
#include "./mj_sim.hpp"
#include "./constraint_path_planner.hpp"

// Spline alias (3D)
using namespace Eigen;
using Spline3d = Eigen::Spline<double, 3>;

const double RADIUS_SPH = 0.02;

// global instance
MjSim mj(model_path);

// Utility methods
Eigen::VectorXd arrayToVector(const std::array<double, DOF> &a)
{
    Eigen::VectorXd v(static_cast<int>(DOF));
    for (std::size_t i = 0; i < DOF; ++i) v[static_cast<int>(i)] = a[i];
    return v;
}

std::array<double, DOF> vectorToArray(const Eigen::VectorXd &v, bool throwOnSizeMismatch = true)
{
    if (v.size() != static_cast<int>(DOF)) {
        if (throwOnSizeMismatch) throw std::runtime_error("vectorToArray: size mismatch");
        // 部分コピーで埋める場合: 残りはゼロで埋める
        std::array<double, DOF> out{};
        int copyN = std::min<int>(v.size(), static_cast<int>(DOF));
        for (int i = 0; i < copyN; ++i) out[i] = v[i];
        return out;
    }
    std::array<double, DOF> out;
    for (std::size_t i = 0; i < DOF; ++i) out[i] = v[static_cast<int>(i)];
    return out;
}


using Positions = std::vector<Position>;


// ---------------------- Refactor: Classes for Renderer, UI, and Drawing ----------------------

// Spline sampling used by GeometryPrims::referencePath
static inline void catmullRomSplinePoints(const Positions& wp,
										  std::vector<Position>& out_points,
										  int num_steps = 20) {
	out_points.clear();
	if (wp.size() < 2) {
		out_points = wp;
		return;
	}
	// duplicate endpoints for simple Catmull-Rom handling
	Positions ctrl;
	ctrl.reserve(wp.size() + 2);
	ctrl.push_back(wp.front());
	ctrl.insert(ctrl.end(), wp.begin(), wp.end());
	ctrl.push_back(wp.back());

	for (size_t i = 0; i + 3 < ctrl.size(); ++i) {
		const auto& p0 = ctrl[i+0];
		const auto& p1 = ctrl[i+1];
		const auto& p2 = ctrl[i+2];
		const auto& p3 = ctrl[i+3];
		for (int s = 0; s <= num_steps; ++s) {
			double t = static_cast<double>(s) / num_steps;
			double t2 = t * t;
			double t3 = t2 * t;
			auto blend = [&](double p0v, double p1v, double p2v, double p3v) {
				return 0.5 * ((2.0 * p1v) +
							  (-p0v + p2v) * t +
							  (2.0 * p0v - 5.0 * p1v + 4.0 * p2v - p3v) * t2 +
							  (-p0v + 3.0 * p1v - 3.0 * p2v + p3v) * t3);
			};
			Position q;
			q.x = blend(p0.x, p1.x, p2.x, p3.x);
			q.y = blend(p0.y, p1.y, p2.y, p3.y);
			q.z = blend(p0.z, p1.z, p2.z, p3.z);
			out_points.push_back(q);
		}
	}
}

// Geometry drawing utility: adds simple primitives to the scene.
class GeometryPrims {
public:
	GeometryPrims(mjvScene& scn, mjData* d) : scn_(scn), d_(d) {}

	bool geom(mjtGeom type,
			  const mjtNum size[3],
			  const mjtNum pos[3],
			  const mjtNum mat[9],
			  const float rgba[4],
			  std::string_view name) {
		if (scn_.ngeom >= scn_.maxgeom) {
			if (d_) mj_warning(d_, mjWARN_VGEOMFULL, scn_.maxgeom);
			return false;
		}
		mjvGeom* g = scn_.geoms + scn_.ngeom;
		memset(g, 0, sizeof(mjvGeom));
		mjv_initGeom(g, type, size, pos, mat, rgba);
		g->objtype = mjOBJ_UNKNOWN;
		g->objid = -1;
		g->category = mjCAT_DECOR;
		g->segid = scn_.ngeom;
		if (!name.empty()) {
			strncpy_s(g->label, sizeof(g->label), std::string(name).c_str(), _TRUNCATE);
		}
		scn_.ngeom++;
		return true;
	}

	bool sphere(const Position& p, double radius, const float rgba[4], std::string_view name = {}) {
		mjtNum size[3] = { (mjtNum)radius, 0, 0 };
		mjtNum R[9] = {1,0,0, 0,1,0, 0,0,1};
		return geom(mjGEOM_SPHERE, size, &p.x, R, rgba, name);
	}

	bool box(const Position& center, double halfExtent, const float rgba[4], std::string_view name = {}) {
		mjtNum size[3] = { (mjtNum)halfExtent, (mjtNum)halfExtent, (mjtNum)halfExtent };
		mjtNum R[9] = {1,0,0, 0,1,0, 0,0,1};
		return geom(mjGEOM_BOX, size, &center.x, R, rgba, name);
	}

	bool line(const Position& from, const Position& to, const float rgba[4], double width = 0.05) {
		if (scn_.ngeom >= scn_.maxgeom) {
			if (d_) mj_warning(d_, mjWARN_VGEOMFULL, scn_.maxgeom);
			return false;
		}
		mjvGeom* g = scn_.geoms + scn_.ngeom;
		memset(g, 0, sizeof(mjvGeom));
		mjv_initGeom(g, mjGEOM_NONE, nullptr, nullptr, nullptr, rgba);
		g->objtype = mjOBJ_UNKNOWN;
		g->objid = -1;
		g->category = mjCAT_DECOR;
		g->segid = scn_.ngeom;
		mjv_connector(g, mjGEOM_LINE, (mjtNum)width, &from.x, &to.x);
		scn_.ngeom++;
		return true;
	}

	// Helpers
	bool referencePath(const std::vector<Position>& wp) {
		if (wp.size() < 2) return true;
		std::vector<Position> sampled; sampled.reserve(wp.size() * 20);
		catmullRomSplinePoints(wp, sampled, 20);
		static constexpr float kRefRGBA[4] = {0.1f, 0.7f, 1.0f, 1.0f};
		return path(sampled, kRefRGBA);
	}

	bool path(std::vector<Position>& pts, const float rgba[4]) {
		if (pts.size() < 2) return true;
		for (size_t i = 1; i < pts.size(); ++i) {
			if (!line(pts[i-1], pts[i], rgba, 0.02)) return false;
		}
		return true;
	}

	bool waypoints(const std::vector<Position>& pts, double size, const float rgba[4]) {
		for (const auto& p : pts) {
			if (!box(p, size, rgba)) return false;
		}
		return true;
	}

private:
	mjvScene& scn_;
	mjData* d_{};
};

// Renderer wrapper: handles mjv/mjr lifecycle and frame flow.
class MuJoCoRenderer {
public:
	explicit MuJoCoRenderer(MjSim& sim) : sim_(sim) {}

	void init(int maxgeom = 2000) {
		mjv_defaultCamera(&sim_.cam);
		mjv_defaultOption(&sim_.opt);
		mjv_defaultScene(&sim_.scn);
		mjr_defaultContext(&sim_.con);
		mjv_makeScene(sim_.m, &sim_.scn, maxgeom);
		sim_.scn.flags[mjCAT_DECOR] = 1;
		mjr_makeContext(sim_.m, &sim_.con, mjFONTSCALE_150);
	}

	void beginFrame() {
		mjv_updateScene(sim_.m, sim_.d, &sim_.opt, nullptr, &sim_.cam, mjCAT_ALL, &sim_.scn);
	}

	void addModelDecorations() {
		mjv_addGeoms(sim_.m, sim_.d, &sim_.opt, nullptr, mjCAT_DECOR, &sim_.scn);
	}

	void render(const mjrRect& rect) {
		mjr_render(rect, &sim_.scn, &sim_.con);
	}

	mjrContext* context() { return &sim_.con; }
	mjvScene& scene() { return sim_.scn; }

private:
	MjSim& sim_;
};

// UI wrapper: builds and drives mjUI with the same layout across programs.
class MuJoCoUI {
public:
	MuJoCoUI() {
		memset(&ui_, 0, sizeof(ui_));
		ui_.spacing = mjui_themeSpacing(0);
		ui_.color = mjui_themeColor(0);
		ui_.predicate = nullptr;
		ui_.rectid = 1; // rect[1] is UI
		ui_.auxid = 0;
		vis_ee_[0]=vis_ee_[1]=vis_ee_[2]=0.0;
		vis_base_[0]=vis_base_[1]=vis_base_[2]=0.0;
		vis_s_ = 0.0;
	}

	void buildDefaultPanels(MjSim& sim, const mjrContext* con) {
		mjuiDef def[] = {
			{ mjITEM_SECTION, "Display", 0, nullptr, "" },
			{ mjITEM_RADIO,   "Path", 1, &show_refPath_, "Off\nOn" },
			{ mjITEM_SELECT,  "Labels", 1, &label_choice_, "None\nGeom\nSite\nJoint\nBody\nContactPoint" },
			{ mjITEM_SECTION, "Geometry", 1, nullptr, "" },
			{ mjITEM_EDITNUM, "EE", 1, &vis_ee_, "3" },
			{ mjITEM_EDITNUM, "BASE", 1, &vis_base_, "3" },
			{ mjITEM_EDITNUM, "s", 1, &vis_s_, "1" },
			{ mjITEM_EDITINT, "geom", 1, &sim.scn.ngeom, "1" },
			{ mjITEM_END,     "", 0, nullptr, "" }
		};
		mjui_add(&ui_, def);
		if (ui_.nsect > 0) ui_.sect[0].state = mjSECT_OPEN;

		// Joints section
		mjuiDef sectJoints[] = {
			{ mjITEM_SECTION, "Joints", 1, nullptr, "" },
			{ mjITEM_END,     "", 0, nullptr, "" }
		};
		mjui_add(&ui_, sectJoints);

		// Template slider, customized per joint
		mjuiDef defSlider[] = {
			{ mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" },
			{ mjITEM_END,       "", 0, nullptr, "" }
		};

		for (int j = 0; j < sim.m->njnt; ++j) {
			int type = sim.m->jnt_type[j];
			if (type != mjJNT_HINGE && type != mjJNT_SLIDE) continue;
			int qadr = sim.m->jnt_qposadr[j];
			defSlider[0].pdata = const_cast<mjtNum*>(&sim.d->qpos[qadr]);
			const char* jname = mj_id2name(sim.m, mjOBJ_JOINT, j);
			if (jname && jname[0] != '\0') {
				strncpy_s(defSlider[0].name, sizeof(defSlider[0].name), jname, _TRUNCATE);
			} else {
				snprintf(defSlider[0].name, sizeof(defSlider[0].name), "joint %d", j);
			}
			if (sim.m->jnt_limited[j]) {
				double lo = sim.m->jnt_range[2*j + 0];
				double hi = sim.m->jnt_range[2*j + 1];
				snprintf(defSlider[0].other, sizeof(defSlider[0].other), "%.6g %.6g", lo, hi);
			} else if (type == mjJNT_HINGE) {
				strncpy_s(defSlider[0].other, sizeof(defSlider[0].other), "-3.1416 3.1416", _TRUNCATE);
			} else {
				strncpy_s(defSlider[0].other, sizeof(defSlider[0].other), "-1 1", _TRUNCATE);
			}
			mjui_add(&ui_, defSlider);
		}
		mjui_resize(&ui_, con);
	}

	void processEvents(GLFWwindow* window, const mjrContext* con, int fbw, int fbh) {
		static int prev_left = 0;
		double x,y; glfwGetCursorPos(window,&x,&y);
		int w=fbw, h=fbh;
		memset(&state_, 0, sizeof(state_));
		state_.nrect = 3;
		state_.rect[0].left = 0; state_.rect[0].bottom = 0; state_.rect[0].width = w; state_.rect[0].height = h;
		state_.rect[1].left = 0; state_.rect[1].bottom = 0; state_.rect[1].width = ui_.width; state_.rect[1].height = h;
		int remain = w - ui_.width; if(remain<0) remain = 0;
		state_.rect[2].left = ui_.width; state_.rect[2].bottom = 0; state_.rect[2].width = remain; state_.rect[2].height = h;
		state_.x = (int)x;
		state_.y = h - (int)y;
		int left_now = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
		bool in_ui = (state_.x < ui_.width);
		state_.mouserect = in_ui ? ui_.rectid : -1;
		if (left_now && !prev_left) { state_.type = mjEVENT_PRESS; state_.button = mjBUTTON_LEFT; }
		else if (!left_now && prev_left) { state_.type = mjEVENT_RELEASE; state_.button = mjBUTTON_LEFT; }
		else state_.type = mjEVENT_MOVE;
		state_.left = left_now;
		if (in_ui) {
			mjui_event(&ui_, &state_, con);
		}
		prev_left = left_now;
	}

	void perFrameLayout(mjrContext* con, int fbw, int fbh) {
		mjui_resize(&ui_, con);
		int id = ui_.auxid;
		if (con->auxFBO[id] == 0 ||
			con->auxFBO_r[id] == 0 ||
			con->auxColor[id] == 0 ||
			con->auxColor_r[id] == 0 ||
			con->auxWidth[id] != ui_.width ||
			con->auxHeight[id] != ui_.maxheight ||
			con->auxSamples[id] != ui_.spacing.samples) {
			mjr_addAux(id, ui_.width, ui_.maxheight, ui_.spacing.samples, con);
		}
		mjui_update(-1, -1, &ui_, &state_, con);
	}

	void render(mjrContext* con) {
		mjui_render(&ui_, &state_, con);
	}

	mjrRect view3dRect() const { return state_.rect[2]; }
	bool showRefPath() const { return show_refPath_ != 0; }
	int labelChoice() const { return label_choice_; }

	void setEE(const double ee[3]) { vis_ee_[0]=ee[0]; vis_ee_[1]=ee[1]; vis_ee_[2]=ee[2]; }
	void setBase(const double base[3]) { vis_base_[0]=base[0]; vis_base_[1]=base[1]; vis_base_[2]=base[2]; }
	void setS(double s) { vis_s_ = s; }

private:
	mjUI ui_{};
	mjuiState state_{};
	int show_refPath_ = 1;
	int label_choice_ = 1;
	double vis_ee_[3];
	double vis_base_[3];
	double vis_s_{};
};

struct PathPoint
{
	double rate;
	Position point;
};

class Path{
public:
	std::vector<PathPoint> points;
};


// ---------------------- End refactor section ----------------------

void create3rdSpline(
	const Positions& wp,
	std::vector<Position>& points)
{
	catmullRomSplinePoints(wp, points);
}

// Fit a spline to Path points (positions) using Eigen::SplineFitting.
// Returns: 0 success, 1 = not enough points, 2 = invalid rate sequence.
inline int makePathSpline(const Path& path, Spline3d& spline, int degree = 3) {
    const size_t N = path.points.size();
    if (N < 2) {
        return 1; // not enough points
    }
    // Validate rates strictly monotonic non-decreasing within [0,1]
    double prevRate = -1.0;
    for (size_t i = 0; i < N; ++i) {
        double r = path.points[i].rate;
        if (!(r >= 0.0 && r <= 1.0) || r < prevRate) {
            return 2; // invalid rate sequence
        }
        prevRate = r;
    }
    // Build matrix of points (3 x N)
    Eigen::Matrix<double, 3, Eigen::Dynamic> pts(3, static_cast<int>(N));
    Eigen::RowVectorXd u(static_cast<int>(N));
    for (size_t i = 0; i < N; ++i) {
        const auto& P = path.points[i].point;
        pts(0, static_cast<int>(i)) = P.x;
        pts(1, static_cast<int>(i)) = P.y;
        pts(2, static_cast<int>(i)) = P.z;
        u(static_cast<int>(i)) = path.points[i].rate; // assume valid
    }
    int maxDegree = static_cast<int>(N) - 1;
    if (degree > maxDegree) degree = maxDegree;
    if (degree < 1) degree = 1;
    spline = Eigen::SplineFitting<Spline3d>::Interpolate(pts, degree, u);
    return 0;
}

// Sample spline at normalized parameter t in [0,1].
inline Position sampleSpline(const Spline3d& spline, double t) {
    double clamped = std::min(1.0, std::max(0.0, t));
    Eigen::Matrix<double,3,1> v = spline(clamped);
    return Position{v(0), v(1), v(2)};
}

void generatePath(const Positions& wp, Path& path)
{
	if(path.points.size() > 0) 
		path.points.clear();

	std::vector<Position> points;
	create3rdSpline(wp, points);

	double totalNorm = 0.0;
	Position prev = points[0];
	for (const auto& p : points) {
		totalNorm += (p - prev).norm2();
		prev = p;
	}

	prev = points[0];
	double rate = 0.0;
	for (const auto& p : points) {
		rate += ((p - prev).norm2() / totalNorm);
		path.points.push_back({rate, p});
		prev = p;
	}
	path.points.back().rate = 1.0; // for compensation.
}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
	if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
		mj_resetData(mj.m, mj.d);
		mj_forward(mj.m, mj.d);
	}
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
	mj.button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	mj.button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	mj.button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
	glfwGetCursorPos(window, &mj.lastx, &mj.lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
	if (!mj.button_left && !mj.button_middle && !mj.button_right) return;
	double dx = xpos - mj.lastx;
	double dy = ypos - mj.lasty;
	mj.lastx = xpos;
	mj.lasty = ypos;
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
										glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
	mjtMouse action;
	if (mj.button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (mj.button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else action = mjMOUSE_ZOOM;
	mjv_moveCamera(mj.m, action, dx/height, dy/height, &mj.scn, &mj.cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
	mjv_moveCamera(mj.m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mj.scn, &mj.cam);
}

class SimplePathReader{
public:
	SimplePathReader(const Path& path, int T)
	: path_(path)
	, t_(0)
	, splineValid_(false)
	, T_(T)	
	{
		if (makePathSpline(path_, spline_) == 0) {
			splineValid_ = true;
		}
	}

	Position update()
	{
		if (t_ >= T_) {
			return sampleSpline(spline_, 1.0);
		}
		return sampleSpline(spline_, static_cast<double>(t_++) / T_);
	}
private:
	Path path_;
	int t_;
	Spline3d spline_;
	bool splineValid_;
	int T_;
};

class Pose
{
public:
	Pose() = default;
	Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
		: pos(position), quat(orientation) {}
	Eigen::Vector3d pos;
	Eigen::Quaterniond quat;
	void show(){
		std::cout << pos << std::endl << quat << std::endl;
	}
};

void getSitePose(const MjSim& mj, const char* siteName, Pose& pose)
{
    int sid = mj_name2id(mj.m, mjOBJ_SITE, siteName);
    if (sid < 0) throw std::runtime_error("site not found");
    mj_forward(mj.m, mj.d);
    double* p = mj.d->site_xpos + 3*sid;
    double* R = mj.d->site_xmat + 9*sid; // 3x3 行列（row-major）
    double q[4];
    mju_mat2Quat(q, R); // q: {w, x, y, z}
    Eigen::Vector3d pos(p[0], p[1], p[2]);
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]); // Eigen( w, x, y, z )
	pose = Pose(pos, quat);
}

VectorXd qpos(const MjSim& mj)
{
    VectorXd q(mj.m->nq);
    for (int i = 0; i < mj.m->nq; ++i) 
		q[i] = mj.d->qpos[i];
    return q;
}

void getEEPose(const MjSim& mj, Pose& pose) 
{
	getSitePose(mj, "site_gripper", pose);
}
void getBasePose(const MjSim& mj, Pose& pose) 
{
	getSitePose(mj, "site_base", pose);
}

int createPath(MjSim& mj, WayPoints& points)
{
	auto constraintPathPlanner = std::make_shared<ConstraintPathPlanner>(mj);
	PathPlanningInput input;
	input.start = vectorToArray(qpos(mj));
	input.goal = vectorToArray(qpos(mj));


	WayPoint w1 = input.start;
	w1[0] = 5;
	input.goal = w1;
	input.goal[1] = 5;

//	input.goal[1] = 20;
//	input.goal[2] = 1.72; 
// temprary disabled.
//	int result = constraintPathPlanner->plan(input, points);
//	if (result != EXIT_SUCCESS) {
//		mju_error("Path planning failed.");
//	}

// temprary added.
	points.push_back(input.start);
	points.push_back(w1);
	points.push_back(input.goal);
	int result = EXIT_SUCCESS;
// temprary added end.
	return result;
}

void updateJointPosition(MjSim& mj, const std::string& jointName, double pos)
{
	int jointId = mj_name2id(mj.m, mjOBJ_JOINT, jointName.c_str());
	if (jointId < 0) {
		mju_error("Joint not found: %s", jointName.c_str());
		return;
	}
	mj.d->ctrl[jointId] = pos;
}

class Kinematics {
public:
	Kinematics(const std::string& modelPath)
	: mj_(modelPath)
	{
	}
	bool fk(
		const std::array<double, DOF> q, 
		Position& p, 
		std::string sName = "site_base") // "site_gripper"
	{
		for (int i = 0; i < mj_.m->nq; ++i) 
			mj_.d->qpos[i] = q[i];
		// optionally copy velocities if needed: 
		// memcpy(d_tmp->qvel, d_main->qvel, sizeof(mjtNum)*m->nv);
		mj_forward(mj_.m, mj_.d);

		int sid = mj_name2id(mj_.m, mjOBJ_SITE, sName.c_str());
		if (sid < 0) 
			return false;
		double* pos = mj_.d->site_xpos + 3*sid;
		p = Position{pos[0], pos[1], pos[2]};
		return true;
	}
private:
	MjSim mj_;
};

class ContinuousPath
{
public:
	ContinuousPath(): waypoints_(){}
	void generate(const WayPoints& waypoints)
	{
		waypoints_ = waypoints;
		n_ = static_cast<int>(waypoints_.size());
		if (n_ <= 1) return; // nothing to build

		// parameter spacing: s in [0,1], equally spaced
		h_ = 1.0 / static_cast<double>(std::max(1, n_ - 1));

		// allocate M (second derivatives) per dof
		M_.assign(DOF, std::vector<double>(n_, 0.0));

		// For each DOF, build natural cubic spline second derivatives
		for (int dim = 0; dim < DOF; ++dim) {
			// collect y values
			std::vector<double> y(n_);
			for (int i = 0; i < n_; ++i) y[i] = waypoints_[i][dim];
			computeNaturalSecondDerivatives(y, M_[dim]);
		}
	}

	// evaluate q(s) for s in [0,1]
	std::array<double, DOF> q(double s)
	{
		std::array<double, DOF> out{};
		if (n_ == 0) return out;
		if (n_ == 1) return waypoints_[0];

		double sc = std::min(1.0, std::max(0.0, s));
		// find segment index
		int idx;
		if (sc >= 1.0) {
			idx = n_ - 2;
		} else {
			idx = static_cast<int>(sc / h_);
			if (idx < 0) idx = 0;
			if (idx > n_ - 2) idx = n_ - 2;
		}
		double si = idx * h_;
		double si1 = si + h_;

		for (int dim = 0; dim < DOF; ++dim) {
			const double* M = M_[dim].data();
			double yi = waypoints_[idx][dim];
			double yi1 = waypoints_[idx+1][dim];
			double xi = si;
			double xi1 = si1;
			double x = sc;
			double A = (xi1 - x) / h_;
			double B = (x - xi) / h_;
			// cubic spline formula using normalized barycentric coords A,B
			double val = M[idx] * (A*A*A) * (h_ * h_) / 6.0
					   + M[idx+1] * (B*B*B) * (h_ * h_) / 6.0
					   + (yi - M[idx] * h_ * h_ / 6.0) * A
					   + (yi1 - M[idx+1] * h_ * h_ / 6.0) * B;
			out[dim] = val;
		}
		return out;
	}

	// derivative dq/ds
	std::array<double, DOF> dq(double s)
	{
		std::array<double, DOF> out{};
		if (n_ == 0) return out;
		if (n_ == 1) return out; // zero derivative

		double sc = std::min(1.0, std::max(0.0, s));
		int idx;
		if (sc >= 1.0) {
			idx = n_ - 2;
		} else {
			idx = static_cast<int>(sc / h_);
			if (idx < 0) idx = 0;
			if (idx > n_ - 2) idx = n_ - 2;
		}
		double si = idx * h_;
		double si1 = si + h_;
		double x = sc;

		for (int dim = 0; dim < DOF; ++dim) {
			const std::vector<double>& Mv = M_[dim];
			double yi = waypoints_[idx][dim];
			double yi1 = waypoints_[idx+1][dim];
			// derivative formula
			double term1 = - Mv[idx] * (si1 - x) * (si1 - x) / (2.0 * h_);
			double term2 =   Mv[idx+1] * (x - si) * (x - si) / (2.0 * h_);
			double term3 = (yi1 - yi) / h_ - (h_ / 6.0) * (Mv[idx+1] - Mv[idx]);
			out[dim] = term1 + term2 + term3;
		}
		return out;
	}

private:
	WayPoints waypoints_;
	int n_ = 0;
	double h_ = 0.0; // uniform spacing
	// M_[dim][i] second derivative at knot i for each dimension
	std::vector<std::vector<double>> M_;

	// compute natural cubic spline second derivatives (M) for scalar y values
	void computeNaturalSecondDerivatives(const std::vector<double>& y, std::vector<double>& M_out)
	{
		int n = static_cast<int>(y.size());
		M_out.assign(n, 0.0);
		if (n <= 1) return;

		// build tridiagonal system: lower, diag, upper, rhs
		std::vector<double> lower(n, 0.0), diag(n, 0.0), upper(n, 0.0), rhs(n, 0.0);
		// natural spline boundary
		diag[0] = 1.0; rhs[0] = 0.0;
		diag[n-1] = 1.0; rhs[n-1] = 0.0;

		// interior equations
		for (int i = 1; i < n-1; ++i) {
			lower[i] = h_;
			diag[i]  = 2.0 * (h_ + h_);
			upper[i] = h_;
			rhs[i] = 6.0 * ( (y[i+1] - y[i]) / h_ - (y[i] - y[i-1]) / h_ );
		}

		// Thomas algorithm
		// forward
		for (int i = 1; i < n; ++i) {
			if (diag[i-1] == 0.0) continue;
			double w = lower[i] / diag[i-1];
			diag[i] -= w * upper[i-1];
			rhs[i]  -= w * rhs[i-1];
		}
		// back substitution
		if (diag[n-1] == 0.0) return;
		M_out[n-1] = rhs[n-1] / diag[n-1];
		for (int i = n-2; i >= 0; --i) {
			double up = (i < n-1) ? upper[i] : 0.0;
			double d = diag[i];
			double r = rhs[i];
			double next = (i+1 < n) ? M_out[i+1] : 0.0;
			if (d == 0.0) M_out[i] = 0.0; else M_out[i] = (r - up * next) / d;
		}
	}
};

int main(int argc, const char** argv) 
{
	printf("kita0\n");	
	// kinematics
	std::shared_ptr<Kinematics> kin = std::make_shared<Kinematics>(model_path);

	//  temp
	Pose pose;
	getEEPose(mj, pose);
	pose.show();

	// init GLFW
	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
	}

	printf("kita1\n");	
	// create window, make OpenGL context current, request v-sync
	GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simple Sim", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// Renderer & UI setup
	MuJoCoRenderer renderer(mj);
	renderer.init(2000);
	std::shared_ptr<MuJoCoUI> simui = std::make_shared<MuJoCoUI>();
	simui->buildDefaultPanels(mj, renderer.context());

	// カメラ初期化: モデル中心とスケールに基づき俯瞰
	mj_forward(mj.m, mj.d);
	for(int i=0;i<3;i++) mj.cam.lookat[i] = mj.m->stat.center[i];
	mj.cam.lookat[0] += 3.0; // x +1 
	mj.cam.distance = 5.0 * mj.m->stat.extent;
	mj.cam.elevation = -60.0;
	mj.cam.azimuth = 75.0;

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	printf("Timestep: %f seconds\n", mj.m->opt.timestep);

//	// Positions Definition
//	Positions blueSph_wp = {
//		{0.0, 0.0, 0.0},
//		{1.0, 0.0, 0.0},
//		{1.0, 0.5, 0.2},
//		{1.0, 1.5, 1.2},
//		{1.0, 1.5, 2.2}
//	};
	Positions refPath;

	// Create a SimplePathReader instance
	// Path blueSphPath;
	// generatePath(blueSph_wp, blueSphPath);
	// int T = 200;  // Define the duration of the path
	// SimplePathReader blueSphPathReader(blueSphPath, T);

	std::vector<Position> movedBasePoints;
	movedBasePoints.push_back({0.0, 0.0, 0.0});

	std::vector<Position> movedEEPoints;
	movedEEPoints.push_back({0.0, 0.0, 0.0});


	Path blueSphMovedPath, redSphMovedPath, greenSphMovedPath;
	// blueSphMovedPath.points.push_back({0.0, blueSphPathReader.update()});
	redSphMovedPath.points.push_back({0.0, {0.0, 0.0, 0.0}});
	greenSphMovedPath.points.push_back({0.0, {0.0, 0.0, 0.0}});

	// redSph
	SecondOrderDynamics redSph(0.1, 0.25, 1.0, {0.0, 0.0, 0.0});
	SecondOrderDynamics greenSph(0.1, 0.25, 2.0, {0.0, 0.0, 0.0});

	// reference path
	ContinuousPath contPath;
	Positions ps; // in Cartesian space
	double s = 0; // rate of path

	static int count = 0;
	// run main loop, target real-time simulation and 60 fps rendering
	while (!glfwWindowShouldClose(window)) {
		double simstart = mj.d->time;
		while (mj.d->time - simstart < 1.0/60.0) {
			mj_step(mj.m, mj.d);

			// createPath
			if(2 == count){
				WayPoints qs; // in Configuration space
				int ret = createPath(mj, qs);
				if (ret != EXIT_SUCCESS) {
					mju_error("Path planning failed.");
				}
				contPath.generate(qs);
				// Path blueSphPath;
				// generatePath(qs, blueSphPath);
				// blueSphMovedPath.points.push_back({0.0, blueSphPathReader.update()});

				// fk: qs->ps
				for (const auto& q : qs) {
					Position p;
					kin->fk(q, p);
					ps.push_back(p);
				}
			} 

			// update Joint pos for test
			if(2 < count){
				auto updateS = [&](){
					s += 0.001;
					s = std::clamp(s, 0.0, 1.0);
				};
				updateS();
				auto q = contPath.q(s);
				//int jointId = mj_name2id(mj.m, mjOBJ_JOINT, jname.c_str());
				//if (jointId < 0) {
				//	mju_error("Joint not found: %s", jname.c_str());
				//}
				//double pos = qpos(mj)[jointId] + 0.1;
				std::string joint_base_x = "joint_base_x";
				std::string joint_base_y = "joint_base_y";
				std::string joint_base_yaw = "joint_base_yaw";
				updateJointPosition(mj, joint_base_x, q[0]);
				updateJointPosition(mj, joint_base_y, q[1]);
				updateJointPosition(mj, joint_base_yaw, q[2]);
			}

			count++;
		}

		mjrRect viewport_full = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &viewport_full.width, &viewport_full.height);
	simui->processEvents(window, renderer.context(), viewport_full.width, viewport_full.height);
	simui->perFrameLayout(renderer.context(), viewport_full.width, viewport_full.height);
	// Update the scene first (this resets scn.ngeom)
	renderer.beginFrame();

		// Update live EE position variables used by the UI (site name: "site_gripper")
		{
			Pose eePose; getEEPose(mj, eePose);
			double tmp[3] = {eePose.pos[0], eePose.pos[1], eePose.pos[2]};
			simui->setEE(tmp);
		}
		{
			Pose basePose; getBasePose(mj, basePose);
			double tmp[3] = {basePose.pos[0], basePose.pos[1], basePose.pos[2]};
			simui->setBase(tmp);
		}
		simui->setS(s);

		// double dt = mj.d->time - simstart;
		int ec = EXIT_SUCCESS;
		GeometryPrims prims(renderer.scene(), mj.d);
		if (simui->showRefPath()) {
			if (!prims.referencePath(ps)) return EXIT_FAILURE;
		}

		// draw spheres and the moved path
//		// blue sphere
//		Position pos_ref = blueSphPathReader.update();
//		ec = drawSph(mj, pos_ref, CLR_BLUE, "ref");
//		if (ec != EXIT_SUCCESS) return ec;
//		// red sphere
//		Position pos_redSph = redSph.update(pos_ref, dt);
//		ec = drawSph(mj, pos_redSph, CLR_RED, "p1");
//		if (ec != EXIT_SUCCESS) return ec;
//		// green sphere
//		Position pos_greenSph = greenSph.update(pos_ref, dt);
//		ec = drawSph(mj, pos_greenSph, CLR_GREEN, "p2");
//		if (ec != EXIT_SUCCESS) return ec;
		

		auto drawMovedPath = [&prims](
			const Position& newPos,
			std::vector<Position>& points,
			const float rgba[4]) -> int {
				Position prev = points.back();
				if((prev - newPos).norm2()>0.0001) {
					points.push_back(newPos);
				}
				return prims.path(points, rgba) ? EXIT_SUCCESS : EXIT_FAILURE;
			};
		Pose basePose;
		getBasePose(mj, basePose);
		ec = drawMovedPath(basePose.pos, movedBasePoints, CLR_YELLOW);
		if (ec != EXIT_SUCCESS) return ec;
		Pose EEPose;
		getEEPose(mj, EEPose);
		ec = drawMovedPath(EEPose.pos, movedEEPoints, CLR_GREEN);
		if (ec != EXIT_SUCCESS) return ec;

//		ec = drawMovedPath(mj, redSphMovedPath, pos_redSph, CLR_RED);
//		if (ec != EXIT_SUCCESS) return ec;
//		ec = drawMovedPath(mj, greenSphMovedPath, pos_greenSph, CLR_GREEN);
//		if (ec != EXIT_SUCCESS) return ec;

//		printf("dt: %f, ngeom: %d\n", dt, mj.scn.ngeom);

		// Map UI selection to mj.opt.label
	switch (simui->labelChoice()) {
			case 0: mj.opt.label = mjLABEL_NONE; break;
			case 1: mj.opt.label = mjLABEL_GEOM; break; // Geom
			case 2: mj.opt.label = mjLABEL_SITE; break; // Site
			case 3: mj.opt.label = mjLABEL_JOINT; break; // Joint
			case 4: mj.opt.label = mjLABEL_BODY; break; // Body
			case 5: mj.opt.label = mjLABEL_CONTACTPOINT; break; // Contact point
			default: mj.opt.label = mjLABEL_GEOM; break;
		}

	renderer.addModelDecorations();

	// 3D表示領域 (rect[2]) へ描画
	mjrRect view3d = simui->view3dRect();
	renderer.render(view3d);
	// UIを最後に描画
	simui->render(renderer.context());
		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	//free visualization storage
	mjv_freeScene(&mj.scn);
	mjr_freeContext(&mj.con);
	mj_deleteData(mj.d);
	mj_deleteModel(mj.m);
#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
#endif
	return EXIT_SUCCESS;
}
