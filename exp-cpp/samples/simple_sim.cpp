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


// UI state for spline toggle
static int show_refPath = 1; // 1=ON, 0=OFF
static mjUI ui0;            // single UI panel
static mjuiState uistate;   // state
// live display of end-effector position in the UI (bound to mjui edit items)
static double vis_ee[3] = {0.0, 0.0, 0.0};
static double vis_base[3] = {0.0, 0.0, 0.0};
static double vis_s = 0.0;
// UI selection for which labels to show (pulldown)
static int label_choice = 1; // default: 1 -> Geom

// Build minimal UI with a radio button group to toggle spline visibility
static void build_ui(const mjrContext* con) {
	mjuiDef def[] = {
		{ mjITEM_SECTION, "Display", 0, nullptr, "" },
		{ mjITEM_RADIO,   "Path", 1, &show_refPath, "Off\nOn" },
		{ mjITEM_SELECT,  "Labels", 1, &label_choice, "None\nGeom\nSite\nJoint\nBody\nContactPoint" },
		{ mjITEM_SECTION, "Geometry", 1, nullptr, "" },
		{ mjITEM_EDITNUM, "EE", 1, &vis_ee, "3" },
		{ mjITEM_EDITNUM, "BASE", 1, &vis_base, "3" },
		{ mjITEM_EDITNUM, "s", 1, &vis_s, "1" },
		{ mjITEM_EDITINT, "geom", 1, &mj.scn.ngeom, "1" },
		{ mjITEM_END,     "", 0, nullptr, "" }
	};
	mjui_add(&ui0, def);
	if (ui0.nsect > 0) ui0.sect[0].state = mjSECT_OPEN; // セクション展開
	ui0.spacing = mjui_themeSpacing(0);
	ui0.color   = mjui_themeColor(0);

	// Add Joint sliders section and one slider per hinge/slide joint
	mjuiDef sectJoints[] = {
		{ mjITEM_SECTION, "Joints", 1, nullptr, "" },
		{ mjITEM_END,     "", 0, nullptr, "" }
	};
	mjui_add(&ui0, sectJoints);

	// template slider, will be customized per joint
	mjuiDef defSlider[] = {
		{ mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" },
		{ mjITEM_END,       "", 0, nullptr, "" }
	};

	for (int j = 0; j < mj.m->njnt; ++j) {
		int type = mj.m->jnt_type[j];
		if (type != mjJNT_HINGE && type != mjJNT_SLIDE) continue; // only scalar joints

		// bind directly to qpos for this joint
		int qadr = mj.m->jnt_qposadr[j];
		defSlider[0].pdata = &mj.d->qpos[qadr];

		// name
		const char* jname = mj_id2name(mj.m, mjOBJ_JOINT, j);
		if (jname && jname[0] != '\0') {
			strncpy_s(defSlider[0].name, sizeof(defSlider[0].name), jname, _TRUNCATE);
		} else {
			snprintf(defSlider[0].name, sizeof(defSlider[0].name), "joint %d", j);
		}

		// set range: use model limits if available, otherwise sensible defaults
		if (mj.m->jnt_limited[j]) {
			double lo = mj.m->jnt_range[2*j + 0];
			double hi = mj.m->jnt_range[2*j + 1];
			snprintf(defSlider[0].other, sizeof(defSlider[0].other), "%.6g %.6g", lo, hi);
		} else if (type == mjJNT_HINGE) {
			strncpy_s(defSlider[0].other, sizeof(defSlider[0].other), "-3.1416 3.1416", _TRUNCATE);
		} else { // slider
			strncpy_s(defSlider[0].other, sizeof(defSlider[0].other), "-1 1", _TRUNCATE);
		}

		mjui_add(&ui0, defSlider);
	}
	mjui_resize(&ui0, con);
}

// Simple mouse->ui event helper (minimal subset)
static void process_ui_events(GLFWwindow* window, const mjrContext* con, int fbw, int fbh) {
	static int prev_left = 0;
	double x,y; glfwGetCursorPos(window,&x,&y);
	int w=fbw, h=fbh; // framebufferサイズ使用
	memset(&uistate, 0, sizeof(uistate));
	// rect[0]=全体, rect[1]=UI, rect[2]=3D表示領域
	uistate.nrect = 3;
	uistate.rect[0].left = 0; uistate.rect[0].bottom = 0; uistate.rect[0].width = w; uistate.rect[0].height = h;
	uistate.rect[1].left = 0; uistate.rect[1].bottom = 0; uistate.rect[1].width = ui0.width; uistate.rect[1].height = h;
	int remain = w - ui0.width; if(remain<0) remain = 0;
	uistate.rect[2].left = ui0.width; uistate.rect[2].bottom = 0; uistate.rect[2].width = remain; uistate.rect[2].height = h;
	uistate.x = (int)x;
	uistate.y = h - (int)y;
	int left_now = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	int uiwidth = ui0.width;
	bool in_ui = (uistate.x < uiwidth);
	// UIはrectid=1を使用
	uistate.mouserect = in_ui ? ui0.rectid : -1;
	if (left_now && !prev_left) { uistate.type = mjEVENT_PRESS; uistate.button = mjBUTTON_LEFT; }
	else if (!left_now && prev_left) { uistate.type = mjEVENT_RELEASE; uistate.button = mjBUTTON_LEFT; }
	else uistate.type = mjEVENT_MOVE;
	uistate.left = left_now;
	if (in_ui) {
		mjui_event(&ui0, &uistate, con); // propagate all UI events including MOVE
	}
	prev_left = left_now;
}

// Minimal per-frame UI maintenance (resize + update) similar to simulate's UiModify
// simple layout: rect[0]=full window, rect[1]=UI, rect[2]=3D viewport
static void ui_per_frame(mjrContext* con, int fbw, int fbh);

// ui_per_frame 定義（毎フレーム UI サイズと補助FBO確認）
static void ui_per_frame(mjrContext* con, int fbw, int fbh) {
	mjui_resize(&ui0, con);
	int id = ui0.auxid;
	if (con->auxFBO[id] == 0 ||
		con->auxFBO_r[id] == 0 ||
		con->auxColor[id] == 0 ||
		con->auxColor_r[id] == 0 ||
		con->auxWidth[id] != ui0.width ||
		con->auxHeight[id] != ui0.maxheight ||
		con->auxSamples[id] != ui0.spacing.samples) {
		mjr_addAux(id, ui0.width, ui0.maxheight, ui0.spacing.samples, con);
	}
	// UIレイアウト（rect[1]使用）
	mjui_update(-1, -1, &ui0, &uistate, con);
}

struct PathPoint
{
	double rate;
	Position point;
};

class Path{
public:
	std::vector<PathPoint> points;
};

void catmullRomSplinePoints(const Positions& wp, std::vector<Position>& out_points, int num_steps = 20) {
	if (wp.size() < 2) return;
	for (size_t i = 0; i + 1 < wp.size(); ++i) {
		const Position& p0 = (i == 0) ? wp[0] : wp[i-1];
		const Position& p1 = wp[i];
		const Position& p2 = wp[i+1];
		const Position& p3 = (i+2 < wp.size()) ? wp[i+2] : wp[wp.size()-1];
		for (int j = 1; j <= num_steps; ++j) {
			double t = (double)j / num_steps;
			Position pt;
			pt.x = 0.5 * ((2.0 * p1.x) +
				(-p0.x + p2.x) * t +
				(2.0*p0.x - 5.0*p1.x + 4.0*p2.x - p3.x) * t * t +
				(-p0.x + 3.0*p1.x - 3.0*p2.x + p3.x) * t * t * t);
			pt.y = 0.5 * ((2.0 * p1.y) +
				(-p0.y + p2.y) * t +
				(2.0*p0.y - 5.0*p1.y + 4.0*p2.y - p3.y) * t * t +
				(-p0.y + 3.0*p1.y - 3.0*p2.y + p3.y) * t * t * t);
			pt.z = 0.5 * ((2.0 * p1.z) +
				(-p0.z + p2.z) * t +
				(2.0*p0.z - 5.0*p1.z + 4.0*p2.z - p3.z) * t * t +
				(-p0.z + 3.0*p1.z - 3.0*p2.z + p3.z) * t * t * t);
			out_points.push_back(pt);
		}
	}
}

int drawGeom(
	enum mjtGeom_ geom, 
	MjSim& mj,
	const Position& pt,
	const mjtNum size[3],
	const mjtNum pos[3], 
	const mjtNum mat[9], 
	const float rgba[4],
	std::string name)
{
	if ( mj.scn.ngeom>=mj.scn.maxgeom ) {
		mj_warning(mj.d, mjWARN_VGEOMFULL, mj.scn.maxgeom);
		return EXIT_FAILURE;
	}
	mjvScene& scn(mj.scn);
    mjvGeom *g = scn.geoms + scn.ngeom;
	memset(g, 0, sizeof(mjvGeom));

    // Add it to the scene
	mjv_initGeom(g, geom, size, pos, mat, rgba);
    g->objtype = mjOBJ_UNKNOWN;
    g->objid = -1;
    g->category = mjCAT_DECOR;
    g->segid = scn.ngeom;
	strncpy_s(g->label, sizeof(g->label), name.c_str(), _TRUNCATE);

	scn.ngeom++;
	return EXIT_SUCCESS;	
}

int drawSph(
	MjSim& mj,
	const Position& pt,
	const float rgba[4],
	std::string name)
{ 
	mjtNum sphsize[3] = {RADIUS_SPH, 0, 0};
    mjtNum myrot3x3[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
	return drawGeom(mjGEOM_SPHERE, mj, pt, sphsize, &pt.x, myrot3x3, rgba, name);
}

int drawBox(
	MjSim& mj,
	const Position& pt,
	double radius,
	const float rgba[4],
	std::string name)
{ 
	mjtNum boxsize[3] = {radius, radius, radius};
    mjtNum myrot3x3[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
	return drawGeom(mjGEOM_BOX, mj, pt, boxsize, &pt.x, myrot3x3, rgba, name);
}

int drawLine(
	MjSim& mj,
	const Position& from,
	const Position& to,
	const float rgba[4])
{
	if ( mj.scn.ngeom>=mj.scn.maxgeom ) {
		mj_warning(mj.d, mjWARN_VGEOMFULL, mj.scn.maxgeom);
		return EXIT_FAILURE;
	}
	mjvGeom* g = mj.scn.geoms + mj.scn.ngeom;
	memset(g, 0, sizeof(mjvGeom));
	mjv_initGeom(g, mjGEOM_NONE, NULL, NULL, NULL, rgba);
	g->objtype = mjOBJ_UNKNOWN;
	g->objid = -1;
	g->category = mjCAT_DECOR;
	g->segid = mj.scn.ngeom;
	mjv_connector(g, mjGEOM_LINE, 0.05, &from.x, &to.x);
	mj.scn.ngeom++;
	return EXIT_SUCCESS;
}

int drawReferencePath(
	MjSim& mj,
	const Positions& wp)
{
	if (wp.size() < 2) return EXIT_SUCCESS;

	// Draw Positions
	double boxSize = 0.05;
	for (const auto& pt : wp) {
		if (drawBox(mj, pt, boxSize, CLR_GREEN, "") != EXIT_SUCCESS)
			return EXIT_FAILURE;
	}

	// Draw Catmull-Rom spline
	std::vector<Position> points;
	catmullRomSplinePoints(wp, points);
	if (points.empty()) return EXIT_SUCCESS;
	Position prev = points[0];

	for (const auto& pt : points) {
		if (drawLine(mj, prev, pt, CLR_BLUE) != EXIT_SUCCESS) 
			return EXIT_FAILURE;
		prev = pt;
	}
	return EXIT_SUCCESS;
}

int drawWayPoint(
	MjSim& mj,
	const Positions& points)
{
	double radius = 0.02;
	for (const auto& pt : points) {
		if (drawBox(mj, pt, radius, CLR_PURPLE, "") != EXIT_SUCCESS)
			return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int drawPath(
	MjSim& mj,
	std::vector<Position>& points,
	const float rgba[4])
{
	Position prev = points[0];
	for (const auto& pt : points) {
		if (drawLine(mj, prev, pt, rgba) != EXIT_SUCCESS) 
			return EXIT_FAILURE;
		prev = pt;
	}
	return EXIT_SUCCESS;
}

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

	// create window, make OpenGL context current, request v-sync
	GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simple Sim", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&mj.cam);
	mjv_defaultOption(&mj.opt);
	mjv_defaultScene(&mj.scn);
	mjr_defaultContext(&mj.con);

	// init simple UI
	memset(&ui0,0,sizeof(ui0));
	ui0.spacing = mjui_themeSpacing(0);
	ui0.color = mjui_themeColor(0);
	ui0.predicate = NULL;
	ui0.rectid = 1; // rect[1] をUI用に確保
	ui0.auxid = 0;
	// make OpenGL context (later) before building UI so we have font metrics

	// create scene and context
	mjv_makeScene(mj.m, &mj.scn, 2000);
	mj.scn.flags[mjCAT_DECOR] = 1;
	mjr_makeContext(mj.m, &mj.con, mjFONTSCALE_150);
	build_ui(&mj.con);

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
		process_ui_events(window, &mj.con, viewport_full.width, viewport_full.height);
		ui_per_frame(&mj.con, viewport_full.width, viewport_full.height);
		// Update the scene first (this resets scn.ngeom)
		mjv_updateScene(mj.m, mj.d, &mj.opt, NULL, &mj.cam, mjCAT_ALL, &mj.scn);

		// Update live EE position variables used by the UI (site name: "site_gripper")
		{
			Pose eePose;
			getEEPose(mj, eePose);
			vis_ee[0] = eePose.pos[0];
			vis_ee[1] = eePose.pos[1];
			vis_ee[2] = eePose.pos[2];
		}
		{
			Pose basePose;
			getBasePose(mj, basePose);
			vis_base[0] = basePose.pos[0];
			vis_base[1] = basePose.pos[1];
			vis_base[2] = basePose.pos[2];
		}
		{
			vis_s = s;
		}

		// double dt = mj.d->time - simstart;
		int ec = EXIT_SUCCESS;
		if (show_refPath) {
			ec = drawReferencePath(mj, ps);
			if (ec != EXIT_SUCCESS) return ec;
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
		

		auto drawMovedPath = [](
			MjSim& mj,
			const Position& newPos,
			std::vector<Position>& points, 
			const float rgba[4]) -> int {
				Position prev = points.back();
				if((prev - newPos).norm2()>0.0001) {
					points.push_back(newPos);
				}
				return drawPath(mj, points, rgba);
			};
		Pose basePose;
		getBasePose(mj, basePose);
		ec = drawMovedPath(mj, basePose.pos, movedBasePoints, CLR_YELLOW);
		if (ec != EXIT_SUCCESS) return ec;
		Pose EEPose;
		getEEPose(mj, EEPose);
		ec = drawMovedPath(mj, EEPose.pos, movedEEPoints, CLR_GREEN);
		if (ec != EXIT_SUCCESS) return ec;

//		ec = drawMovedPath(mj, redSphMovedPath, pos_redSph, CLR_RED);
//		if (ec != EXIT_SUCCESS) return ec;
//		ec = drawMovedPath(mj, greenSphMovedPath, pos_greenSph, CLR_GREEN);
//		if (ec != EXIT_SUCCESS) return ec;

//		printf("dt: %f, ngeom: %d\n", dt, mj.scn.ngeom);

		// Map UI selection to mj.opt.label
		switch (label_choice) {
			case 0: mj.opt.label = mjLABEL_NONE; break;
			case 1: mj.opt.label = mjLABEL_GEOM; break; // Geom
			case 2: mj.opt.label = mjLABEL_SITE; break; // Site
			case 3: mj.opt.label = mjLABEL_JOINT; break; // Joint
			case 4: mj.opt.label = mjLABEL_BODY; break; // Body
			case 5: mj.opt.label = mjLABEL_CONTACTPOINT; break; // Contact point
			default: mj.opt.label = mjLABEL_GEOM; break;
		}

		mjv_addGeoms(mj.m, mj.d, &mj.opt, NULL, mjCAT_DECOR, &mj.scn);

		// 3D表示領域 (rect[2]) へ描画
		mjrRect view3d = uistate.rect[2];
		mjr_render(view3d, &mj.scn, &mj.con);
		// UIを最後に描画
		mjui_render(&ui0, &uistate, &mj.con);
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
