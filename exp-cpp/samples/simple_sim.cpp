// Simple MuJoCo simulation sample: visualize panda_arm_mjcf.xml
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <array>
#include <initializer_list>
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
// Embedding Python
#include <Python.h>

#include "./constraint_path_planner.hpp"

// Spline alias (3D)
using Spline3d = Eigen::Spline<double, 3>;

const float CLR_RED[4] = {1.f, 0.f, 0.f, 1.f};
const float CLR_GREEN[4] = {0.f, 1.f, 0.f, 1.f};
const float CLR_BLUE[4] = {0.f, 0.f, 1.f, 1.f};
//const float CLR_PURPLE[4] = {1.f, 0.f, 1.f, 1.f};
const float CLR_YELLOW[4] = {1.f, 1.f, 0.f, 1.f};
const double RADIUS_SPH = 0.03;


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

using WayPoints = std::vector<Position>;

class MjSim
{
public:
	MjSim(){}
	mjModel* m = NULL;
	mjData* d = NULL;
	mjvCamera cam;
	mjvOption opt;
	mjvScene scn;
	mjrContext con;
	bool button_left = false;
	bool button_middle = false;
	bool button_right = false;
	double lastx = 0;
	double lasty = 0;
};
MjSim mj;

// UI state for spline toggle
static int show_spline = 1; // 1=ON, 0=OFF
static mjUI ui0;            // single UI panel
static mjuiState uistate;   // state

// Build minimal UI with a radio button group to toggle spline visibility
static void build_ui(const mjrContext* con) {
	mjuiDef def[] = {
		{ mjITEM_SECTION, "Display", 0, nullptr, "" },
		{ mjITEM_RADIO,   "Spline", 1, &show_spline, "Off\nOn" },
		{ mjITEM_END,     "", 0, nullptr, "" }
	};
	mjui_add(&ui0, def);
	if (ui0.nsect > 0) ui0.sect[0].state = mjSECT_OPEN; // セクション展開
	ui0.spacing = mjui_themeSpacing(0);
	ui0.color   = mjui_themeColor(0);
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
	if (in_ui && uistate.type != mjEVENT_MOVE) {
		mjui_event(&ui0, &uistate, con); // show_spline更新
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

void catmullRomSplinePoints(const WayPoints& wp, std::vector<Position>& out_points, int num_steps = 20) {
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

int drawSph(
	MjSim& mj,
	const Position& pt,
	double radius,
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
	mjtNum sphsize[3] = {radius, 0, 0};
    mjtNum myrot3x3[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
	mjv_initGeom(g, mjGEOM_SPHERE, sphsize, &pt.x, myrot3x3, rgba);
    g->objtype = mjOBJ_UNKNOWN;
    g->objid = -1;
    g->category = mjCAT_DECOR;
    g->segid = scn.ngeom;
	strncpy_s(g->label, sizeof(g->label), name.c_str(), _TRUNCATE);

	scn.ngeom++;
	return EXIT_SUCCESS;
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

int drawSpline(
	MjSim& mj,
	const WayPoints& wp,
	const float rgba[4]
) {
	if (wp.size() < 2) return EXIT_SUCCESS;

	std::vector<Position> points;
	catmullRomSplinePoints(wp, points);

	if (points.empty()) return EXIT_SUCCESS;
	Position prev = points[0];

	for (const auto& pt : points) {
		if (drawLine(mj, prev, pt, rgba) != EXIT_SUCCESS) 
			return EXIT_FAILURE;
		prev = pt;
	}
	return EXIT_SUCCESS;
}

int drawPath(
	MjSim& mj,
	const Path& path,
	const float rgba[4])
{
	const std::vector<PathPoint>& points = path.points;
	Position prev = points[0].point;
	for (const auto& pt : points) {
		if (drawLine(mj, prev, pt.point, rgba) != EXIT_SUCCESS) 
			return EXIT_FAILURE;
		prev = pt.point;
	}
	return EXIT_SUCCESS;
}

void create3rdSpline(
	const WayPoints& wp,
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

void generatePath(const WayPoints& wp, Path& path)
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

// Second-order-delayed system (Position type supported)
class SecondOrderDynamics {
public:
	// Parameters: inertia m, damping ratio zeta, natural angular frequency omega
	double m;
	double zeta;
	double omega; // rad/s
	Position y;    // Output (position)
	Position yd;   // First derivative (velocity)

	SecondOrderDynamics(double m_, double zeta_, double omega_, const Position& initp)
		: m(m_), zeta(zeta_), omega(omega_), y(initp), yd{0,0,0} {}

	// ref: reference input, dt: control timestep
	// Continuous system: y'' + 2*zeta*omega*y' + omega^2 * y = omega^2 * ref
	// Numerical integration: Forward Euler (extendable to Heun / Runge-Kutta if needed)
	Position update(const Position& ref, double dt) {
		// ydd = ω^2 (ref - y) - 2 ζ ω yd
		Position ydd = (ref - y) * (omega * omega) - yd * (2.0 * zeta * omega);
		yd += ydd * dt;
		y  += yd * dt;
		return y;
	}
};

int main(int argc, const char** argv) {

	// Decide model file path
	const char* model_path = "../models/urdf/robot/panda_arm_mjcf.xml";
	if (argc == 2) model_path = argv[1];

	// load and compile model
	char error[1000] = "Could not load model";
	mj.m = mj_loadXML(model_path, 0, error, 1000);
	if (!mj.m) {
		mju_error("Load model error: %s", error);
	}

	// make data
	mj.d = mj_makeData(mj.m);

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
	mj.cam.distance = 2.0 * mj.m->stat.extent;
	mj.cam.elevation = -20.0;
	mj.cam.azimuth = 90.0;

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	printf("Timestep: %f seconds\n", mj.m->opt.timestep);

/*
	// --- Embedded Python: create a quick plot via matplotlib to verify embedding ---
	// This will produce 'cpp_plot.png' in the current working directory.
	auto runPythonMatplotlibSample = []() -> bool {
		// Initialize Python interpreter if not already
		if (!Py_IsInitialized()) {
			Py_Initialize();
		}
		const char* script = R"PY(
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
x = np.linspace(0, 10, 200)
y = np.sin(x)
plt.figure()
plt.plot(x, y)
plt.title('Embedded Python plot from C++')
plt.xlabel('x')
plt.ylabel('sin(x)')
plt.savefig('cpp_plot.png')
print('cpp_plot.png saved')
)PY";
		int rc = PyRun_SimpleString(script);
		if (rc != 0) {
			fprintf(stderr, "Embedded python script failed with code %d\n", rc);
			// Attempt to finalize interpreter anyway
			if (Py_IsInitialized()) Py_FinalizeEx();
			return false;
		}
		// Finalize python interpreter
		if (Py_IsInitialized()) Py_FinalizeEx();
		return true;
	};

	if (!runPythonMatplotlibSample()) {
		fprintf(stderr, "Warning: embedded matplotlib sample failed\n");
	} else {
		printf("Embedded matplotlib sample written: cpp_plot.png\n");
	}
*/
	// Create Path planner 
	auto constraintPathPlanner = std::make_shared<ConstraintPathPlanner>();
	int result = constraintPathPlanner->plan();

	// WayPoints Definition
	WayPoints blueSph_wp = {
		{0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0},
		{1.0, 0.5, 0.2}};
	// add line2
	WayPoints wp2 = {
		{0.0, 0.0, 0.0},
		{0.0, 1.1, 1.0},
		{1.0, 0.5, 0.5}};

	// Create a SimplePathReader instance
	Path blueSphPath;
	int T = 200;  // Define the duration of the path
	generatePath(blueSph_wp, blueSphPath);
	SimplePathReader blueSphPathReader(blueSphPath, T);

	Path blueSphMovedPath, redSphMovedPath, greenSphMovedPath;
	blueSphMovedPath.points.push_back({0.0, blueSphPathReader.update()});
	redSphMovedPath.points.push_back({0.0, {0.0, 0.0, 0.0}});
	greenSphMovedPath.points.push_back({0.0, {0.0, 0.0, 0.0}});

	// redSph
	SecondOrderDynamics redSph(0.1, 0.25, 1.0, {0.0, 0.0, 0.0});
	SecondOrderDynamics greenSph(0.1, 0.25, 2.0, {0.0, 0.0, 0.0});


	// run main loop, target real-time simulation and 60 fps rendering
	while (!glfwWindowShouldClose(window)) {
		double simstart = mj.d->time;
		while (mj.d->time - simstart < 1.0/60.0) {
			mj_step(mj.m, mj.d);
		}
	mjrRect viewport_full = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &viewport_full.width, &viewport_full.height);
	process_ui_events(window, &mj.con, viewport_full.width, viewport_full.height);
	ui_per_frame(&mj.con, viewport_full.width, viewport_full.height);

		// Update the scene first (this resets scn.ngeom)
		mjv_updateScene(mj.m, mj.d, &mj.opt, NULL, &mj.cam, mjCAT_ALL, &mj.scn);


		double dt = mj.d->time - simstart;
		int ec = EXIT_SUCCESS;
		if (show_spline) {
			ec = drawSpline(mj, blueSph_wp, CLR_YELLOW);
			if (ec != EXIT_SUCCESS) return ec;
		}

		// draw spheres and the moved path
		// blue sphere
		Position pos_ref = blueSphPathReader.update();
		ec = drawSph(mj, pos_ref, RADIUS_SPH, CLR_BLUE, "ref");
		if (ec != EXIT_SUCCESS) return ec;
		// red sphere
		Position pos_redSph = redSph.update(pos_ref, dt);
		ec = drawSph(mj, pos_redSph, RADIUS_SPH, CLR_RED, "p1");
		if (ec != EXIT_SUCCESS) return ec;
		// green sphere
		Position pos_greenSph = greenSph.update(pos_ref, dt);
		ec = drawSph(mj, pos_greenSph, RADIUS_SPH, CLR_GREEN, "p2");
		if (ec != EXIT_SUCCESS) return ec;
		

		auto drawMovedPath = [](MjSim& mj, Path& path, const Position& pos, const float rgba[4]) -> int {
			if((path.points.back().point - pos).norm2()>0.0001) {
				path.points.push_back({0.0, pos});
			}
			return drawPath(mj, path, rgba);
		};
		ec = drawMovedPath(mj, blueSphMovedPath, pos_ref, CLR_BLUE);
		if (ec != EXIT_SUCCESS) return ec;
		ec = drawMovedPath(mj, redSphMovedPath, pos_redSph, CLR_RED);
		if (ec != EXIT_SUCCESS) return ec;
		ec = drawMovedPath(mj, greenSphMovedPath, pos_greenSph, CLR_GREEN);
		if (ec != EXIT_SUCCESS) return ec;

//		printf("dt: %f, ngeom: %d\n", dt, mj.scn.ngeom);

		mj.opt.label = mjLABEL_GEOM;
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
