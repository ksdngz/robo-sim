// Simple MuJoCo simulation sample: visualize panda_arm_mjcf.xml
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <array>
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

//using Position = std::array<double, 3>;

class Position
{
public:
	double x;
	double y;
	double z;

	Position operator+(const Position& rhs) const {
		return {x + rhs.x, y + rhs.y, z + rhs.z};
	}
	Position operator-(const Position& rhs) const {
		return {x - rhs.x, y - rhs.y, z - rhs.z};
	}
	operator std::array<double,3>() const {
		return {x, y, z};	
	}
	double norm2() const {
		return std::sqrt(x * x + y * y + z * z);
	}
};

using WayPoints = std::vector<Position>;


const float CLR_RED[4] = {1.f, 0.f, 0.f, 1.f};
//const float CLR_GREEN[4] = {0.f, 1.f, 0.f, 1.f};
const float CLR_BLUE[4] = {0.f, 0.f, 1.f, 1.f};
const float CLR_PURPLE[4] = {1.f, 0.f, 1.f, 1.f};
const float CLR_YELLOW[4] = {1.f, 1.f, 0.f, 1.f};
const double RADIUS_SPH = 0.03;

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
	const float rgba[4])
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
	strncpy_s(g->label, sizeof(g->label), "mySph1", _TRUNCATE);

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


void generatePath(const WayPoints& wp, Path& path)
{
	if(path.points.size() > 0) 
		path.points.clear();

	std::vector<Position> points;
	create3rdSpline(wp, points);

	double totalNorm = 0.0;
	for (const auto& p : points) {
		totalNorm += p.norm2();
	}

	Position prev = points[0];
	double rate = 0.0;
	for (const auto& p : points) {
		rate += ((p - prev).norm2() / totalNorm);
		path.points.push_back({rate, p});
		prev = p;
	}
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
	SimplePathReader(const Path& path)
	: path_(path)
	, index_(0){}

	Position& update(){
		if(index_ >= path_.points.size()) {
			return path_.points.back().point;
		}
		return path_.points[index_++].point;
	}
private:
	Path path_;
	int index_;
};

int main(int argc, const char** argv) {

	// モデルファイルパスを決定
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

	// create scene and context
	mjv_makeScene(mj.m, &mj.scn, 2000);
	mj.scn.flags[mjCAT_DECOR] = 1;
	mjr_makeContext(mj.m, &mj.con, mjFONTSCALE_150);

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	printf("Timestep: %f seconds\n", mj.m->opt.timestep);

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
	Path blueSphPath, blueSphMovedPath;
	generatePath(blueSph_wp, blueSphPath);
	SimplePathReader blueSphPathReader(blueSphPath);
	blueSphMovedPath.points.push_back({0.0, blueSphPathReader.update()});

	int ec = EXIT_SUCCESS;
//		ec = drawSpline(mj, blueSph_wp, CLR_PURPLE);
//		if (ec != EXIT_SUCCESS) return ec;

	ec = drawSpline(mj, wp2, CLR_PURPLE);
	if (ec != EXIT_SUCCESS) return ec;

	// draw sphere
	Position p_leader = {1.0, 0.0, 0.0};
	ec = drawSph(mj, p_leader, RADIUS_SPH, CLR_RED);
	if (ec != EXIT_SUCCESS) return ec;


	// run main loop, target real-time simulation and 60 fps rendering
	while (!glfwWindowShouldClose(window)) {
		double simstart = mj.d->time;
		while (mj.d->time - simstart < 1.0/60.0) {
			mj_step(mj.m, mj.d);
		}
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// Update the scene first (this resets scn.ngeom)
		mjv_updateScene(mj.m, mj.d, &mj.opt, NULL, &mj.cam, mjCAT_ALL, &mj.scn);

		printf("ngeom pre: %d\n", mj.scn.ngeom);

		// draw blue sphere and the moved path
		Position pos_blueSph = blueSphPathReader.update();
		ec = drawSph(mj, pos_blueSph, RADIUS_SPH, CLR_BLUE);
		if (ec != EXIT_SUCCESS) return ec;

		if((blueSphMovedPath.points.back().point - pos_blueSph).norm2()>0.0001) {
			blueSphMovedPath.points.push_back({0.0, pos_blueSph});
		}
		ec = drawPath(mj, blueSphMovedPath, CLR_YELLOW);
		if (ec != EXIT_SUCCESS) return ec;

		printf("ngeom: %d\n", mj.scn.ngeom);

		mj.opt.label = mjLABEL_GEOM;
		mjv_addGeoms(mj.m, mj.d, &mj.opt, NULL, mjCAT_DECOR, &mj.scn);
		mjr_render(viewport, &mj.scn, &mj.con);
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
