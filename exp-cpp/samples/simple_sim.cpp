// Simple MuJoCo simulation sample: visualize panda_arm_mjcf.xml
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <array>
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

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

using Position = std::array<double, 3>;
using WayPoints = std::vector<Position>;

int drawLine(
	mjvScene& scn,
	const Position& from,
	const Position& to,
	float rgba[4])
{
	if ( scn.ngeom>=scn.maxgeom ) {
		mj_warning(d, mjWARN_VGEOMFULL, scn.maxgeom);
		return EXIT_FAILURE;
	}
	mjvGeom* g = scn.geoms + scn.ngeom;
	memset(g, 0, sizeof(mjvGeom));
	mjv_initGeom(g, mjGEOM_NONE, NULL, NULL, NULL, rgba);
	g->objtype = mjOBJ_UNKNOWN;
	g->objid = 0;
	g->category = mjCAT_DECOR;
	g->segid = scn.ngeom;
	mjv_connector(g, mjGEOM_LINE, 0.05, from.data(), to.data());
	scn.ngeom++;
	return EXIT_SUCCESS;
}

int drawSpline(
	mjvScene& scn,
	const WayPoints& wp,
	float rgba[4]
) {
	if (wp.size() < 2) return EXIT_SUCCESS; // nothing to draw

	// Catmull-Rom Spline: interpolate between wp[1] ... wp[N-2]
	const int NUM_STEPS = 20; // number of segments per span
	int err = EXIT_SUCCESS;
	for (size_t i = 0; i + 1 < wp.size(); ++i) {
		// For the first and last segment, duplicate endpoints for tangent
		const Position& p0 = (i == 0) ? wp[0] : wp[i-1];
		const Position& p1 = wp[i];
		const Position& p2 = wp[i+1];
		const Position& p3 = (i+2 < wp.size()) ? wp[i+2] : wp[wp.size()-1];

		// Interpolate between p1 and p2
		Position prev = p1;
		for (int j = 1; j <= NUM_STEPS; ++j) {
			double t = (double)j / NUM_STEPS;
			// Catmull-Rom formula
			Position pt;
			for (int k = 0; k < 3; ++k) {
				pt[k] = 0.5 * ((2.0 * p1[k]) +
					(-p0[k] + p2[k]) * t +
					(2.0*p0[k] - 5.0*p1[k] + 4.0*p2[k] - p3[k]) * t * t +
					(-p0[k] + 3.0*p1[k] - 3.0*p2[k] + p3[k]) * t * t * t);
			}
			if (drawLine(scn, prev, pt, rgba) != EXIT_SUCCESS) err = EXIT_FAILURE;
			prev = pt;
		}
	}
	return err;
}


void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
	if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
		mj_resetData(m, d);
		mj_forward(m, d);
	}
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
	glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
	if (!button_left && !button_middle && !button_right) return;
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
										glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
	mjtMouse action;
	if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else action = mjMOUSE_ZOOM;
	mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

int main(int argc, const char** argv) {
	// モデルファイルパスを決定
	const char* model_path = "../models/urdf/robot/panda_arm_mjcf.xml";
	if (argc == 2) model_path = argv[1];

	// load and compile model
	char error[1000] = "Could not load model";
	m = mj_loadXML(model_path, 0, error, 1000);
	if (!m) {
		mju_error("Load model error: %s", error);
	}

	// make data
	d = mj_makeData(m);

	// init GLFW
	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
	}

	// create window, make OpenGL context current, request v-sync
	GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simple Sim", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	scn.flags[mjCAT_DECOR] = 1;
	mjr_makeContext(m, &con, mjFONTSCALE_150);

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	// run main loop, target real-time simulation and 60 fps rendering
	while (!glfwWindowShouldClose(window)) {
		mjtNum simstart = d->time;
		while (d->time - simstart < 1.0/60.0) {
			mj_step(m, d);
		}
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// Update the scene first (this resets scn.ngeom)
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

		// add line
		WayPoints wp = {
			{0.0, 0.0, 0.0},
			{1.0, 0.0, 0.0},
			{1.0, 0.5, 0.2}};
		float rgba[4] = {1.f, 0.f, 1.f, 0.9f};
		int errCode = drawSpline(scn, wp, rgba);
		if (errCode != EXIT_SUCCESS){
		  return errCode;
		}

		mjr_render(viewport, &scn, &con);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	//free visualization storage
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
	mj_deleteData(d);
	mj_deleteModel(m);
#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
#endif
	return EXIT_SUCCESS;
}
