// Simple MuJoCo simulation sample: visualize panda_arm_mjcf.xml
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
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

		// Then append custom decor geoms so they are not wiped by update
		mjvGeom* thisgeom = scn.geoms + scn.ngeom;
		// addCapsuleToScene(&scn, from, to, /*radius=*/0.04, rgba);
		if ( scn.ngeom>=scn.maxgeom ) {
		  mj_warning(d, mjWARN_VGEOMFULL, scn.maxgeom);
		  return EXIT_FAILURE;
		}
		else {
			memset(thisgeom, 0, sizeof(mjvGeom));
			float rgba[4]  = {1.f, 0.f, 1.f, 0.9f};
			mjv_initGeom(thisgeom, mjGEOM_NONE, NULL, NULL, NULL, rgba); 
			thisgeom->objtype = mjOBJ_UNKNOWN;
			thisgeom->objid = 0;
			thisgeom->category = mjCAT_DECOR;
			thisgeom->segid = scn.ngeom;
			mjtNum from[3] = {0.0, 0.0, 0.0};
			mjtNum to[3]   = {1.0, 0.5, 0.2};
			mjv_connector(thisgeom, mjGEOM_LINE, 0.05, from, to);
			scn.ngeom++;
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
