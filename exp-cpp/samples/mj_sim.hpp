#pragma once
#include <mujoco/mujoco.h>

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
