#pragma once
#include <mujoco/mujoco.h>

class MjSim
{
public:
	MjSim(std::string modelPath)
	{
		// load and compile model
		char error[1000] = "Could not load model";
		m = mj_loadXML(modelPath.c_str(), 0, error, 1000);
		if (!m) {
			mju_error("Load model error: %s", error);
		}

		// make data
		d = mj_makeData(m);
	}
	~MjSim()
	{
		mj_deleteData(d);
		mj_deleteModel(m);
	}

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
