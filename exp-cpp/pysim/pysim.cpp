// pysim.cpp
// 2-DOF (x,y) point mass simulation using second order dynamics and Python (matplotlib) animation.
// Requirements implemented:
//  - DynamicsSimulation class defined here, invoked from main
//  - Uses common/SecondOrderDynamics.hpp (second-order lag system)
//  - Embeds Python via Python.h to call a matplotlib animation script (pysim/sim_animation.py)
//  - Saves animation to file (gif/mp4 configurable via parameter)
//  - Dynamics parameters and initial state configurable through the class interface

// Ensure Python headers are included without debug refcount macros that require a debug-build Python.
#define PY_SSIZE_T_CLEAN
#ifdef _DEBUG
	#define PYSIM_RESTORE_DEBUG 1
	#undef _DEBUG
#endif
#ifdef Py_DEBUG
	#undef Py_DEBUG
#endif
#ifdef Py_REF_DEBUG
	#undef Py_REF_DEBUG
#endif
#include <Python.h>
#ifdef PYSIM_RESTORE_DEBUG
	#ifndef _DEBUG
		#define _DEBUG 1
	#endif
	#undef PYSIM_RESTORE_DEBUG
#endif

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include <optional>

#include "common/Position.hpp"
#include "common/SecondOrderDynamics.hpp"

class DynamicsSimulation {
public:
	struct Params {
		double m      = 1.0;      // inertia parameter (used in struct but original SecondOrderDynamics currently not using 'm')
		double zeta   = 0.7;      // damping ratio
		double omega  = 2.0;      // natural angular frequency (rad/s)
		double dt     = 0.01;     // timestep (s)
		double T      = 10.0;     // total simulation time (s)
		Position init_pos {0.0, 0.0, 0.0}; // initial position (z unused)
		Position init_vel {0.0, 0.0, 0.0}; // initial velocity (z unused)
		std::string save_path = "simulation.gif"; // output animation file (extension determines writer)
		bool show_window = false; // show interactive window as well
	};

	explicit DynamicsSimulation(const Params& p) : params(p) {}

	void run() {
		allocate();
		simulate();
		sendToPython();
	}

private:
	Params params;
	std::vector<double> time;
	std::vector<double> xs;
	std::vector<double> ys;

	void allocate() {
		size_t steps = static_cast<size_t>(std::ceil(params.T / params.dt)) + 1;
		time.reserve(steps);
		xs.reserve(steps);
		ys.reserve(steps);
	}

	Position reference(double t) {
		// Reference trajectory (Lissajous / circular hybrid) for richer motion
		double xr = 0.5 * std::cos(0.6 * t) + 0.3 * std::sin(1.3 * t);
		double yr = 0.5 * std::sin(0.8 * t) + 0.3 * std::cos(1.1 * t);
		return {xr, yr, 0.0};
	}

	void simulate() {
		SecondOrderDynamics dyn(params.m, params.zeta, params.omega, params.init_pos);
		// set initial velocity
		dyn.yd.x = params.init_vel.x;
		dyn.yd.y = params.init_vel.y;
		dyn.yd.z = 0.0; // unused

		double t = 0.0;
		while (t <= params.T + 1e-9) {
			Position ref = reference(t);
			Position y = dyn.update(ref, params.dt);
			time.push_back(t);
			xs.push_back(y.x);
			ys.push_back(y.y);
			t += params.dt;
		}
	}

	void sendToPython() {
		if (time.empty()) {
			std::cerr << "[DynamicsSimulation] No data to send to Python.\n";
			return;
		}

		// Initialize Python
		if (!Py_IsInitialized()) {
			Py_Initialize();
		}

		// Add script directory to sys.path.
		// Use the directory of the current source file (pysim.cpp) so that running from build/Debug finds the script in source tree.
		try {
			auto scriptDir = std::filesystem::path(__FILE__).parent_path(); // .../exp-cpp/pysim
			if (!std::filesystem::exists(scriptDir / "sim_animation.py")) {
				std::cerr << "[DynamicsSimulation] Warning: sim_animation.py not found at " << (scriptDir / "sim_animation.py") << "\n";
			}
			std::string pathStr = scriptDir.string();
			for (size_t pos = 0; pos < pathStr.size(); ++pos) {
				if (pathStr[pos] == '\\') { pathStr.insert(pos, 1, '\\'); ++pos; }
			}
			std::string addPathCmd = "import sys, os; p='" + pathStr + "';\n";
			addPathCmd += "p=os.path.abspath(p); (p not in sys.path) and sys.path.insert(0,p)";
			PyRun_SimpleString(addPathCmd.c_str());
		} catch (const std::exception& e) {
			std::cerr << "[DynamicsSimulation] Failed to set Python path: " << e.what() << "\n";
		}

		PyObject* pName = PyUnicode_DecodeFSDefault("sim_animation");
		PyObject* pModule = PyImport_Import(pName);
		Py_DECREF(pName);

		if (!pModule) {
			std::cerr << "[DynamicsSimulation] Failed to import sim_animation.py\n";
			PyErr_Print();
			finalizePython();
			return;
		}

		PyObject* pFunc = PyObject_GetAttrString(pModule, "animate_sim");
		if (!pFunc || !PyCallable_Check(pFunc)) {
			std::cerr << "[DynamicsSimulation] animate_sim not callable.\n";
			PyErr_Print();
			Py_XDECREF(pFunc);
			Py_DECREF(pModule);
			finalizePython();
			return;
		}

		PyObject* pyTimes = vectorToPyList(time);
		PyObject* pyXs    = vectorToPyList(xs);
		PyObject* pyYs    = vectorToPyList(ys);

		PyObject* pySave = PyUnicode_FromString(params.save_path.c_str());
		PyObject* pyShow = PyBool_FromLong(params.show_window ? 1 : 0); // new reference

		PyObject* args = PyTuple_New(5);
		PyTuple_SetItem(args, 0, pyTimes); // steals ref
		PyTuple_SetItem(args, 1, pyXs);
		PyTuple_SetItem(args, 2, pyYs);
		PyTuple_SetItem(args, 3, pySave);
		PyTuple_SetItem(args, 4, pyShow);

		PyObject* result = PyObject_CallObject(pFunc, args);
		if (!result) {
			std::cerr << "[DynamicsSimulation] Python call failed.\n";
			PyErr_Print();
		} else {
			Py_DECREF(result);
			std::cout << "[DynamicsSimulation] Animation script executed. Saved to: " << params.save_path << "\n";
		}

		Py_DECREF(args);
		Py_DECREF(pFunc);
		Py_DECREF(pModule);

		finalizePython();
	}

	static PyObject* vectorToPyList(const std::vector<double>& v) {
		PyObject* listObj = PyList_New(static_cast<Py_ssize_t>(v.size()));
		for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(v.size()); ++i) {
			PyObject* f = PyFloat_FromDouble(v[static_cast<size_t>(i)]);
			PyList_SET_ITEM(listObj, i, f); // steals reference to f
		}
		return listObj;
	}

	static void finalizePython() {
		if (Py_IsInitialized()) {
			Py_Finalize();
		}
	}
};

int main(int argc, const char** argv) {
	std::cout << "Starting 2-DOF second-order dynamics simulation..." << std::endl;

	DynamicsSimulation::Params p;
	// Optional: parse minimal command-line overrides (dt, T, save file)
	// Example usage: pysim.exe 0.005 8.0 out.mp4
	if (argc > 1) p.dt = std::stod(argv[1]);
	if (argc > 2) p.T = std::stod(argv[2]);
	if (argc > 3) p.save_path = argv[3];

	// Example custom initial conditions / parameters (could also be exposed via CLI):
	p.m = 1.0;
	p.zeta = 0.9;
	p.omega = 3.0;    // faster response
	p.init_pos = {0.2, -0.1, 0.0};
	p.init_vel = {0.0, 0.0, 0.0};
	p.show_window = false; // set true to display interactive window (if backend supports)

	DynamicsSimulation sim(p);
	sim.run();

	std::cout << "Simulation complete." << std::endl;
	return 0;
}
