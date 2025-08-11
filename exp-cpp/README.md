

# exp-cpp: MuJoCo + GLFW C++ Simulation Environment

This directory provides a C++ sample environment for physics simulation using MuJoCo 3.3.5 and GLFW. It automatically downloads the official MuJoCo and GLFW binaries, and supports build/run with Visual Studio and ClangCL.

---

## 1. Quick Setup (Automatic)

Just run `bootstrap.bat` to automatically:
- Download and extract MuJoCo 3.3.5 (`mujoco-3.3.5/`)
- Download and extract GLFW 3.3.8 (`third_party/glfw/`)
- Generate build directory and build with CMake

```powershell
cd path\to\exp-cpp
bootstrap.bat
```

---

## 2. How to Run the Sample

After building, run the sample executable:

```powershell
cd build
Debug\simple_sim.exe
```

---

## 3. Manual Setup Instructions

1. Download MuJoCo 3.3.5 official binary from https://github.com/google-deepmind/mujoco/releases and extract to `mujoco-3.3.5/`
2. Download GLFW 3.3.8 official binary (glfw-3.3.8.bin.WIN64.zip) from https://www.glfw.org/download.html and extract to `third_party/glfw/`
3. Build with the following commands:

```powershell
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -T ClangCL ..
cmake --build .
```

---

## 4. Notes

- MuJoCo/GLFW versions and paths are managed in `CMakeLists.txt`.
- The `build/` directory is ignored by `.gitignore` as it contains generated files.
- For details, see the scripts and `CMakeLists.txt`.

---

For more information about MuJoCo, visit: https://github.com/google-deepmind/mujoco
