# MuJoCo 3.3.5 Setup Guide

This project uses MuJoCo 3.3.5 for physics simulation. The MuJoCo binary is not included in this repository by default. Please follow the steps below to download and extract the MuJoCo library.

## How to Download and Extract MuJoCo

1. Open a terminal and move to the `exp-cpp` directory:
   ```powershell
   cd path\to\exp-cpp
   ```

2. Run the provided PowerShell script:
   ```powershell
   powershell -ExecutionPolicy Bypass -File scripts/download_mujoco.ps1
   ```



## How to Build and Setup with Bootstrap Scripts

You can use the provided bootstrap scripts to automatically download MuJoCo and build the project with your preferred build system and compiler.

### 1. Build with Ninja and Clang (default)
In the `exp-cpp` directory, run:
```powershell
bootstrap.bat
```
This will download MuJoCo, configure CMake with Ninja and Clang, and build the project.

### 2. Build with MSBuild and ClangCL (Visual Studio)
In the `exp-cpp` directory, run:
```powershell
bootstrap_msbuild.bat
```
This will download MuJoCo, configure CMake with Visual Studio 17 2022 and ClangCL, and build the project.

---

## How to Build the Sample with CMake (Manual)

1. Open a terminal and move to the `exp-cpp` directory:
   ```powershell
   cd path\to\exp-cpp
   ```

2. (Option) Build with MSBuild and Clang using the provided script:
   ```powershell
   scripts\build_clang_msbuild.bat
   ```

   This script will create the build directory, configure CMake with Visual Studio 17 2022 and ClangCL, and build the project automatically.

3. (Manual) Create a build directory and run CMake (specifying Visual Studio 17 2022 as the generator):
   ```powershell
   mkdir build
   cd build
   cmake -G "Visual Studio 17 2022" ..
   cmake --build .
   ```

3. Run the sample executable (after build):
   ```powershell
   .\samples\simple_sim.exe
   ```

This will automatically download the MuJoCo 3.3.5 Windows binary from the official GitHub release and extract it to the `mujoco-3.3.5` directory.

If the directory already exists, the script will skip the download and extraction.

---

For more information about MuJoCo, visit: https://github.com/google-deepmind/mujoco
