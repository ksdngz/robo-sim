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


## How to Build the Sample with CMake

1. Open a terminal and move to the `exp-cpp` directory:
   ```powershell
   cd path\to\exp-cpp
   ```

2. Create a build directory and run CMake:
   ```powershell
   mkdir build
   cd build
   cmake ..
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
