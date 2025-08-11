@echo off
REM Bootstrap script: download MuJoCo and build with MSBuild and ClangCL (Visual Studio Generator)

REM === Download MuJoCo (PowerShell) ===
powershell -ExecutionPolicy Bypass -File scripts/download_mujoco.ps1

REM === Settings: Visual Studio Generator with ClangCL ===
set GENERATOR=Visual Studio 17 2022
set TOOLSET=ClangCL

REM === Create build directory ===
if not exist build mkdir build
cd build

REM === Run CMake ===
cmake -G "%GENERATOR%" -T %TOOLSET% ..

REM === Build ===
cmake --build .
