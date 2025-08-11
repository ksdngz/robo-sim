@echo off
REM Bootstrap script: download MuJoCo and build with specified build system and compiler

REM === Download MuJoCo (PowerShell) ===
powershell -ExecutionPolicy Bypass -File scripts/download_mujoco.ps1

REM === Settings: edit as needed ===
set GENERATOR=Ninja
set CC=clang
set CXX=clang++

REM === Create build directory ===
if not exist build mkdir build
cd build

REM === Run CMake ===
cmake -G "%GENERATOR%" -DCMAKE_C_COMPILER=%CC% -DCMAKE_CXX_COMPILER=%CXX% ..

REM === Build ===
cmake --build .
