@echo off
REM Bootstrap script: download MuJoCo and build with MSBuild and ClangCL (Visual Studio Generator)

REM === Download MuJoCo (PowerShell) ===
powershell -ExecutionPolicy Bypass -File scripts/download_mujoco.ps1

REM === Download and extract GLFW (Visual Studio precompiled binaries) ===
set GLFW_VER=3.3.8
set GLFW_ZIP=glfw-%GLFW_VER%.bin.WIN64.zip
set GLFW_URL=https://github.com/glfw/glfw/releases/download/%GLFW_VER%/%GLFW_ZIP%
set GLFW_DIR=third_party\glfw

if exist %GLFW_DIR%\include\GLFW\glfw3.h (
	echo GLFW already installed in %GLFW_DIR%
) else (
	echo Downloading GLFW %GLFW_VER% Visual Studio binaries
	powershell -Command "Invoke-WebRequest -Uri %GLFW_URL% -OutFile %GLFW_ZIP%"
	echo Extracting GLFW ...
	powershell -Command "Expand-Archive -Path %GLFW_ZIP% -DestinationPath %GLFW_DIR%"
	if exist %GLFW_DIR%\include rmdir /s /q %GLFW_DIR%\include
	if exist %GLFW_DIR%\lib rmdir /s /q %GLFW_DIR%\lib
	move %GLFW_DIR%\glfw-%GLFW_VER%.bin.WIN64\include %GLFW_DIR%\include
	move %GLFW_DIR%\glfw-%GLFW_VER%.bin.WIN64\lib-vc2022 %GLFW_DIR%\lib
	rmdir /s /q %GLFW_DIR%\glfw-%GLFW_VER%.bin.WIN64
	del %GLFW_ZIP%
	echo GLFW VC installed in %GLFW_DIR%
)

REM === Download and extract Eigen ===
set EIGEN_VER=3.4.0
set EIGEN_ZIP=eigen-%EIGEN_VER%.zip
set EIGEN_URL=https://gitlab.com/libeigen/eigen/-/archive/%EIGEN_VER%/%EIGEN_ZIP%
set EIGEN_DIR=third_party\eigen

if exist %EIGEN_DIR%\Eigen\Dense (
    echo Eigen already installed in %EIGEN_DIR%
) else (
    echo Downloading Eigen %EIGEN_VER%
    powershell -Command "Invoke-WebRequest -Uri %EIGEN_URL% -OutFile %EIGEN_ZIP%"
    echo Extracting Eigen ...
    powershell -Command "Expand-Archive -Path %EIGEN_ZIP% -DestinationPath third_party"
    move third_party\eigen-%EIGEN_VER% %EIGEN_DIR%
    del %EIGEN_ZIP%
    echo Eigen installed in %EIGEN_DIR%
)

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
