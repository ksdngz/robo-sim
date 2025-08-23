@echo on
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

REM === Boost (minimal libs needed by OMPL) ===
set BOOST_VER=1_84_0
set BOOST_DIR=third_party\boost_%BOOST_VER%
set BOOST_ZIP=boost_%BOOST_VER%.zip
set BOOST_URL=https://archives.boost.io/release/1.84.0/source/boost_%BOOST_VER%.zip

if exist %BOOST_DIR%\stage\lib\boost_system.lib (
	echo Boost already built in %BOOST_DIR%
) else (
	if not exist %BOOST_DIR% (
		echo Downloading Boost %BOOST_VER%
		powershell -Command "Invoke-WebRequest -Uri %BOOST_URL% -OutFile %BOOST_ZIP%"
		echo Extracting Boost ...
		powershell -Command "Expand-Archive -Path %BOOST_ZIP% -DestinationPath third_party"
		del %BOOST_ZIP%
	)
	pushd %BOOST_DIR%
	echo Bootstrapping Boost ...
	call bootstrap.bat >NUL
	echo Building Boost ^(system filesystem program_options serialization^)...
	REM 以下をリンクを参考に使用
	REM https://www.kkaneko.jp/tools/win/boost.html
	REM https://qiita.com/aloac/items/73aa086924f121603839
	REM .\b2.exe --prefix=build --build-type=complete toolset=msvc link=static,shared address-model=64 install
	REM .\b2.exe --with-system --with-filesystem --with-program_options --with-serialization--prefix=build --build-type=complete toolset=msvc link=static,shared address-model=64 install
	REM 以下は自動生成されたが使用してない

	REM	b2 --with-system --with-filesystem --with-program_options --with-serialization -j %NUMBER_OF_PROCESSORS% variant=release link=static runtime-link=shared address-model=64 stage >NUL
	popd
)

REM === OMPL (download, build, install) ===
set OMPL_VER=1.6.0
set OMPL_ZIP=ompl-%OMPL_VER%.zip
set OMPL_URL=https://github.com/ompl/ompl/archive/refs/tags/%OMPL_VER%.zip
set OMPL_SRC=third_party\ompl
set OMPL_INSTALL=%OMPL_SRC%\install
if exist %OMPL_INSTALL%\share\ompl\cmake\omplConfig.cmake (
	echo OMPL already installed in %OMPL_INSTALL%
) else (
	if exist %OMPL_SRC% rmdir /s /q %OMPL_SRC%
	echo Downloading OMPL %OMPL_VER%
	powershell -Command "Invoke-WebRequest -Uri %OMPL_URL% -OutFile %OMPL_ZIP%"
	echo Extracting OMPL ...
	powershell -Command "Expand-Archive -Path %OMPL_ZIP% -DestinationPath third_party"
	rename third_party\ompl-%OMPL_VER% ompl
	del %OMPL_ZIP%
	mkdir %OMPL_SRC%\build 2>NUL
	pushd %OMPL_SRC%\build
	echo Configuring OMPL ...
	cmake -G "Visual Studio 17 2022" -T ClangCL ^
		-DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_APPS=OFF ^
		-DBOOST_ROOT="%CD%\..\..\boost_%BOOST_VER%" ^
		-DBOOST_LIBRARYDIR="%CD%\..\..\boost_%BOOST_VER%\stage\lib" ^
		-DCMAKE_INSTALL_PREFIX="%CD%\..\install" ..
	echo Building & installing OMPL ...
	cmake --build . --config Release --target install
	popd
	if exist %OMPL_INSTALL%\share\ompl\cmake\omplConfig.cmake (
		echo OMPL installed in %OMPL_INSTALL%
	) else (
		echo OMPL install appears to have failed.
	)
)

REM === Install Embedded Python (minimal, embeddable distribution) and required pip packages ===
set PYTHON_VER=3.10.9
set PYTHON_ZIP=python-%PYTHON_VER%-embed-amd64.zip
set PYTHON_URL=https://www.python.org/ftp/python/%PYTHON_VER%/%PYTHON_ZIP%
set PYTHON_DIR=third_party\python

if exist "%PYTHON_DIR%\python.exe" (
	echo Embedded Python already installed in %PYTHON_DIR%
) else (
	echo Downloading Embedded Python %PYTHON_VER%
	powershell -Command "Invoke-WebRequest -Uri %PYTHON_URL% -OutFile %PYTHON_ZIP%"
	echo Extracting Embedded Python ...
	powershell -Command "Expand-Archive -Path %PYTHON_ZIP% -DestinationPath %PYTHON_DIR%"
	del %PYTHON_ZIP%
	echo Embedded Python installed in %PYTHON_DIR%
)

REM Ensure python import library (python39.lib) exists at %PYTHON_DIR% (copy from scripts/download_python if missing)
if not exist "%PYTHON_DIR%\python39.lib" (
	if exist "scripts\download_python\python39.lib" (
		echo Copying python39.lib into %PYTHON_DIR%
		copy /Y "scripts\download_python\python39.lib" "%PYTHON_DIR%\python39.lib" >NUL
	) else (
		echo Warning: scripts\download_python\python39.lib not found. Python linking may fail.
	)
) else (
	echo python39.lib already present in %PYTHON_DIR%
)

REM Ensure pip + setuptools + wheel are available in the embedded Python
if exist "%PYTHON_DIR%\python.exe" (
	echo Enabling pip in embedded Python...
	%PYTHON_DIR%\python.exe -m ensurepip --default-pip || (
		echo ensurepip failed, attempting to bootstrap get-pip.py
		powershell -Command "Invoke-WebRequest -Uri https://bootstrap.pypa.io/get-pip.py -OutFile get-pip.py"
		%PYTHON_DIR%\python.exe get-pip.py
		del get-pip.py
	)
	echo Upgrading pip/setuptools/wheel
	%PYTHON_DIR%\python.exe -m pip install --upgrade pip setuptools wheel

	REM Install common scientific packages used for plotting
	echo Installing numpy and matplotlib into embedded Python - this may take a while...
	%PYTHON_DIR%\python.exe -m pip install numpy matplotlib
	echo pip install finished

	REM --- Ensure embeddable Python can load site-packages by enabling 'import site' in the _pth file ---
	echo Checking for embeddable _pth file and enabling import site if needed
	powershell -NoProfile -Command "$pth = Get-ChildItem -Path '%PYTHON_DIR%' -Filter '*_pth' -File -ErrorAction SilentlyContinue | Select-Object -First 1; if ($pth) { Copy-Item $pth.FullName ($pth.FullName + '.bak') -Force; (Get-Content $pth.FullName) -replace '^[#\s]*import site','import site' | Set-Content $pth.FullName; Write-Output 'patched:' + $pth.FullName } else { Write-Output 'no_pth_found' }"

	REM Temporarily add Scripts folder to PATH for this script execution so pip scripts are callable
	set "PATH=%PYTHON_DIR%\Scripts;%PATH%"
	echo Scripts folder added to PATH for this run

	REM Re-run pip install to ensure packages are available after enabling site
	echo Reinstalling numpy and matplotlib to ensure availability
	%PYTHON_DIR%\python.exe -m pip install --upgrade pip setuptools wheel
	%PYTHON_DIR%\python.exe -m pip install --no-warn-script-location --upgrade numpy matplotlib
	echo pip reinstall finished
)

REM Create a simple matplotlibrc to force Agg backend (headless) to avoid GUI backend issues
if exist "%PYTHON_DIR%\python.exe" (
	echo Creating matplotlibrc to use Agg - headless
	echo backend : Agg ^> "%PYTHON_DIR%\matplotlibrc"
	echo matplotlibrc created at %PYTHON_DIR%\matplotlibrc
)

REM === Ensure Python C API headers: if Include dir absent, download source and copy ===
set PY_INCLUDE_DIR=%PYTHON_DIR%\Include
if exist "%PY_INCLUDE_DIR%\Python.h" (
	echo Python headers present: %PY_INCLUDE_DIR%
) else (
	call :DOWNLOAD_PY_HEADERS
)

goto :AFTER_PY_HEADERS

:DOWNLOAD_PY_HEADERS
echo Python headers not found. Downloading source archive...
set PY_SRC_TGZ=Python-%PYTHON_VER%.tgz
powershell -NoProfile -Command "Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/%PYTHON_VER%/Python-%PYTHON_VER%.tgz' -OutFile '%PY_SRC_TGZ%' | Out-Null" || (
	echo Failed to download Python source archive & goto :CLEANUP_PY_HEADERS
)
if not exist "%PY_SRC_TGZ%" (
	echo Archive file %PY_SRC_TGZ% not found after download & goto :CLEANUP_PY_HEADERS
)
if exist py_src_tmp rmdir /s /q py_src_tmp
mkdir py_src_tmp
tar -xf "%PY_SRC_TGZ%" -C py_src_tmp 2>NUL || (
	echo tar extraction failed & goto :CLEANUP_PY_HEADERS
)
if exist py_src_tmp\Python-%PYTHON_VER%\Include (
	echo Copying headers into %PY_INCLUDE_DIR%
	if not exist "%PY_INCLUDE_DIR%" mkdir "%PY_INCLUDE_DIR%"
	xcopy /E /I /Y py_src_tmp\Python-%PYTHON_VER%\Include %PY_INCLUDE_DIR% >NUL
	echo Python headers installed.
) else (
	echo Failed to locate extracted Include directory ^(py_src_tmp\Python-%PYTHON_VER%\Include ^). Install manually if build fails.
)

:CLEANUP_PY_HEADERS
del "%PY_SRC_TGZ%" 2>NUL
rmdir /s /q py_src_tmp 2>NUL
goto :eof

:AFTER_PY_HEADERS

REM Ensure pyconfig.h exists (copy from scripts\download_python if available)
if not exist "%PY_INCLUDE_DIR%\pyconfig.h" (
	if exist "scripts\download_python\pyconfig.h" (
		echo Copying pyconfig.h into %PY_INCLUDE_DIR%
		if not exist "%PY_INCLUDE_DIR%" mkdir "%PY_INCLUDE_DIR%"
		copy /Y "scripts\download_python\pyconfig.h" "%PY_INCLUDE_DIR%\pyconfig.h" >NUL
		echo pyconfig.h copied.
	) else (
		echo Warning: scripts\download_python\pyconfig.h not found; pyconfig.h still missing.
	)
) else (
	echo pyconfig.h already present.
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
