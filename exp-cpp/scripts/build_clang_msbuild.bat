@echo off
REM Build with MSBuild and Clang (using Visual Studio Generator)
setlocal
if not exist build mkdir build
cd build
cmake -G "Visual Studio 17 2022" -T ClangCL ..
cmake --build .
endlocal
