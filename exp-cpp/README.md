

# exp-cpp: MuJoCo + GLFW C++ Simulation Environment

This directory provides a C++ sample environment for physics simulation using MuJoCo 3.3.5 and GLFW. It automatically downloads the official MuJoCo and GLFW binaries, and supports build/run with Visual Studio and ClangCL.

---

## 1. Quick Setup (Automatic)

Just run `bootstrap.bat` to automatically:
- Download and extract MuJoCo 3.3.5 (`mujoco-3.3.5/`)
- Download and extract GLFW 3.3.8 (`third_party/glfw/`)
- Build Boost (Release & Debug) into `third_party/boost_1_84_0/build/lib`
- Install OMPL via `scripts/install_ompl.ps1` (defaults to Release)
- Generate build directory and build with CMake

```powershell
cd path\to\exp-cpp
bootstrap.bat
```

If you also need OMPL Debug libs for Debug builds, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\install_ompl.ps1 -Config Debug
```

---

## 2. How to Run the Sample

After building, run the sample executable:

```powershell
cd build
Debug\simple_sim.exe
```

---

## 3. Manual Setup Instructions (including Debug/Release of OMPL/Boost)

1. Download MuJoCo 3.3.5 official binary from https://github.com/google-deepmind/mujoco/releases and extract to `mujoco-3.3.5/`
2. Download GLFW 3.3.8 official binary (glfw-3.3.8.bin.WIN64.zip) from https://www.glfw.org/download.html and extract to `third_party/glfw/`
3. To build the workspace, choose the configuration (RelWithDebInfo recommended for step-debug) and run:

```powershell
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -T ClangCL ..
cmake --build . --config RelWithDebInfo
```

To install OMPL explicitly in a given configuration:

```powershell
# Release (default)
powershell -ExecutionPolicy Bypass -File .\scripts\install_ompl.ps1 -Config Release

# Debug (for building Debug apps without _ITERATOR_DEBUG_LEVEL issues)
powershell -ExecutionPolicy Bypass -File .\scripts\install_ompl.ps1 -Config Debug
```

---

## 4. Notes

- MuJoCo/GLFW versions and paths are managed in `CMakeLists.txt`.
- The `build/` directory is ignored by `.gitignore` as it contains generated files.
- For details, see the scripts and `CMakeLists.txt`.

---

## 5. About Debugging and _ITERATOR_DEBUG_LEVEL

When mixing configurations on Windows, link errors can occur (e.g. `_ITERATOR_DEBUG_LEVEL` mismatch) if your app is Debug but libraries are Release. This repo solves it by:

- Building Boost in both Debug and Release variants (import libs in `third_party/boost_1_84_0/build/lib`).
- Installing OMPL for each needed configuration by running the installer per-config (single prefix `third_party/ompl/install`).

Tips:
- Build your target with the same configuration as the OMPL you intend to use:
	- Release: `cmake --build . --config Release --target path_plan`
	- Debug: `cmake --build . --config Debug --target path_plan`
	- RelWithDebInfo (recommended for step-debug): `cmake --build . --config RelWithDebInfo --target path_plan`
- VS Code launch configurations for `path_plan.exe` are provided in `.vscode/launch.json`.

---

For more information about MuJoCo, visit: https://github.com/google-deepmind/mujoco



## script memo
### pysim
x,y方向2自由度を持つ質点の時間シミュレーションを作成して。下記の条件を満たしてください。
- DynamicsSimulationクラスをpysim.cpp内に定義、main関数から呼び出してシミュレーションを実行する
- この質点は二次遅れ系のダイナミクスを持つ。common/SecondOrderDynamics.hppを用いて実装する
- シミュレーションはpythonのmatplotlibのアニメーションを用いる。python.h経由でpython上でアニメーション描画処理を実装する。描画用のpythonスクリプトは、pysim/sim_animation.pyとしてファイルを生成する
- 生成したアニメーションは描画に加えて、動画ファイル(gif/mp4など任意)として保存する
- dynamicsのパラメータ、質点初期位置、速度をDynamicsSimulationから設定可能とする
- 実装後、ビルドは実行しない
