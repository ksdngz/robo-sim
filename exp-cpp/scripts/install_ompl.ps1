# OMPL build/install helper (PowerShell)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Resolve repo root and third_party
$repo = (Resolve-Path "$PSScriptRoot\..").Path
$tp   = Join-Path $repo 'third_party'

# VS generator/toolset (align with project)
$vsGen   = 'Visual Studio 17 2022'
$toolset = 'ClangCL'

# Paths: Eigen (source -> local install), Boost (user-provided layout), OMPL
$eigenSrc    = Join-Path $tp 'eigen'
$eigenBuild  = Join-Path $eigenSrc 'build'
$eigenPrefix = Join-Path $tp 'eigen-install'
$eigenCfg    = Join-Path $eigenPrefix 'share\eigen3\cmake\Eigen3Config.cmake'

# Boost (prefer build layout, fallback to stage)
$boostBuildRoot = Join-Path $tp 'boost_1_84_0\build'
$boostStageRoot = Join-Path $tp 'boost_1_84_0'
$boostIncBuild  = Join-Path $boostBuildRoot 'include\boost-1_84'
$boostLibBuild  = Join-Path $boostBuildRoot 'lib'
$boostLibStage  = Join-Path $boostStageRoot 'stage\lib'

if (Test-Path $boostLibBuild) {
  $boostRoot = $boostBuildRoot
  $boostInc  = $boostIncBuild
  $boostLib  = $boostLibBuild
} elseif (Test-Path $boostLibStage) {
  $boostRoot = $boostStageRoot
  $boostInc  = Join-Path $boostStageRoot 'boost'
  $boostLib  = $boostLibStage
} else {
  throw "Boost libraries not found. Expected one of: `n  $boostLibBuild`n  $boostLibStage"
}

$omplSrc     = Join-Path $tp 'ompl'
$omplBuild   = Join-Path $omplSrc 'build\Release'
$omplPrefix  = Join-Path $omplSrc 'install'

Write-Host "Repo:        $repo"
Write-Host "Eigen src:   $eigenSrc"
Write-Host "Eigen cfg:   $eigenCfg"
Write-Host "Boost inc:   $boostInc"
Write-Host "Boost lib:   $boostLib"
Write-Host "OMPL src:    $omplSrc"
Write-Host "OMPL build:  $omplBuild"
Write-Host "OMPL prefix: $omplPrefix"

# 1) Ensure Eigen is installed (to get Eigen3Config.cmake for CONFIG find)
if (-not (Test-Path $eigenCfg)) {
  Write-Host "Installing Eigen headers to: $eigenPrefix"
  if (Test-Path $eigenBuild) { cmd /c "rmdir /s /q `"$eigenBuild`"" }
  New-Item -ItemType Directory -Force -Path $eigenBuild | Out-Null
  Push-Location $eigenBuild
  cmake .. -G "$vsGen" -T $toolset -DCMAKE_INSTALL_PREFIX="$eigenPrefix"
  cmake --build . --config Release --target install
  Pop-Location
  if (-not (Test-Path $eigenCfg)) { throw "Eigen3Config.cmake not found: $eigenCfg" }
}

# 2) Validate Boost libraries availability
$need = @('program_options','serialization')
foreach ($c in $need) {
  $found = Get-ChildItem -Path $boostLib -Filter "*${c}*.lib" -ErrorAction SilentlyContinue
  if (-not $found) { throw "Boost $c .lib not found in: $boostLib" }
}

# Prefer non-debug, multithreaded, toolset-tagged import libs
$poLib = Get-ChildItem -Path $boostLib -Filter 'boost_program_options-vc143-mt-*.lib' | Where-Object { $_.Name -notmatch '-gd-' } | Select-Object -First 1
if (-not $poLib) { $poLib = Get-ChildItem -Path $boostLib -Filter 'libboost_program_options-vc143-mt-*.lib' | Where-Object { $_.Name -notmatch '-gd-' } | Select-Object -First 1 }
$serLib = Get-ChildItem -Path $boostLib -Filter 'boost_serialization-vc143-mt-*.lib' | Where-Object { $_.Name -notmatch '-gd-' } | Select-Object -First 1
if (-not $serLib) { $serLib = Get-ChildItem -Path $boostLib -Filter 'libboost_serialization-vc143-mt-*.lib' | Where-Object { $_.Name -notmatch '-gd-' } | Select-Object -First 1 }
if (-not $poLib -or -not $serLib) { throw "Could not resolve Boost component libs in $boostLib" }
Write-Host "Using Boost PROGRAM_OPTIONS: $($poLib.Name)"
Write-Host "Using Boost SERIALIZATION:  $($serLib.Name)"

# Prefer dynamic Boost import libs to avoid variant mismatch with ClangCL
$useStatic = $false
Write-Host "Boost static: OFF (forcing dynamic import libs)"

# 3) Configure, build, and install OMPL
if (Test-Path $omplBuild) { cmd /c "rmdir /s /q `"$omplBuild`"" }
New-Item -ItemType Directory -Force -Path $omplBuild | Out-Null
Push-Location $omplBuild

cmake ../.. `
  -G "$vsGen" -T $toolset `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_INSTALL_PREFIX="$omplPrefix" `
  -DBoost_NO_BOOST_CMAKE=ON `
  -DBoost_NO_SYSTEM_PATHS=ON `
  -DBoost_USE_MULTITHREADED=ON `
  -DBoost_USE_STATIC_LIBS:BOOL=$useStatic `
  -DBoost_USE_STATIC_RUNTIME=OFF `
  -DBoost_COMPILER="-vc143" `
  -DBoost_DEBUG=ON `
  -DBoost_ROOT="$boostRoot" -DBOOST_ROOT="$boostRoot" `
  -DBoost_INCLUDE_DIR="$boostInc" -DBoost_INCLUDE_DIRS="$boostInc" -DBOOST_INCLUDEDIR="$boostInc" `
  -DBoost_LIBRARY_DIRS="$boostLib" -DBOOST_LIBRARYDIR="$boostLib" `
  -DBoost_ADDITIONAL_VERSIONS="1.84;1.84.0" `
  -DBoost_USE_DEBUG_RUNTIME=OFF `
  -DBoost_PROGRAM_OPTIONS_LIBRARY="$($poLib.FullName)" `
  -DBoost_SERIALIZATION_LIBRARY="$($serLib.FullName)" `
  -DEigen3_DIR="$eigenCfg"

cmake --build . --config Release --target install
Pop-Location

if (Test-Path (Join-Path $omplPrefix 'share\ompl\cmake\omplConfig.cmake')) {
  Write-Host "OMPL installed to: $omplPrefix"
} else {
  throw "OMPL install failed: omplConfig.cmake not found under $omplPrefix"
}