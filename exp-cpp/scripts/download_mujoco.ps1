# Script to download and extract MuJoCo 3.3.5 Windows binary
# Usage: powershell -ExecutionPolicy Bypass -File scripts/download_mujoco.ps1

$version = "3.3.5"
$baseUrl = "https://github.com/google-deepmind/mujoco/releases/download/v$version"
$zipName = "mujoco-3.3.5-windows-x86_64.zip"
$downloadUrl = "https://github.com/google-deepmind/mujoco/releases/download/3.3.5/$zipName"
$targetDir = "../mujoco-$version"


# Change working directory to the script location
Set-Location -Path $PSScriptRoot

# Skip if already exists
if (!(Test-Path $targetDir)) {
    Write-Host "Downloading MuJoCo $version ..."
    Invoke-WebRequest -Uri $downloadUrl -OutFile $zipName
    Write-Host "Extracting..."
    $tmpDir = "./tmp_mujoco_extract"
    Expand-Archive -Path $zipName -DestinationPath $tmpDir
    # Find subdirectory (mujoco-3.3.5)
    $innerDir = Join-Path $tmpDir "mujoco-$version"
    if (Test-Path $innerDir) {
        Move-Item -Path (Join-Path $innerDir '*') -Destination $targetDir -Force
    } else {
        Move-Item -Path (Join-Path $tmpDir '*') -Destination $targetDir -Force
    }
    Remove-Item $zipName
    Remove-Item $tmpDir -Recurse -Force
    Write-Host "Extracted to mujoco-$version directory."
} else {
    Write-Host "$targetDir already exists. Skipping."
}
