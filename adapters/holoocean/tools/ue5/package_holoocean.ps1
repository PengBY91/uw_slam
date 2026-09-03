# PREP-A-02 step: package the HoloOcean project (game target + cook + pak) into
# C:\Users\pengb\uw_worlds_build, then assemble the UwWorlds HoloOcean package
# under %LOCALAPPDATA%\holoocean\2.3.0\worlds\UwWorlds.
$ErrorActionPreference = "Continue"
$log = "C:\Users\pengb\uw_slam_holoocean_check\holoocean_package.log"
function Log($m) { $line = "$(Get-Date -Format s) $m"; Add-Content -Path $log -Value $line; Write-Host $line }
$ue = "C:\Program Files\Epic Games\UE_5.3"
$proj = "C:\Users\pengb\HoloOcean\engine\Holodeck.uproject"
$archive = "C:\Users\pengb\uw_worlds_build"
Log "STEP package"
& "$ue\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="$proj" -noP4 -platform=Win64 -clientconfig=Development -build -cook -unversionedcookedcontent -stage -pak -archive -archivedirectory="$archive" -nocompileeditor -utf8output 2>&1 | ForEach-Object { Log $_ }
Log "package exit=$LASTEXITCODE"
$exe = Get-ChildItem "$archive" -Recurse -Filter "Holodeck.exe" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty FullName
Log ("Holodeck.exe found: " + ($exe -join " ; "))
$pkg = Join-Path $env:LOCALAPPDATA "holoocean\2.3.0\worlds\UwWorlds"
if (Test-Path "$archive\Windows") {
  Log "STEP assemble package at $pkg"
  if (Test-Path $pkg) { Remove-Item $pkg -Recurse -Force }
  New-Item -ItemType Directory -Path $pkg | Out-Null
  Copy-Item "$archive\Windows" -Destination "$pkg\Windows" -Recurse
  Copy-Item "C:\Users\pengb\uw_slam_holoocean_check\UwWorlds\config.json" -Destination "$pkg\config.json"
  Log ("package exe: " + (Test-Path "$pkg\Windows\Holodeck\Binaries\Win64\Holodeck.exe"))
  Log ("package size GB: " + [math]::Round(((Get-ChildItem $pkg -Recurse -File | Measure-Object Length -Sum).Sum / 1GB), 2))
}
Log "PACKAGE_DONE"
