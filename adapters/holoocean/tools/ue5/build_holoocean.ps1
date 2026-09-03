# PREP-A-02 step 5/6: build the HoloOcean engine project headlessly with
# UE 5.3.2's UnrealBuildTool (no editor GUI), then cook content. Target names
# are discovered from engine\Source\*.Target.cs. Logs to holoocean_build.log.
$ErrorActionPreference = "Continue"
$log = "C:\Users\pengb\uw_slam_holoocean_check\holoocean_build.log"
function Log($m) { $line = "$(Get-Date -Format s) $m"; Add-Content -Path $log -Value $line; Write-Host $line }
$ue = "C:\Program Files\Epic Games\UE_5.3"
$proj = "C:\Users\pengb\HoloOcean\engine\Holodeck.uproject"
if (-not (Test-Path $proj)) { Log "no uproject at $proj"; Log "BUILD_DONE fail"; exit 1 }
$targets = Get-ChildItem "C:\Users\pengb\HoloOcean\engine\Source" -Filter "*.Target.cs" -Recurse | ForEach-Object { $_.BaseName -replace "\.Target$", "" }
Log ("targets: " + ($targets -join ", "))
$editorTarget = $targets | Where-Object { $_ -match "Editor$" } | Select-Object -First 1
$gameTarget = $targets | Where-Object { $_ -notmatch "Editor$" } | Select-Object -First 1
Log "editor target=$editorTarget game target=$gameTarget"
Log "STEP generate project files"
& "$ue\Engine\Build\BatchFiles\Build.bat" -projectfiles -project="$proj" -game -engine -progress 2>&1 | ForEach-Object { Log $_ }
Log "STEP build $editorTarget Win64 Development"
& "$ue\Engine\Build\BatchFiles\Build.bat" $editorTarget Win64 Development -Project="$proj" 2>&1 | ForEach-Object { Log $_ }
Log "build editor exit=$LASTEXITCODE"
Log "STEP cook Windows"
& "$ue\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun -project="$proj" -noP4 -platform=Win64 -clientconfig=Development -cook -unversionedcookedcontent -skipstage -nocompileeditor 2>&1 | ForEach-Object { Log $_ }
Log "cook exit=$LASTEXITCODE"
Log "BUILD_DONE"
