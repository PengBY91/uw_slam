# PREP-A-02 step 2 (2026-09-02): install the two prerequisites that CAN be
# scripted -- Epic Games Launcher and Visual Studio 2022 Community with the
# "Game development with C++" workload and the MSVC 14.38 toolset UE 5.3
# expects. UE 5.3 itself and the Epic<->GitHub account link are GUI/manual
# steps (see tools/sitl/../adapters/holoocean/docs/ue5-world-packaging.md).
# Each winget call triggers a UAC prompt on the Windows desktop.
$log = "C:\Users\pengb\uw_slam_holoocean_check\ue_prereqs_install.log"
function Log($m) { $line = "$(Get-Date -Format s) $m"; Add-Content -Path $log -Value $line; Write-Host $line }
Log "START epic launcher"
winget install --id EpicGames.EpicGamesLauncher --exact --silent --accept-package-agreements --accept-source-agreements 2>&1 | ForEach-Object { Log $_ }
Log "EXIT epic launcher code=$LASTEXITCODE"
Log "START visual studio 2022 community (NativeGame + NativeDesktop + MSVC 14.38)"
winget install --id Microsoft.VisualStudio.2022.Community --exact --accept-package-agreements --accept-source-agreements --override "--passive --wait --norestart --add Microsoft.VisualStudio.Workload.NativeGame --add Microsoft.VisualStudio.Workload.NativeDesktop --add Microsoft.VisualStudio.Component.VC.14.38.17.8.x86.x64 --add Microsoft.VisualStudio.Component.Windows11SDK.22621 --includeRecommended" 2>&1 | ForEach-Object { Log $_ }
Log "EXIT visual studio code=$LASTEXITCODE"
Log "DONE"
