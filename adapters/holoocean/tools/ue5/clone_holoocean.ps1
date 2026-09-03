$ErrorActionPreference = "Continue"
$log = "C:\Users\pengb\uw_slam_holoocean_check\holoocean_clone.log"
function Log($m) { $line = "$(Get-Date -Format s) $m"; Add-Content -Path $log -Value $line; Write-Host $line }
$env:GIT_TERMINAL_PROMPT = "0"
$repo = "https://github.com/byu-holoocean/HoloOcean.git"
$dest = "C:\Users\pengb\HoloOcean"
$tags = git ls-remote --tags $repo 2>&1 | ForEach-Object { ($_ -split "`t")[1] } | Where-Object { $_ -match "2\.3" }
Log ("tags matching 2.3: " + ($tags -join ", "))
$ref = "develop"
foreach ($cand in @("refs/tags/v2.3.0", "refs/tags/2.3.0", "refs/tags/v2.3.0-release")) {
  if ($tags -contains $cand) { $ref = $cand -replace "refs/tags/", ""; break }
}
Log "cloning ref=$ref into $dest"
git clone --branch $ref --depth 1 --recurse-submodules --shallow-submodules $repo $dest 2>&1 | ForEach-Object { Log $_ }
Log "clone exit=$LASTEXITCODE"
if (Test-Path "$dest\.gitattributes") { Log ("gitattributes: " + ((Get-Content "$dest\.gitattributes" | Select-Object -First 5) -join " | ")) }
Log ("uproject: " + (Test-Path "$dest\engine\holodeck.uproject"))
if (Test-Path "$dest\engine\holodeck.uproject") { Log ("uproject content: " + (Get-Content "$dest\engine\holodeck.uproject" -Raw)) }
Log ("top-level: " + ((Get-ChildItem $dest | Select-Object -ExpandProperty Name) -join ", "))
Log ("engine dir: " + ((Get-ChildItem "$dest\engine" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Name) -join ", "))
Log ("size GB: " + [math]::Round(((Get-ChildItem $dest -Recurse -File | Measure-Object Length -Sum).Sum / 1GB), 2))
Log "CLONE_DONE"
