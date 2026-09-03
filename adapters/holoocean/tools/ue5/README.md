# UE 5.3 / HoloOcean 引擎工程自动化脚本（PREP-A-02）

这些脚本在 **Windows 侧**运行（从 WSL2 经 `powershell.exe` 调用，或直接在 PowerShell 里运行），
是 `adapters/holoocean/docs/ue5-world-packaging.md` 各步骤的可复现实现。路径按本机
（`C:\Users\pengb\...`）硬编码，换机器改脚本顶部的变量。

| 脚本 | 作用 |
|---|---|
| `install_ue_prereqs.ps1` | winget 安装 Epic Games Launcher 与 VS2022 Community（NativeGame/NativeDesktop 工作负载、MSVC 14.38、Win11 SDK） |
| `BuildConfiguration.xml` | 放到 `%APPDATA%\Unreal Engine\UnrealBuildTool\`，把 UBT 钉在 MSVC 14.38（UE 5.3 编不过 14.44） |
| `clone_holoocean.ps1` | 克隆 `byu-holoocean/HoloOcean`（优先 `v2.3.0` tag），需要 GitHub 已绑定 Epic |
| `build_holoocean.ps1` | UBT 无头编译 `HolodeckEditor` + UAT 烘焙。编译前关闭所有 UnrealEditor 实例；Smart App Control 必须已关闭 |
| `make_smoke_level.py` | UE Python 编辑器脚本，无头生成 `/Game/UwSmokeLevel`（水面、两个 WaterPPV、灯光、海床、圆柱目标）。用 `UnrealEditor-Cmd.exe <uproject> -run=pythonscript -script=<本文件> -unattended -nullrhi` 执行 |
| `py_probe.py` | 验证无头 Python 编辑器脚本可用 |
| `package_holoocean.ps1` | UAT `BuildCookRun -build -cook -stage -pak -archive` 到 `C:\Users\pengb\uw_worlds_build`，再组装成 `%LOCALAPPDATA%\holoocean\2.3.0\worlds\UwWorlds\` |
| `UwWorlds.config.json` | 包描述：`path` 指向包内 `Holodeck.exe`，`worlds` 列出关卡与 `env_min/env_max`（声呐八叉树边界） |
| `smoke_world_test.py` | 验收：用未改动的 HoloOcean 2.3.0 客户端加载 `UwWorlds/UwSmokeLevel`，BlueROV2 + 声呐 + 相机，检查声呐帧非零 |

执行顺序：prereqs → BuildConfiguration.xml → clone → build → make_smoke_level → package → smoke_world_test。
