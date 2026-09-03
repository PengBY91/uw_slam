# UE 5.3 工程与自定义世界打包（PREP-A-02）

> 状态：2026-09-02 决定投入。本页记录从零到"HoloOcean 能加载我们自己打包的世界"的全部步骤，
> 哪些已自动化、哪些必须在 Windows 桌面上手动做，以及每步的验证方法。
> 依据：HoloOcean 客户端源码 `docs/develop/start.rst`、`env-docs/create-env.rst`、`env-docs/package-env.rst`
> （本机 `C:\Users\pengb\holoocean_client\docs\develop\`）。

## 0. 本机现状（2026-09-02 探测）

- Windows 11 家庭版 10.0.26200，RTX 4070 Laptop 8 GB，C 盘剩余 370 GB。
- HoloOcean 2.3.0 Python 客户端 + 官方 `Ocean` 世界包（6.2 GB，十个关卡，含 `Tank`/`PierHarbor`/`Dam`）。
- **无** Unreal Engine、**无** Visual Studio、**无** Epic Games Launcher、**无** 引擎工程源码。
- 官方文档明确：公开的 `byu-holoocean/HoloOcean` 引擎工程只含 `ExampleLevel`；`PierHarbor`、`Dam` 等用了购买资产，仅 BYU 实验室成员可得。所以我们打包出来的是一个**只含自建关卡的新世界包**，与官方 `Ocean` 包并存，官方关卡不能改也不能混入。

## 1. 自动化部分（已由 `uw_install_ue_prereqs.ps1` 执行）

脚本位置：`C:\Users\pengb\uw_slam_holoocean_check\uw_install_ue_prereqs.ps1`，日志 `ue_prereqs_install.log`。每个 winget 安装会在 Windows 桌面弹 UAC 提示，需要点"是"。

1. `winget install EpicGames.EpicGamesLauncher`
2. `winget install Microsoft.VisualStudio.2022.Community`，工作负载：`NativeGame`（使用 C++ 的游戏开发）+ `NativeDesktop`，组件：`VC.14.38.17.8`（UE 5.3 对应的 MSVC 工具集，5.3 与 14.39 及以上有已知兼容问题）+ `Windows11SDK.22621`。约 10 GB 下载，20–40 分钟。

验证：`vswhere.exe -products * -property displayName` 列出 Community；`C:\Program Files\Epic Games\Launcher\Portal\Binaries\Win64\EpicGamesLauncher.exe` 存在（2026-09-02 实装路径，不在 x86 目录）。已装：Launcher 1.3.193.0；VS Community 2022 含 NativeGame/NativeDesktop 工作负载，MSVC 14.38.33130 与 14.44.35207。

## 2. 手动部分（Windows 桌面，约 1–2 小时含下载）

> 2026-09-03 进度：第 1–3 步已完成并从 WSL2 验证——`C:\Program Files\Epic Games\UE_5.3` 为 5.3.2，含引擎源码，`UnrealEditor.exe` 与 `RunUAT.bat` 在位；Git for Windows 2.55；`git ls-remote` 已能读到 `byu-holoocean/HoloOcean` 私有仓库（GitHub↔Epic 绑定生效）。第 4 步改为脚本 `uw_clone_holoocean.ps1` 自动克隆（优先 2.3.x tag，否则 develop）；第 5–6 步（编译、烘焙）与第 3 节打包改为用 UBT/UAT 无头执行，见 `uw_build_holoocean.ps1`，不需要打开编辑器。

1. **Epic 账号**：打开 Epic Games Launcher，登录（没有就注册）。
2. **安装 UE 5.3**：Launcher → Unreal Engine → 库 → "引擎版本" 旁的 "+" → 选 **5.3.x**（不要选 5.4/5.5，HoloOcean 开发文档钉在 5.3）→ 安装到默认 `C:\Program Files\Epic Games\UE_5.3`。约 40–50 GB，可勾掉"调试符号"和不需要的目标平台节省空间。
3. **绑定 GitHub**：unrealengine.com → 账号 → 连接（Connections）→ GitHub → 授权；接受 Epic 发来的 GitHub 组织邀请（邮件）。这一步是访问 `byu-holoocean/HoloOcean` 私有仓库的前提。上次在 WSL2 沙箱里 SSH 22 端口被代理挡住、HTTPS 报 404，都是没绑定的表现。
4. **克隆引擎工程**（在 Windows 上用 Git for Windows 或 GitHub Desktop，不要在 WSL2 里）：
   ```
   git clone https://github.com/byu-holoocean/HoloOcean.git C:\Users\pengb\HoloOcean
   cd C:\Users\pengb\HoloOcean
   git checkout develop     # 或与客户端 2.3.0 对应的 tag
   pip install -e client/   # 用现在跑 HoloOcean 的那个 Python 3.11.9
   ```
5. **打开工程**：双击 `HoloOcean\engine\holodeck.uproject`，引擎版本对话框选 5.3；提示"模块缺失或引擎版本不同"时选"是"重新编译（首次几分钟到十几分钟）。报错按 `docs/develop/troubleshooting.rst` 处理。
6. **烘焙**：编辑器 → Platforms → Windows → Cook Content，成功弹窗即可。

### 2.1 无头编译时踩到的两个坑（2026-09-03）

- **Live Coding 占用**：只要有任何 UnrealEditor 实例在跑（哪怕只是项目选择界面），UnrealBuildTool 会报 `Unable to build while Live Coding is active` 直接退出。编译前关掉编辑器。
- **MSVC 版本**：UBT 默认挑最新的 14.44 工具集，UE 5.3 的引擎头文件（`ConcurrentLinearAllocator.h`：`__has_feature` 未定义，C4668/C4067）编不过。已在 `%APPDATA%\Unreal Engine\UnrealBuildTool\BuildConfiguration.xml` 里把 `WindowsPlatform/CompilerVersion` 钉到 `14.38.33130`。这个文件是按用户生效的，换机器要重建。

- **Smart App Control 拦截自编译 DLL**（2026-09-03，最关键的一条）：`HolodeckEditor` 编译成功后，烘焙时 `UnrealEditor-Cmd` 报 `Failed to load 'UnrealEditor-Holodeck.dll' (GetLastError=4551)`，弹窗"游戏模块 Holodeck 无法被加载"。根因是 Windows 11 家庭版的 Smart App Control 处于开启状态（注册表 `HKLM\SYSTEM\CurrentControlSet\Control\CI\Policy\VerifiedAndReputablePolicyState = 1`），它只放行有签名且有云端信誉的二进制，任何本地编译的 DLL 都会被拒。没有脚本化开关，也没有自签名绕过。**必须手动关闭**：设置 → 隐私和安全性 → Windows 安全中心 → 应用和浏览器控制 → 智能应用控制设置 → 关闭。注意：**关闭后无法再次开启，除非重装 Windows**，这是微软的设计。关闭后重新跑 `uw_build_holoocean.ps1`（编译会命中缓存，直接进烘焙）。

## 3. 冒烟关卡与打包（PREP-A-02 的验收）

> 2026-09-03 进度：烘焙已通过（3958 个包，19 分钟）；冒烟关卡 `UwSmokeLevel` 已由 `tools/ue5/make_smoke_level.py` 无头生成（15 个 actor：海床与四面墙、水面 `WaterSurface`、`WaterPPV_Tint`/`WaterPPV_Fog`、Sun/SkyLight/SkyAtmosphere/VolumetricCloud/HeightFog、`PilingTarget` 圆柱、PlayerStart）；打包与 `UwWorlds` 包组装由 `tools/ue5/package_holoocean.ps1` 执行；验收脚本 `tools/ue5/smoke_world_test.py`。脚本全部收进 `adapters/holoocean/tools/ue5/`。

1. 新建关卡 `UwSmokeLevel`：按 `create-env.rst`——地形 z<0；z=0 放一个 Plane 作水面，材质任意水材质，碰撞设为全部忽略，标签 `WaterSurface`，设为可移动；两个 PostProcessVolume 覆盖整个水下区域，标签都是 `WaterPPV`，第一个调 Scene Color Tint（Dam 用的是 `AAD9C8FF`），第二个挂 `Content/WeatherContent/Fog/MM_Fog_Water_Simple` 调 `Fog_Depth`/`Fog_Opacity`/`Fog_Color`；灯光用 Env. Light Mixer 建齐 DirectionalLight、ExponentialHeightFog（两个 Inscattering Color 设黑）、SkyAtmosphere、SkyLight、VolumetricCloud。放一根圆柱当声呐目标。**不要用 UE 自带 Water 插件**，与 HoloOcean 不兼容。
2. Platforms → Windows → Package Project，输出到 `C:\Users\pengb\uw_worlds_build\`。
3. 按 `package-env.rst` 的目录结构组织成 HoloOcean 包（`config.json` 里 `name` 用 `UwWorlds`，`worlds` 列出 `UwSmokeLevel` 及其场景），压缩成 zip，用 `holoocean.packagemanager.install("UwWorlds", url="file:///C:/Users/pengb/uw_worlds_build/UwWorlds.zip")` 安装，或直接放到 `%LOCALAPPDATA%\holoocean\2.3.0\worlds\UwWorlds\`。
4. 验收：`holoocean.make(scenario_cfg={... "package_name": "UwWorlds", "world": "UwSmokeLevel", agent BlueROV2 + ImagingSonar ...})` 能起来并拿到非全零声呐帧；`packagemanager.installed_packages()` 同时列出 `Ocean` 和 `UwWorlds`。
   **2026-09-03 实跑结果（`tools/ue5/smoke_world_test.py`）：通过。** `installed packages: ['Ocean', 'UwWorlds']`，环境 10 s 起来，声呐帧 (512, 768) 非零像素 16–17%、峰值 0.47，相机帧到达，150 tick 后位姿 x=2.23 m（推力生效）。增量打包 2 分钟。

## 4. 之后

- 养殖区（PREP-A-06）、结构物（A-07）、水池（A-08）三个关卡都建在这个工程里，同一个 `UwWorlds` 包发布，版本号跟着 `uw_metadata.manifest_version` 走。
- 资产来源：Fab 上 Quixel 免费材质；网衣、浮球、缆绳需要自建或购买，购买的资产不能进 git，只进打包产物。
- 工程本身（不含购买资产）建议放团队私有 git，提交 `.uproject`、`Content/UwWorlds/**`（自建资产）、`Config/`，忽略 `Binaries/`、`Intermediate/`、`Saved/`、`DerivedDataCache/`。
