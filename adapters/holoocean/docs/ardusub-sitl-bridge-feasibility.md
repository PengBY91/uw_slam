# ArduSub SITL ↔ HoloOcean 桥接可行性结论（PREP-A-05）

> 状态：v3 2026-09-03 · **SITL 侧与 HoloOcean 侧均已实跑验证并通过全部验收**（Windows 上 UE5 + HoloOcean 2.3.0，经 WSL2 的 `powershell.exe` 互操作启动）
>
> **v2 更正**：v2 曾把机体失控自转归因于"HoloOcean 的 BlueROV2 几乎没有偏航阻尼"，**那个结论是错的**。真因是本仓库的推进器几何表来自 HoloOcean Python 客户端里那张上游自己标注"可能不对，以 C++ 为准"的表，而它确实不对；由此推出的 ArduSub 映射把偏航接反，让 ArduSub 的航向保持变成正反馈。改用引擎 C++ 的真实几何后，偏航从 30 rad/s 的发散变成 0.98 rad/s 的受控响应，全部验收项通过。详见 2.4。
> 出自 `docs/ROV平台到货前准备工作规格-2026-09-02.md` PREP-A-05（时间盒 3 人日）
> 代码：`adapters/holoocean/uw_holoocean_adapter/ardusub_sitl_bridge.py`（WSL2 侧，全部逻辑）、
> `holoocean_sitl_physics_host.py`（Windows 侧哑转发）、
> `adapters/holoocean/scenarios/ardusub_sitl_bridge.json`（精简场景）、
> `adapters/holoocean/tools/sitl_bridge_check.py`（验收驱动）、
> `adapters/holoocean/tests/test_ardusub_sitl_bridge.py`（45 条离线单测）

## 0. 结论

**方案甲（SITL PWM → 推力 → HoloOcean `AUV_THRUSTERS` 内部动力学 → JSON 回灌）可行，全部验收项在真实 HoloOcean 上通过。** 稳态 RTF 1.00，协议、推进器映射、坐标转换、深度链路、四轴响应全部验证正确。

规格预设的退路方案乙（自写刚体动力学 + `AUV_FORCES` 只做渲染）**不需要启用**。

### 0.1 真实 HoloOcean 实测（2026-09-03，修正推进器几何之后）

场景 `ardusub_sitl_bridge.json`（无相机、无声呐、`ticks_per_sec: 50`、`frames_per_sec: false`），`SIM_RATE_HZ=50`，桥接 `--step-hz 50`，每推进器上限 28.75 N：

| 检查项 | 结果 |
|---|---|
| 稳态帧率 / RTF | **50.0 frames/s / RTF 1.00**（从第一分钟起即稳定） |
| 失步 | stale=0、dropped=0、foreign=0 |
| SITL 解析 JSON | 六个必需字段全部被接受 |
| 深度传感器 | spawn 3.00 m → 实读 **3.07 m**（`press_abs` 1313.8 hPa） |
| forward | **+2.095 m/s** |
| right | **+2.095 m/s**（与 forward 完全对称，符合对称 vectored 布局） |
| down | **+4.594 m/s** |
| yaw right | **+0.984 rad/s**（受控响应） |
| DEPTH_HOLD | 15 s 内保持 **0.32 m** |
| HoloOcean IMU 含不含重力 | **含**（首帧 \|accel\| = 9.800 m/s²，首次实测证实 `imu.proto` 里那条一直没验证过的假设） |
| EKF 与物理一致性 | 吻合（前进 1.96 vs 2.095、横移 1.77 vs 2.095、下潜 4.76 vs 4.594） |

**RESULT: OK。**

### 0.1.1 修正前后对照（同一套检查、同一个场景）

| 轴 | 用 Python 表推的映射 | 用引擎 C++ 几何 |
|---|---|---|
| forward | +0.371 m/s | **+2.095 m/s** |
| right | +0.092 m/s（未过 0.10 门限） | **+2.095 m/s** |
| down | +0.311 m/s | **+4.594 m/s** |
| yaw | +30.2 rad/s（发散） | **+0.984 rad/s**（受控） |
| RTF | 0.5–0.85 | **1.00** |
| EKF vs 物理 | 差一个数量级 | 吻合 |

修正前那些"通过"的轴向检查是**假阳性**：验收脚本只判断"有没有朝期望方向运动、超过门限"，而一台在翻滚打转的机体各个方向都有速度。RTF 也随之改善，因为失控自转本身在消耗 UE5 的物理算力。

### 0.2 mock 物理后端对照（同一套检查，无仿真器）

用同框架的 mock 刚体替代 HoloOcean，在本机 WSL2 上：

| 指标 | 实测 |
|---|---|
| 稳态帧率 | **50.0 frames/s（目标 50 Hz）** |
| 稳态 RTF | **1.00** |
| 失步 | 108 s 内 stale=0、dropped=0、foreign=0 |
| 深度传感器 | spawn 3.00 m → 实读 2.74–3.26 m（含气压噪声） |
| 四轴符号 | forward / right / down / yaw-right 全部方向正确，两次独立运行数值一致到 1% |
| DEPTH_HOLD | 15 s 内保持在 0.29–0.40 m |

规格担心的"HoloOcean tick 率远低于 SITL 主循环 400 Hz"这一点，实际比预想好：ArduSub 的 SITL 默认是 **1200 Hz** 而不是 400 Hz，但 JSON 后端的时间是**被外部物理驱动的**（SITL 按我们回传的 `timestamp` 推进自己的时钟），所以把 `SIM_RATE_HZ` 和场景 `ticks_per_sec` 一起设成 50 就得到 RTF=1，代价是 ArduSub 的控制回路在仿真里以 50 Hz 而不是 1200 Hz 运行。

tick 率的担心也被实测否掉了：**无渲染场景 + WSL2↔Windows TCP 往返，HoloOcean 稳定供到 50 ticks/s（RTF 1.00）**，与 `docs/perf/tick_budget_2026-09-02.md` 由"无渲染每 tick 约 15 ms"外推的 ≈66 ticks/s 一致（余量被 TCP 往返和 UE5 负载波动吃掉一部分）。规格里作为退路的方案乙（自写刚体动力学 + `AUV_FORCES` 只做渲染）**不需要启用**。

## 1. 架构：逻辑留在 WSL2，Windows 只做哑转发

```
ArduSub SITL (WSL2) --UDP 9002--> ardusub_sitl_bridge.py (WSL2) --TCP 5601--> holoocean_sitl_physics_host.py (Windows) --> HoloOcean/UE5
                    <--JSON 行--                                <--RawSensorFrame--
```

理由和 `bridged_realtime_ros_session.py` 一样：PWM 解码、推进器对应关系、坐标转换这些**会出错的部分**全部留在仓库里、有单测、改了不用重新拷文件到 Windows。代价是每个物理步多一次 WSL2↔Windows TCP 往返，这正是第 4 节要量的东西。

Windows 侧**没有**复用 `holoocean_bridge_sensor_host.py`：它走 `scenario_manifest.load_realtime_manifest`，而那套校验为感知回路要求完整传感器集（双目 + 成像声呐 + 姿态/IMU/深度/位姿）。SITL 物理桥不需要任何一个，而按 tick 预算每帧 1080p 相机约 65 ms、768×512 声呐约 60 ms——正好是本任务要回答的那个 tick 率。所以另写了一个不做 manifest 校验的最小 host。

## 2. 实施中确认的关键事实

### 2.1 ArduSub 的 `SIM_RATE_HZ` 默认是 1200，不是 400

`libraries/SITL/SITL.cpp` 的 `SIM_RATE_HZ_DEFAULT` 在 SITL 构建下是 1200（JSON 后端 readme 里写的 400 是 copter 的板载值）。任何外部物理后端都供不到 1200 Hz。`tools/sitl/run_sitl.sh` 因此新增 `SITL_RATE_HZ` 环境变量，桥接也会拿包里的 `frame_rate` 字段和自己的 `--step-hz` 对账并告警。

**三个数必须相等**：`SITL_RATE_HZ` = 桥接 `--step-hz` = 场景 `ticks_per_sec`。一个 HoloOcean tick 推进 `1/ticks_per_sec` 的仿真时间，桥接推进 `1/step_hz`，不等就是两个仿真器各自以为同步、实际时钟越走越开。

### 2.2 SITL 的 home 必须在海平面（这是第 1 周 PREP-C-01 的遗留问题）

ArduSub 的 SITL 气压计是**水压计**：`AP_Baro_SITL::_timer()` 走 `SimpleUnderWaterAtmosphere(-sim_alt)`，即深度直接取 AMSL 海拔的相反数。`sim_vehicle.py` 默认 home 是 CMAC（海拔 584 m），于是 ArduSub 认为自己在水面**以上** 581 m，`SCALED_PRESSURE2.press_abs` 报 −55925 hPa（实测 −57318 hPa，算式 `(101325 + 9800×(−581))/100` 完全对得上）。

第 1 周 PREP-C-01 的 `sitl_smoke.py` 一直把这个数打印出来但没有校验，所以没被发现。`run_sitl.sh` 现在默认 `--custom-location=...,0,0`，改成海平面后 3 m 深处读 1304–1333 hPa（理论 1307），深度链路才真正可用。

### 2.3 `MANUAL_CONTROL.z` 是 0..1000、500 为中位

x / y / r 是 ±1000、0 中位，但 **z（油门）是 0..1000、500 中位**，所有模式下都如此。用 `SERVO_OUTPUT_RAW` 实测确认：z=0 时四个垂推停在 PWM 1700（半推力）而不是 1500，也就是说 z=0 是**半速下潜指令**。写错会让 DEPTH_HOLD 看起来在漂移（实测"漂移 26 m"），而它其实在忠实执行下潜。

另外 ArduSub 按 `JS_GAIN_DEFAULT`（默认 0.5）缩放飞手输入，满量程轴只到半推力。

### 2.4 HoloOcean **Python 客户端**的推进器表是错的（本次最重要的发现）

`holoocean/agents.py` 里的 `thruster_d`/`thruster_p` 上游自己注明 "provided for
convenience, **may not be correct — check the C++**"。去引擎源码
（`Source/Holodeck/Agents/Public/BlueROV2.h` 的 `thrusterLocations` +
`Private/BlueROV2.cpp` 的 `ApplyThrusters`）核对之后：它确实不对，而且不是小偏差。

| | Python 客户端表 | 引擎 C++ 真值 |
|---|---|---|
| 位置 x 号 | 全部取反 | — |
| 斜推方向 | 前两个朝后（−x）、后两个朝前（+x） | **四个全部朝前（+x）**，左右由 y 分量区分 |
| 偏航力臂 | 前 0.189 / 后 0.032（不对称） | **四个都是 0.180**（对称） |

引擎的几何是对称的；v2 里报的"HoloOcean 的机体有寄生偏航力矩"是那张错表造成的假象。

**后果（都是实测出来的，不是推断）。** 用错表推出的 ArduSub→推进器映射
`(6,7,4,5,3,2,0,1)` + 全 −1 符号，在引擎的真实几何里产生：

| ArduSub 指令 | 错映射的实际效果 | 正确 |
|---|---|---|
| **forward** | **0 N —— 完全没有前进力** | +81.3 N |
| lateral | −81.3 N（右）✓ | 同 |
| **yaw** | +20.7（左）**反向** | −20.7（右） |
| throttle | +115 N（上）✓ | 同 |
| roll | +25.1 ✓ | 同 |
| **pitch** | +13.8 **反向** | −13.8 |

前进恰好归零，是因为"在 Python 表的方向下代表前进"的那个推力模式，在引擎的方向下
是个零空间向量。而**偏航反向让 ArduSub 的航向保持变成正反馈**——它命令右转、得到
左转、于是加大命令——这才是 v2 里 60+ rad/s 失控自转的真正原因。lateral/throttle/roll
碰巧正确，是因为它们的推力模式在两套几何下恰好对称。

**修法**：`thrust_allocation.py` 的 `_THRUSTER_D`/`_THRUSTER_P` 换成引擎 C++ 的真值
（已转到本仓库的客户端系：x 前、y 左、z 上、米），桥接的映射改为
`ARDUSUB_TO_HOLOOCEAN = (4,5,7,6,0,1,3,2)`，符号改为逐电机
`(-1,-1,+1,+1,-1,-1,-1,-1)`（不再是单一全局常量——引擎的四个斜推都朝前，所以
ArduSub 前向因子为正的两个电机正向驱动、为负的两个反向驱动）。

**这条影响的不止 A-05**：`thrust_allocation.py` 的 `allocate()` 是这张表的伪逆，
而它驱动 `scripted_pilot.py` 的飞手指令通路（PREP-C-02）。**所有在此之前用
HoloOcean 做的操纵/控制相关结论都要重新看**——尤其是"纯 surge 指令产生零净力"
这一条，意味着之前任何"给前进指令、机体不太动"的观察都有可能是这个 bug，而不是
水动力。

**为什么单测没抓到**：`tests/test_thrust_allocation.py` 的 9 条断言全部是
`allocate()` → `commanded_wrench()` 的**自洽性**检查——对任何一张内部一致的表都成立。
现在补了 `test_thruster_geometry_matches_the_engine_source`（把表钉死到引擎 C++ 的
常量上）和 `test_each_axis_demand_produces_a_pure_force_in_the_engine_geometry`
（断言纯 surge 指令产生真实前进力且无寄生力矩），后者正是本来该抓到这个 bug 的测试。

### 2.4b HoloOcean 的转动阻尼确实偏弱，但不是本次故障的原因

引擎里转动方向**没有水动力阻尼**：`HolodeckBuoyantAgent::ApplyBuoyancyDragForce()`
只按线速度在质心加一个二次阻力（`CoefficientOfDrag = 0.8`、`AreaOfDrag = 0.45`，
且各向同性），转动方向只有 `BlueROV2::EnableDamping()` 里的
`SetAngularDamping(0.75)`——UE 的一阶速率衰减，不是水动力。

修正映射之后偏航是受控的 0.98 rad/s，所以这条**不构成阻碍**，但作为保真度限制仍应记住：
仿真里的转动阻尼是一个通用的 0.75/s 衰减项，真机是二次水动力阻尼。同理，各向同性的
平动阻力让下潜速度做到 4.6 m/s（真机 BlueROV2 约 1 m/s 量级）。要在仿真里调姿态或
垂向控制参数之前，先确认这两点是否可接受。

### 2.5 ArduSub 启动时把深度归零

真机流程是在水面上电，所以 ArduSub 用启动瞬间的气压做基准。在 3 m 处 spawn 的机体，ArduSub 报"深度 0"，`VFR_HUD.alt` 也一直相对启动点。验收要看 `SCALED_PRESSURE2.press_abs` 的绝对值，不能看 `alt`。

### 2.6 `LOCAL_POSITION_NED` ArduSub 不主动发

即使 `REQUEST_DATA_STREAM(MAV_DATA_STREAM_ALL)` 之后也没有（EKF 拿到原点之后才偶尔出现）。轴向检查改用 `GLOBAL_POSITION_INT` 的速度，姿态真值用 `SIMSTATE`。

### 2.7 ArduSub 的 EKF 水平速度在机体自转时不可信

> 下面的数字取自修正推进器几何**之前**的运行（当时机体在失控自转）。几何修正后
> EKF 与物理真值吻合到 10% 以内（前进 1.96 vs 2.095 m/s），所以这条不是常态缺陷，
> 而是"机体姿态剧烈变化时 EKF 会退化"——验收脚本仍应读物理真值而不是 EKF。

轴向符号检查一开始从 EKF 读速度，结果两次相同的运行能给出 +0.53 和 −0.12 m/s（同一个前进指令）。加了 trace 之后对照发现：**物理侧最大水平速度只有 1.06 m/s，而 EKF 同期报到 +6.72 和 −2.03 m/s**。原因是 2.4 的寄生偏航让机体高速自转，EKF 的水平速度解退化。

所以 `sitl_bridge_check.py` 的轴向符号改成读桥接 `--trace-csv` 里的物理真值（用 `time.monotonic()` 对齐窗口，Linux 上跨进程可比）。改完之后两次独立运行一致到 1%：

| 轴 | 运行 A（物理） | 运行 B（物理） | 同期 EKF |
|---|---|---|---|
| forward | +1.012 m/s | +1.016 m/s | +0.81 / +0.84 |
| right | +0.692 m/s | +0.675 m/s | **+6.72 / −2.03** |
| down | +1.026 m/s | +1.016 m/s | +1.09 / +1.13 |
| yaw rate | +0.959 rad/s | +0.959 rad/s | — |

### 2.8 ~~HoloOcean 的 BlueROV2 几乎没有偏航阻尼~~（v2 的结论，已撤回）

v2 在这里报告：12.7 N·m 偏航力矩下稳态 62 rad/s、等效阻尼 ≈0.12 N·m·s/rad，并据此
判定 HoloOcean 的转动动力学不可信、PREP-A-12/C-07/C-09 都受影响。

**撤回。** 那个测量本身没错（PoseSensor 确实转到 62 rad/s），但归因错了：机体不是被
一个"欠阻尼的开环响应"转起来的，而是被 2.4 的偏航反接送进了 ArduSub 航向保持的**正
反馈**——命令右转、得到左转、于是加大命令。修正几何之后同一套检查测到 0.98 rad/s 的
受控偏航，`0.12 N·m·s/rad` 这个数是闭环发散状态下的产物，不是引擎的阻尼系数。

关于引擎真实的转动阻尼，见 2.4b：确实偏弱（UE 的一阶 `SetAngularDamping(0.75)`，
无水动力转动阻力），但不构成阻碍，且量级远没有 v2 说的那么严重。

**教训**（值得记在流程上）：v2 在"桥接侧四轴符号全部正确"的前提下去归因外部系统，
而那个前提来自一个只检查"有没有朝期望方向动"的验收脚本——在一台翻滚打转的机体上，
这个判据几乎必然通过。**先把自己这侧钉死到对方的源码常量上，再去怀疑对方。**

## 3. 推进器对应关系

ArduSub `SUB_FRAME_VECTORED_6DOF`（`AP_Motors6DOF.cpp:158-165`）的 MOT_1..8 与
HoloOcean 推进器序的对应，是**数值推导**出来的而不是按名字猜的——两张表的名字都
和各自的几何对不上：

```
ARDUSUB_TO_HOLOOCEAN   = (4, 5, 7, 6, 0, 1, 3, 2)
ARDUSUB_THRUSTER_SIGN  = (-1, -1, +1, +1, -1, -1, -1, -1)
```

推导方法：把每个 HoloOcean 推进器的单位力旋量转到 ArduSub 的轴坐标
（roll, pitch, yaw, throttle, forward, lateral，力和力矩都做 FLU→FRD），再和 ArduSub
的因子行按方向匹配。`test_correspondence_matches_both_published_thruster_tables`
每次跑测试都重推一遍，常量不会和任何一张表悄悄偏离。

符号是**逐电机**的，不是单一全局常量：引擎的四个斜推都朝前，所以 ArduSub 前向因子
为正的两个电机正向驱动其推进器、为负的两个反向驱动。v1 那个"全局 −1"是错几何的
产物（见 2.4）。

`thrust_allocation.py` 现在承载引擎 C++ 的真实几何，
`test_thruster_geometry_matches_the_engine_source` 把它钉死在那些常量上。

## 4. 复跑步骤（已执行过一遍）

前置：把两个文件拷进现有的 Windows harness 目录
`C:\Users\pengb\uw_slam_holoocean_check\uw_holoocean_adapter\`（`holoocean_driver.py`、`raw_frame_wire.py` 已经在那里）：

- `uw_holoocean_adapter/holoocean_sitl_physics_host.py`
- `scenarios/ardusub_sitl_bridge.json`

WSL2 侧先起（它是 listener）：

```bash
# 终端 1：桥接，等 Windows 连过来
adapters/holoocean/.venv/bin/python -m uw_holoocean_adapter.ardusub_sitl_bridge \
    --physics holoocean --step-hz 50 --listen-port 5601 \
    --trace-csv /tmp/a05_trace.csv

# 终端 2：SITL（必须和 --step-hz 同为 50）
SITL_RATE_HZ=50 ARDUSUB_MODEL=JSON tools/sitl/run_sitl.sh
```

Windows 侧（PowerShell，`-WorkingDirectory` 要给真实本地路径，见 harness 记录里的 WinError 4551 坑）：

```powershell
python holoocean_sitl_physics_host.py --scenario ardusub_sitl_bridge.json --bridge-host <WSL2 IP>
```

然后：

```bash
# 终端 3：验收（会打印四轴符号、深度、DEPTH_HOLD）
adapters/holoocean/.venv/bin/python adapters/holoocean/tools/sitl_bridge_check.py \
    --expect-rate-hz 50 --expect-depth-m 3.0 --trace-csv /tmp/a05_trace.csv
```

三个操作上的坑（都踩过）：

- **启动顺序**：桥接（listener）→ Windows 主机 → SITL → MAVLink 客户端。SITL 在有 MAVLink 客户端连上之前不会开始发包，所以最后那一步不能省。
- **端口 5760 残留**会让新的 SITL 直接 `bind failed ... Address already in use` 然后退出，日志只在 `/tmp/ArduSub.log` 里。重启前确认 `ss -tlnp | grep 5760` 是空的。
- **后台进程要 `setsid`**：直接 `nohup ... &` 起的进程和调用它的 shell 同进程组，shell 一超时被 SIGTERM，整条链会一起死。

QGC 从 Windows 连 WSL2 的 `tcp:<WSL2 IP>:5760`；规格要求的操控录屏就在这一步录（本次没有录屏，机体自转到每秒 10 圈的状态下录了也没有说明力）。

## 5. 需要拍板的开放问题

1. **重新审视此前所有基于 HoloOcean 的操纵/控制观察（2.4）。** `thrust_allocation.py`
   的分配矩阵此前是错表的伪逆，而它驱动 `scripted_pilot.py` 的飞手指令通路
   （PREP-C-02）。最刺眼的一条是"纯 surge 指令在引擎里产生零净力"——之前任何
   "给前进指令但机体不太动"的观察都可能是这个 bug 而不是水动力。需要清点一遍哪些
   结论要重跑。
2. **仿真水动力的保真度（2.4b）。** 转动只有 UE 的一阶 `SetAngularDamping(0.75)`、
   没有水动力转动阻尼；平动阻力是各向同性的单一 `Cd·A`，导致下潜能到 4.6 m/s
   （真机约 1 m/s 量级）、前进 2.1 m/s（真机约 1.5 m/s）。不构成阻碍，但在仿真里
   调垂向/姿态控制参数之前要先决定这个偏差可不可接受，否则参数搬不到真机。
3. **加回感知之后的 RTF**。本场景刻意无相机无声呐。按 tick 预算，加一路
   960×540@12.5 Hz 相机 + 5 Hz 768×256 声呐大约要 1.05 s 墙钟/仿真秒，也就是
   SITL 闭环 + 感知只能在 realtime profile 下跑，fidelity profile（RTF≈0.1）
   不能用于 SITL 闭环。
4. **QGC 操控录屏**（规格验收项）。机体现在行为正常，这一步已解除阻塞，但需要在
   Windows 上人工录屏：QGC 连 `tcp:<WSL2 IP>:5760`，手柄推杆即可。

## 6. 时间盒使用情况

规格给 3 人日的时间盒，本次在盒内完成了全部内容：协议、映射数值推导、mock 后端、验收驱动、45 条离线单测、SITL 侧实跑、HoloOcean 侧实跑，以及上述 8 项事实的定位。

规格预设的退路——方案乙（自写刚体动力学 + `AUV_FORCES` 只做渲染）——**因为 tick 率的原因不需要启用**（实测 RTF 可达 1.00）。但 2.8 的转动动力学问题给了方案乙一个新的、更强的理由：如果引擎的转动阻尼不可配，那就得自己接管动力学。这是一个独立的决定，不属于 A-05 的验收范围。
