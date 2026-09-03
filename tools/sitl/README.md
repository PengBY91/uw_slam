# ArduSub SITL（PREP-C-01）

到货前用 ArduPilot 的软件在环（SITL）代替真机 BlueROV2 Heavy，开发 `adapters/mavlink/`、参数调优流程、外部导航回灌，以及（PREP-A-05）与 HoloOcean 的物理桥接。

```bash
tools/sitl/setup_ardusub_sitl.sh        # 一次性：clone ArduPilot Sub-4.5 + 装依赖 + 编译 SITL（十几分钟）
tools/sitl/run_sitl.sh                  # 启动 SITL，MAVLink 在 tcp:127.0.0.1:5760（不起 MAVProxy）
tools/sitl/.venv/bin/python tools/sitl/sitl_smoke.py   # 验收：心跳、ATTITUDE、SCALED_PRESSURE2、切 DEPTH_HOLD

# PREP-A-05 的 JSON 外部物理后端（三个速率必须相等，见下）
SITL_RATE_HZ=50 ARDUSUB_MODEL=JSON tools/sitl/run_sitl.sh
```

**两个第 3 周实测发现（2026-09-03，见 `adapters/holoocean/docs/ardusub-sitl-bridge-feasibility.md`）：**

- **home 必须在海平面。** ArduSub 的 SITL 气压计是水压计（`AP_Baro_SITL` 走
  `SimpleUnderWaterAtmosphere(-altitude_amsl)`），深度直接取 AMSL 海拔的相反数。
  `sim_vehicle.py` 默认 home 是 CMAC（海拔 584 m），于是 ArduSub 认为自己在水面
  以上 581 m，`SCALED_PRESSURE2.press_abs` 报 −55925 hPa。`run_sitl.sh` 现在默认
  `--custom-location=-35.363261,149.165230,0,0`，用 `SITL_HOME` 可覆盖但**海拔要留 0**。
  上面 `sitl_smoke.py` 打印的 `press_abs` 在这个修正之前一直是那个负数，只是没有校验。
- **`SIM_RATE_HZ` 默认是 1200，不是 400。** `SITL.cpp` 的 `SIM_RATE_HZ_DEFAULT` 在
  SITL 构建下是 1200（JSON 后端 readme 里的 400 是 copter 板载值）。用外部物理后端时
  必须用 `SITL_RATE_HZ` 显式设定，并让它等于桥接的 `--step-hz` 和 HoloOcean 场景的
  `ticks_per_sec`——三者不等就是两个仿真器各自以为同步、时钟越走越开。

- 构建工具和 pymavlink 装在 `tools/sitl/.venv`，由 setup 脚本创建，不进 conda。**必须基于 Python 3.11**：Sub-4.5 自带的 waf 还在 `import imp`，3.12 起没有这个模块；本机用 uv 装的 `~/.local/bin/python3.11`（没有的话 `uv python install 3.11`），可用 `SITL_PYTHON=` 指定。
- 机架：`-f vectored_6dof`（BlueROV2 Heavy，ArduSub `FRAME_CONFIG=2`）。
- 物理后端：默认 SITL 内置；`ARDUSUB_MODEL=JSON tools/sitl/run_sitl.sh` 切到 JSON 外部物理接口（UDP 9002），供 PREP-A-05 的 HoloOcean 桥接使用。
- QGroundControl 在 Windows 侧：用 TCP 连 WSL2 的 IP（`hostname -I`）端口 5760；不起 MAVProxy 是因为它需要交互终端。
- 固件版本：`setup` 脚本锁 `Sub-4.5` 分支；整机到货后按实际固件版本改 `ARDUPILOT_BRANCH` 重跑。
- 坑：脚本强制系统 Python 在前并清掉 conda 变量（CLAUDE.md 里 colcon 同类问题）；apt 走 `/etc/apt/apt.conf.d/95proxy` 的代理。**上游 `install-prereqs-ubuntu.sh` 在 Ubuntu 24.04 上会因为找不到 `python-argparse` 包中断**，所以 setup 脚本自己装 apt 子集，Python 构建工具（`empy==3.3.4`、`pexpect`、`future`、`pymavlink`、`MAVProxy`、`dronecan`）装进 `tools/sitl/.venv`，waf 和 sim_vehicle.py 都用这个 venv 的 python 跑。
