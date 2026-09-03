# ArduSub SITL（PREP-C-01）

到货前用 ArduPilot 的软件在环（SITL）代替真机 BlueROV2 Heavy，开发 `adapters/mavlink/`、参数调优流程、外部导航回灌，以及（PREP-A-05）与 HoloOcean 的物理桥接。

```bash
tools/sitl/setup_ardusub_sitl.sh        # 一次性：clone ArduPilot Sub-4.5 + 装依赖 + 编译 SITL（十几分钟）
tools/sitl/run_sitl.sh                  # 启动 SITL，MAVLink 在 tcp:127.0.0.1:5760（不起 MAVProxy）
tools/sitl/.venv/bin/python tools/sitl/sitl_smoke.py   # 验收：心跳、ATTITUDE、SCALED_PRESSURE2、切 DEPTH_HOLD
```

- 构建工具和 pymavlink 装在 `tools/sitl/.venv`，由 setup 脚本创建，不进 conda。**必须基于 Python 3.11**：Sub-4.5 自带的 waf 还在 `import imp`，3.12 起没有这个模块；本机用 uv 装的 `~/.local/bin/python3.11`（没有的话 `uv python install 3.11`），可用 `SITL_PYTHON=` 指定。
- 机架：`-f vectored_6dof`（BlueROV2 Heavy，ArduSub `FRAME_CONFIG=2`）。
- 物理后端：默认 SITL 内置；`ARDUSUB_MODEL=JSON tools/sitl/run_sitl.sh` 切到 JSON 外部物理接口（UDP 9002），供 PREP-A-05 的 HoloOcean 桥接使用。
- QGroundControl 在 Windows 侧：用 TCP 连 WSL2 的 IP（`hostname -I`）端口 5760；不起 MAVProxy 是因为它需要交互终端。
- 固件版本：`setup` 脚本锁 `Sub-4.5` 分支；整机到货后按实际固件版本改 `ARDUPILOT_BRANCH` 重跑。
- 坑：脚本强制系统 Python 在前并清掉 conda 变量（CLAUDE.md 里 colcon 同类问题）；apt 走 `/etc/apt/apt.conf.d/95proxy` 的代理。**上游 `install-prereqs-ubuntu.sh` 在 Ubuntu 24.04 上会因为找不到 `python-argparse` 包中断**，所以 setup 脚本自己装 apt 子集，Python 构建工具（`empy==3.3.4`、`pexpect`、`future`、`pymavlink`、`MAVProxy`、`dronecan`）装进 `tools/sitl/.venv`，waf 和 sim_vehicle.py 都用这个 venv 的 python 跑。
