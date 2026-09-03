# tools/calib — IMU 噪声参数标定（PREP-B-05）

`imu_allan.py`：从一段静置 IMU 录制（MCAP，`/raw/imu` 上的 `uw.domain.ImuSample`）算 Allan 偏差，直接输出 rig 文件的 `imu_noise` 块。单测 `adapters/holoocean/tests/test_imu_allan.py` 用已知参数的合成流验证：白噪声密度反推误差 <20%，偏置随机游走 <30%。

## 运行

脚本依赖 numpy / mcap / protobuf 和生成的 `schema_pb2`，所以用 `adapters/holoocean/.venv`：

```bash
adapters/holoocean/.venv/bin/python tools/calib/imu_allan.py \
    --bag /path/to/static.mcap --topic /raw/imu \
    --out imu_noise.yaml --json allan.json --plot allan.png   # --plot 需要 matplotlib（.[plot] extra）
```

输出 `imu_noise.yaml`：

| 字段 | 来源 | 单位 |
|---|---|---|
| `sigma_gyro_c` / `sigma_accel_c` | Allan 曲线 −1/2 斜率段在 τ = 1 s 的值 N（白噪声密度） | rad/s/√Hz、m/s²/√Hz |
| `sigma_gyro_bias` / `sigma_accel_bias` | Allan 最小值 / 0.664（偏置不稳定性） | rad/s、m/s² |
| `sigma_gyro_bias_walk_c` / `sigma_accel_bias_walk_c` | 三项联合非负最小二乘模型 σ²(τ)=N²/τ+C²+K²τ/3 的 K（速率随机游走）；若拟合得到 0 或曲线未进入 +1/2 段则为 0 | rad/s²/√Hz、m/s³/√Hz |
| `rate_hz` | 录制实测采样率 | Hz |
| `gravity_mps2` | `--gravity`（默认 9.80665） | m/s² |

三轴各算一份，取最大值（保守）。`--json` 里保留逐轴、逐 τ 的曲线和每段拟合用到的 τ 区间。

## 真机（HWT9053-485）采集流程（到货后执行）

1. IMU 装进电子舱、固定在最终安装位置后再采（安装应力和温度环境会改变偏置）。
2. ROV 整机放在稳固台面上，**推进器断电**（不是只解锁），关掉 LED 灯与云台电机，避免振动和电流磁场。
3. 树莓派侧跑 PREP-D-02 转发服务，岸上跑 PREP-D-03 `/raw/imu` adapter，按 200 Hz 写 MCAP；确认时间戳来自 PLL 重建的采样时钟而非收包时刻（`header.capture_time`）。
4. 采集 **≥ 8 小时**（随机游走段要 τ 到几百秒才可见，脚本要求每个 τ 至少 9 个簇，8 h 只能可信到 τ≈50 min）。每小时记一次舱内温度（HWT9053 自带温度，或外接探头）；温漂会在 Allan 曲线长 τ 端伪装成随机游走。
5. 跑脚本；若输出提示 `random-walk regime not reached`，把 walk 项留 0（估计器回退到 `sigma_*_bias`），并在 rig 注释里写明。
6. 把 `imu_noise` 块回填到 `configs/rig/bluerov2_contract.yaml`（PREP-A-03 创建，当前值为厂家标称换算），`calibration_version` 递增，注释里记录采集日期、时长、温度范围。

## 用 HoloOcean 录制验证脚本

HoloOcean 的 `IMUSensor` 噪声定义（读自 `external_repos/HoloOcean/engine/Source/Holodeck/Sensors/Private/IMUSensor.cpp`，`TickSensorComponent`）：每次传感器 tick 加一个 N(0, `AccelSigma`/`AngVelSigma`) 的白噪声样本，偏置每 tick 累加一个 N(0, `AccelBiasSigma`/`AngVelBiasSigma`) 的增量。因此与连续时间参数的换算（`rate_hz` 为该传感器的 `Hz`）：

```
sigma_c        = Sigma      / sqrt(rate_hz)     # 白噪声密度  （unit/√Hz）
sigma_walk_c   = BiasSigma  * sqrt(rate_hz)     # 随机游走密度（unit/s/√Hz）
```

例：`AngVelSigma: 0.00123`、`Hz: 200` → `sigma_gyro_c ≈ 8.7e-5 rad/s/√Hz`；`AngVelBiasSigma: 0.00388` → `sigma_gyro_bias_walk_c ≈ 0.055 rad/s²/√Hz`（这是 HoloOcean 文档示例值，作为随机游走非常大，验证时建议改小到 1e-5 量级，否则曲线几乎没有 −1/2 段）。

验证步骤：在 A-03 合同基线上把 IMU 配置改为已知的 `AccelSigma`/`AngVelSigma`/`AccelBiasSigma`/`AngVelBiasSigma`、`ReturnBias: true`，机体静止（推力全零）录 ≥ 30 分钟仿真时间（fidelity profile，ticks 200），`record_session.py` 写 MCAP，跑脚本，与上式换算值比较：白噪声 <20%（验收阈值），随机游走 <30%。仿真录制目前未跑（本机无 HoloOcean），单测用的是同一套离散化公式的合成流。

**待确认**：该换算假设 `TickSensorComponent` 只在传感器自身的 `Hz` 节拍上被调用（`HolodeckSensor` 的采样调度），而不是每个引擎 tick；若引擎按 `ticks_per_sec` 调用，`rate_hz` 应换成 `ticks_per_sec`。用上面的验证录制即可分辨（两者相差 √(ticks_per_sec/Hz)）。
