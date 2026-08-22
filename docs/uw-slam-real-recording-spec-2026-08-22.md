# 真实 HoloOcean 录制规格：三条固定轨迹

> 状态：规格文档。实际在 Windows 机器上执行录制仍是用户动作，不是这份文档
> 或 `record_session.py` 能替代的。

## 目标

固定三类 1–3 分钟轨迹（直线、转弯、小回环），用同一套 `record_session.py`
录制管线产出结构一致的 canonical MCAP bag，供 audit 工具和 baseline 消融消费。

## 话题与实际采样率——一个必须诚实说明的限制

`record_session.py` 的 `_write_keyframe` 只在某个 tick **同时带有
`LeftCamera`/`RightCamera` 两个键**时才会写任何消息（见该函数文档字符串）——
这不是本次改动引入的新限制，是这个仓库整体"以 camera keyframe 为锚点"的架构
延续（`apps/replay_demo` 的每一条证据都挂在某个 `kfN` 上）。结果是：

- `/raw/camera/left`、`/raw/camera/right`：每个 camera keyframe 各一条。
- `/gt/state`：camera keyframe 上若同一 tick 有 `PoseSensor` 读数就写一条。
- `/evidence/depth`：camera keyframe 上若同一 tick 有 `DepthSensor` 读数就写
  一条。
- `/raw/sonar_frame`、`/raw/imu`、`/raw/dvl`：**同样
  只在 camera keyframe 命中的 tick 上，若该 tick 恰好也带有对应传感器读数才
  写**。

也就是说，如果 IMU/声呐/DVL 在真实 HoloOcean 场景里配置的采样率比相机高（比如
IMU 常见 100–200Hz，相机常见 10–30Hz），这套管线录到的不是"IMU 原始高频流"，
而是"每个相机 keyframe 附近恰好命中的那一条 IMU 读数"——是一种降采样到相机
keyframe 频率的近似，不是真实高频 IMU 数据。这与整个仓库当前"批处理、按
keyframe 组织证据"的架构是一致的（不是这次实现的疏漏），但如果将来需要真正
高频 IMU 预积分，`record_session.py` 需要
独立于相机 keyframe 单独记录 IMU tick，这是明确的后续工作，这里不展开。

## 三类固定场景

统一用 `OpenWater-HoveringCamera`（`record_session.py --scenario` 默认值）
场景，通过 `--command` 控制 HoveringAUV 8 路推进器
指令（`[垂直x4, 水平x4]`，见 `_default_command()` 的布局注释）区分轨迹形状。
下面给出的水平推进器不对称值是**未经真实 HoloOcean 安装验证的起点**，不是
标定好的常数——真正在 Windows 上录制时大概率需要按实际转向响应微调，这里给的
是"从哪个值开始试"而不是"精确答案"，因为这个 sandbox 没有真实 HoloOcean 渲染
能力去闭环验证转弯半径。

| 场景 | 推进器指令 | 时长（tick，30Hz 默认） | 预期轨迹 |
|---|---|---:|---|
| `straight_line` | `[0,0,0,0, 10,10,10,10]`（`_default_command()` 默认值，四个水平推进器对称） | 1800–5400（1–3 分钟） | 近似直线前进 |
| `turn` | `[0,0,0,0, 5,15,5,15]`（水平推进器不对称，起点猜测值） | 900–1800（覆盖约 90°–180° 航向变化，不追求闭合） | 单次转弯，不闭合成环 |
| `small_loop` | 与 `turn` 相同的不对称指令 | 1800–5400，直到航向变化约 360°（需要现场观察 `/gt/state` 判断何时闭合，无法在这份文档里给出精确 tick 数） | 转弯持续足够久,自然闭合成一个小回环 |

录制命令示例（`turn`/`small_loop` 用同一个不对称指令,靠时长区分):

```bash
python -m uw_holoocean_adapter.record_session \
  --scenario OpenWater-HoveringCamera --seed 42 \
  --command 0 0 0 0 10 10 10 10 \
  --num-ticks 3600 --out straight_line.mcap

python -m uw_holoocean_adapter.record_session \
  --scenario OpenWater-HoveringCamera --seed 43 \
  --command 0 0 0 0 5 15 5 15 \
  --num-ticks 1200 --out turn.mcap

python -m uw_holoocean_adapter.record_session \
  --scenario OpenWater-HoveringCamera --seed 44 \
  --command 0 0 0 0 5 15 5 15 \
  --num-ticks 3600 --out small_loop.mcap
```

三条轨迹用不同 `--seed`（避免场景随机化状态相互干扰,虽然
`HoloOceanSession.apply_randomization()` 目前还没实现,见
`holoocean_driver.py`),`--num-ticks` 按 30Hz 默认 tick rate 估算,实际以
`HoloOceanSession._env._ticks_per_sec` 为准(见 `record_session.py` 里
`sim_time_s` 的计算方式)。

## 坐标与时钟约定(复用已有约定,不新增)

- 每条消息的 `ObservationHeader.clock_domain` 固定写
  `CLOCK_DOMAIN_SIMULATION`(`time_utils.make_stamp` 的调用方统一这么设置)——
  这是仿真时间,不是墙钟时间,`capture_time`/`receive_time` 的区分见
  `schemas/proto/uw/domain/observation.proto`。
- Pose、位姿全部走 `Pose3`(平移 + xyzw 四元数),不引入欧拉角——见
  `CLAUDE.md`"代码约定"一节。
- `sensor_frame` 命名沿用现有约定:`camera_left_link`/`camera_right_link`(已有)、
  `sonar_link`/`imu_link`/`dvl_link`(本次新增,匹配
  `configs/rig/*.yaml` 里 `frame_tree` 已有的 `sonar_link`/`imu_link` 命名,
  `dvl_link` 是新引入的,当前 `configs/rig/example_auv*.yaml` 还没有对应的
  frame_tree 边——用真实 DVL 数据前需要先补上这条标定边,这是标定层面的
  后续工作,不是这份录制规格能替代的)。

## 验收

- 三个 bag 各自能被 audit 工具正确识别出"哪些 topic 存在
  、哪些缺失、采样率是否合理"。
- 至少 `straight_line.mcap` 能被 `apps/replay_demo`(`estimator_mode:
  stereo_landmark_vo`)跑出非零的相对位姿因子——这是复用 EuRoC adapter
  已经验证过的"真实相机数据能跑出真实 VO"能力。
