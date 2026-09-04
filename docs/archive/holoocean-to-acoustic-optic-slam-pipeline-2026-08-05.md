---
title: HoloOcean 到声光融合 SLAM demo pipeline 方案
created: 2026-08-05
type: engineering-plan
aliases:
  - HoloOcean Acoustic-Optic SLAM Pipeline
  - 声光融合 SLAM demo pipeline
tags:
  - underwater-vision
  - acoustic-optic-fusion
  - holoocean
  - slam
  - demo
status: evolving-plan
updated: 2026-08-22
verified_against: "8df083b + current worktree"
---

# HoloOcean 到声光融合 SLAM demo pipeline 方案

> **文档定位**：本文记录第一阶段 HoloOcean baseline 的工程方案，以及它向当前
> `uw_slam` 架构演进的过程。当前代码事实以
> [代码库参考](../uw-slam-codebase-reference-2026-08-18.md) 为准；长期模块边界和技术
> 决策以[平台架构设计](../acoustic-optic-slam-platform-architecture-2026-08-17.md)为准。
> 本文中的早期时间表和串联方案属于历史工程背景，不代表当前功能已经全部接通。

## 三十秒摘要

最初方案用 SVIn 提供 VIO 位姿，再由 `sonar_camera_reconstruction` 累积声光点云；
它适合作为对比 baseline，但声呐没有反向约束位姿，地图也缺少可重定位的局部证据。
目标架构因此升级为 ROS 无关的领域契约、证据/因子边界、pose-graph 状态估计和
versioned submap。当前仓库已经跑通合成数据垂直切片和一条真实 HoloOcean 双目离线
VO 回放；后者没有 sonar/IMU/DVL、求解器仍 stalled，因此不能表述为真实声光 SLAM
闭环。旧版完整 baseline 也仍未端到端连接。

## 当前实施状态

| 状态 | 内容 |
|---|---|
| 当前垂直切片 | 合成 MCAP → 黑盒相对位姿或相机计算的双目 VO → 深度/声呐因子 → pose graph → ATE；另有并行声光稠密 evidence → submap 路径 |
| 部分实现 | HoloOcean Python 网关已实际录制；50-keyframe 真实双目+depth+GT bag 已离线回放并产出 49 条 VO 相对位姿（对齐 ATE RMSE 0.5596 m，solver stalled）；ROS2 ImagingSonar 只完成传输；相机去畸变原语尚未接入 replay |
| 尚未接通 | 全传感器实时 HoloOcean/UE5 → core 数据流、真实 sonar/IMU/DVL 融合、在线 VIO/SLAM、正式重建后端、旧串联 baseline 的完整端到端运行 |
| 历史 baseline | SVIn 位姿输入 `sonar_camera_reconstruction`；保留用于比较，不作为平台骨架 |

## 阅读导航

1. [总体目标](#1-总体目标)
2. [工程分层](#2-工程分层)
3. [数据接口设计](#3-数据接口设计)
4. [接口级落地补充](#4-接口级落地补充基于-2026-08-13-代码审阅)
5. [Pipeline 分阶段计划](#5-pipeline-分阶段计划)
6. [关键技术风险](#6-关键技术风险)
7. [给工程师的任务拆分](#7-给工程师的任务拆分)
8. [最小验收表](#8-最小验收表)
9. [与现有知识库关联](#9-与现有知识库关联)
10. [目标架构演进记录](#10-目标架构演进记录2026-08-17-评审与升级方案)
11. [代码审计修订附录](#11-代码审计修订附录2026-08-18)

---

> 背景：团队已经基于 HoloOcean（外部知识库资料）搭建好水下声光仿真环境。现阶段目标是结合 HoloOcean 仿真、`ivanacollg/sonar_camera_reconstruction` 与 `AutonomousFieldRoboticsLab/SVIn`，构建从仿真数据生成到声光融合 SLAM demo 的完整 pipeline，暂不追求完整 ROV/AUV 产品系统。

> 2026-08-17 架构定位更新：现有“SVIn VIO 位姿 → `sonar_camera_reconstruction` 点云”的串联方案保留为第一阶段 baseline，但不作为最终系统。目标架构升级为“VIO 先验驱动的声光 pose-graph SLAM + 自适应双前端稠密建图”：SVIn 提供连续局部里程计，成像声纳、depth 与回环约束在上层 pose graph 中修正关键帧位姿；双目视觉精细重建与 sonar-grounded 抗退化重建生成局部几何，再由 submap manager 维护全局一致地图。

> 长期平台边界、领域契约、AI 模型生命周期、运行时状态机和分阶段决策门以[水下声光融合 SLAM 平台长期架构设计](../acoustic-optic-slam-platform-architecture-2026-08-17.md)为准。本文继续保留为 HoloOcean 集成与第一阶段 baseline 的工程方案。

## 1. 总体目标

构建一个可反复运行、可录屏、可做算法替换的 pipeline：

```text
HoloOcean simulation
  -> camera / imaging sonar / IMU / depth / odometry / GT pose
  -> ROS bag or normalized dataset
  -> adapter / topic bridge / calibration package
  -> sonar_camera_reconstruction branch: opti-acoustic reconstruction demo
  -> SVIn branch: visual-inertial-sonar-depth SLAM baseline / audit
  -> evaluation + RViz visualization + video export
```

核心验收需要满足以下几点，不止于“单个模型 smoke test”：

1. 仿真端能稳定导出多传感器同步数据。
2. 数据能转换成两个 codebase 可消费的 topic / bag / config。
3. 至少一个分支能跑出 fused reconstruction / trajectory / map 可视化。
4. 有 ground truth pose 支撑 ATE/RPE、scale drift、trajectory overlay 等基本评测。

## 2. 工程分层

建议不要直接改两个论文仓库的核心代码。应在外层建立一个薄 glue repo，例如：

```text
uw_ao_slam_pipeline/
  README.md
  configs/
    holoocean_sensor_config.yaml
    camera_intrinsics.yaml
    sonar_model.yaml
    tf_tree.yaml
  docker/
    ros1_noetic_sonar_camera_reconstruction.Dockerfile
    ros2_jazzy_svin.Dockerfile
  scripts/
    record_holoocean_bag.py
    export_gt_pose.py
    convert_holoocean_to_ros1_bag.py
    convert_holoocean_to_ros2_bag.py
    topic_audit.sh
    run_sonar_camera_reconstruction.sh
    run_svin_ros2.sh
    run_eval_trajectory.sh
    record_rviz_video.sh
  adapters/
    holoocean_to_oculus_ping/
    holoocean_to_imagenex_range/
    holoocean_to_camera_info/
    tf_static_publishers/
  outputs/
    bags/
    rviz/
    videos/
    eval/
    logs/
  third_party/
    sonar_camera_reconstruction/   # git submodule or external clone
    SVIn/                          # git submodule or external clone
```

原则：

- `HoloOcean` 负责生成可控数据。
- `adapters/` 负责 topic/schema 转换。
- `sonar_camera_reconstruction` 和 `SVIn` 尽量保持原仓库形态，必要修改用 fork/patch 记录。
- `outputs/` 保留每次实验的 bag、日志、截图、评测结果。

## 3. 数据接口设计

### 3.1 HoloOcean 输出最低要求

至少导出：

| 数据 | 用途 | 推荐 topic / 字段 |
|---|---|---|
| RGB camera | optical front-end | `/camera/image_raw`, `/camera/camera_info` |
| imaging sonar | acoustic front-end | 原始 sonar polar intensity；同时保留 range/beam metadata |
| IMU | SVIn / VIO | `/imu/data` |
| depth / pressure | SVIn depth factor | `/depth` 或 `/pressure/depth` |
| odometry / GT pose | reconstruction 初值、评测 | `/odom`, `/ground_truth/pose` |
| TF | frame alignment | `map -> odom -> base_link -> camera/sonar/imu` |

### 3.2 面向 `sonar_camera_reconstruction` 的适配

该仓库 README 描述输入为 sensor information：sonars、camera、odometry，并输出 pointcloud。它包含 `sonar_oculus` msg type。

适配重点：

1. 确认其实际订阅 topic：camera image、Oculus sonar ping、odometry、TF。
2. 将 HoloOcean sonar 转为仓库期望的 `OculusPing` 或兼容消息。
3. 如果短期难以完全模拟 OculusPing，则优先写最小 adapter：只填充算法实际使用的字段，例如 bearing/range/intensity/range resolution。
4. 确保 camera intrinsics、sonar intrinsics、camera-sonar extrinsics 与 TF 一致。

### 3.3 面向 `SVIn` 的适配

SVIn main branch 是 ROS2 Jazzy，但 README 明确说 sonar/depth modes 默认禁用，旧 sonar/custom topic 到 ROS2 仍在迁移。因此 SVIn 分支应分两步：

1. main branch audit：先跑 visual-inertial / GoPro / AFRL launch，验证 build、bag replay、trajectory 输出。
2. sonar/depth branch audit：检查 `ros1` branch 是否更接近论文中的 sonar-depth tightly coupled 实现；必要时用 HoloOcean 数据生成 ROS1 bag。

适配重点：

- SVIn 公开数据里的 sonar topic 可能是 `/imagenex831l/range` 和 `/imagenex831l/range_raw`。
- HoloOcean imaging sonar 可能需要降维/抽取成 profiling sonar range-like observation，才能进入 SVIn 的 sonar factor。
- 不要把“SVIn main branch 跑通 VIO”视为“声光融合 SLAM 跑通”。必须记录 sonar/depth 是否实际启用。

## 4. 接口级落地补充（基于 2026-08-13 代码审阅）

> 本节补充的是“真正接线时要填哪些字段、接哪些 topic、先验风险在哪里”。依据包括 `sonar_camera_reconstruction` main branch 的 README、`merge.launch`、`OculusPing.msg`、`merge_node.py`、`imaging_sonar.py`，以及 SVIn main branch README。SVIn 仓库 clone 在本机曾因网络超时未完整落地，因此 SVIn 的代码级判断目前只使用 README 可核验信息，后续应在 Linux/ROS 环境中继续做 branch-level audit。

### 4.1 `sonar_camera_reconstruction` 实际输入输出

`sonar_camera_reconstruction` 的主分支是 ROS1 Noetic。`merge.launch` 中默认订阅和发布如下：

| 类型 | 默认 topic | ROS message | 用途 |
|---|---|---|---|
| imaging sonar | `/sonar_oculus_node/M750d/ping` | `sonar_oculus/OculusPing` | 声纳极坐标强度图、bearing、range bin metadata |
| camera | `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 单目图像，做 foreground/contact region segmentation |
| odometry | `/bruce/slam/localization/odom` | `nav_msgs/Odometry` | 将融合出的局部点云变换到 `map` frame |
| segmented image | `/sonar_camera_reconstruction/segmented_img/compressed` | `sensor_msgs/CompressedImage` | 视觉分割/匹配调试图 |
| sonar feature image | `/sonar_camera_reconstruction/feature_img/compressed` | `sensor_msgs/CompressedImage` | CFAR 声纳特征可视化 |
| fused cloud | `/sonar_camera_reconstruction/cloud` | `sensor_msgs/PointCloud2` | 最终声光融合点云 |

`OculusPing.msg` 的最小必填字段比完整 Oculus 驱动简单：

```text
std_msgs/Header header
OculusFire fire_msg
uint32 ping_id
uint16 part_number
uint32 start_time
int16[] bearings         # bearing * PI / 18000
float64 range_resolution # 每个 range bin 长度，单位 m
uint32 num_ranges
uint32 num_beams
sensor_msgs/CompressedImage ping
```

对 HoloOcean/ocean_t adapter 来说，第一版不需要模拟完整 Oculus firmware。只要能稳定提供：

1. `header.stamp`：使用 simulation time，不用 wall time。
2. `header.frame_id`：建议为 `sonar_frame`，并在 TF 中连接到 `base_link`。
3. `bearings`：按 `int16` 存储，代码内部用 `bearing * pi / 18000` 转 rad，因此 adapter 应写入 `bearing_deg * 100`。
4. `range_resolution`：`max_range / num_ranges`，单位 m/bin。
5. `num_ranges`、`num_beams`：必须与压缩后的 `ping` 图像尺寸一致。
6. `ping`：把 HoloOcean sonar polar intensity 转成 `uint8` 灰度或 BGR 图后 JPEG/PNG 压缩，填入 `CompressedImage`。

### 4.2 `sonar_camera_reconstruction` 算法实际怎样使用声光数据

代码链路不是一般意义上的 neural fusion，而是几何/区域级匹配：

```text
OculusPing.ping compressed sonar image
  -> cv2 decode -> grayscale sonar image
  -> CFAR detect peaks in polar image
  -> extract first-contact line scan
  -> polar-to-Cartesian remap using bearings + range_resolution
  -> DBSCAN cluster sonar scanline

Compressed camera image
  -> threshold / connected components segmentation
  -> candidate contact regions

Sonar clusters + camera regions
  -> use Ts_c sonar-to-camera transform
  -> project sonar extended coordinates to camera image
  -> choose overlapping camera region / sonar cluster
  -> infer depth-colored region and generate point cloud

Odometry pose
  -> transform local cloud to map frame
  -> publish PointCloud2
```

这意味着“图像能显示”不能作为 adapter 的验收标准，下面这些几何量必须一致：

| 检查项 | 若错误会发生什么 |
|---|---|
| `bearings` 单位和顺序 | polar-to-Cartesian remap 左右翻转或 FOV 错误 |
| `range_resolution` 与 `num_ranges` | 点云尺度错误，近/远结构错位 |
| sonar image shape `[range, beam]` vs `[beam, range]` | CFAR peak 和 bearing/range 对不上 |
| `Ts_c` sonar-to-camera 外参 | 声纳 cluster 投影不到相机 region，输出点云为空或漂移 |
| camera intrinsics `K/D/width/height` | 图像 region 与声纳投影 overlap 错误 |
| odometry frame | 点云能生成但在 `map` 中方向/尺度错误 |
| `/use_sim_time` 和 stamp | replay 时 callback 可运行但 TF/同步关系不可解释 |

第一版调试时建议把 `fast_performance: False`、`sonar_features: True` 打开，优先观察 `/feature_img/compressed` 和 `/segmented_img/compressed`，确认声纳 first-contact line 与 camera contact region 是否真的重叠。若没有重叠，不应先调 CFAR 阈值，而应先检查 `Ts_c`、bearing 顺序、camera intrinsics 和 HoloOcean frame convention。

### 4.3 HoloOcean/ocean_t 到 `OculusPing` 的最小转换规格

如果 `ocean_t` 当前已能从 HoloOcean 输出 sonar polar image，建议先定义一个中间 schema，不直接绑定 ROS message：

```yaml
sonar_ping:
  stamp: <float seconds in sim time>
  frame_id: sonar_frame
  intensity: <uint8 array, shape [num_ranges, num_beams]>
  min_range_m: 0.0
  max_range_m: <float>
  range_resolution_m: <float>
  bearings_deg: <array length num_beams, ascending left-to-right or right-to-left explicitly documented>
  sound_speed_mps: <optional>
  model: holoocean_imaging_sonar
```

再从该中间 schema 生成 ROS1 `OculusPing`。这样后续可以同时派生：

- `sonar_camera_reconstruction` 所需 `OculusPing`；
- SVIn 可能需要的 `/imagenex831l/range` / `/imagenex831l/range_raw`；
- 自研 ROS2 `uw_msgs/SonarImage`；
- 离线评测用 `.npz` / `.hdf5`。

第一版转换伪代码：

```python
msg = OculusPing()
msg.header.stamp = rospy.Time.from_sec(ping.stamp)
msg.header.frame_id = "sonar_frame"
msg.bearings = [int(round(deg * 100.0)) for deg in ping.bearings_deg]
msg.range_resolution = ping.range_resolution_m
msg.num_ranges = ping.intensity.shape[0]
msg.num_beams = ping.intensity.shape[1]
msg.ping = encode_compressed_image(ping.intensity, fmt="jpeg")
```

注意：`bearings` 必须按 `imaging_sonar.py` 中 `interp1d(... assume_sorted=True)` 的假设排序。如果 HoloOcean 输出 beam angle 是降序，应在 adapter 中同时翻转 `bearings` 和 intensity 的 beam 维度，不能只排序 angle。

### 4.4 面向 SVIn 的接入判断

SVIn README 明确给出三个当前限制：

1. main branch 使用 ROS2 Jazzy。
2. sonar/depth modes 在 main branch 默认禁用。
3. 旧 sonar/custom topic 到 ROS2 的迁移仍未完成；公开数据中 sonar topic 名称为 `/imagenex831l/range` 与 `/imagenex831l/range_raw`。

因此本 pipeline 中 SVIn 不应作为第一阶段主线。更合理定位是：

| 阶段 | SVIn 角色 | 交付 |
|---|---|---|
| Phase 1 | 只跑 visual-inertial baseline | 官方 GoPro/AFRL dataset replay + trajectory 输出 |
| Phase 2 | 审计 `ros1` branch 的 sonar/depth factor | topic、message type、factor 入口、配置项清单 |
| Phase 3 | 判断 HoloOcean imaging sonar 能否降维成 Imagenex-style range observation | adapter 需求与不可观测性说明 |
| Phase 4 | 若必要，改为“SVIn 思路参考 + 自研轻量 factor graph 回放” | 不把 SVIn main branch 当完整声光 SLAM 结果 |

关键工程判断：HoloOcean imaging sonar 是二维 polar intensity；SVIn README 暗示使用的是 Imagenex range-like topic。两者未必同构。如果 SVIn 的 sonar factor 只接受单波束/剖面式 range observation，则从 imaging sonar 到 SVIn 需要做抽取：例如在每个 ping 中根据 CFAR/first return 生成若干 range-bearing 或中心 beam range。这个过程会丢失二维声纳图的部分信息，应在 claim 中写成“profiling-sonar proxy / range observation extraction”，不能写成完整利用 FLS image。

### 4.5 推荐的第一周执行顺序

```text
Day 1: 固定 ocean_t/HoloOcean 输出 schema
  - 导出 10–30 s short_line sequence
  - 保存 RGB、sonar polar intensity、IMU、depth、odom、GT pose
  - 写 topic/time/frame audit

Day 2: 构造 OculusPing adapter 离线单元测试
  - 输入一帧 sonar polar intensity
  - 输出 OculusPing bag
  - 检查 bearings、range_resolution、num_ranges、num_beams、compressed image shape

Day 3: 跑 sonar_camera_reconstruction sample bag
  - 原作者 sample 可跑是环境验收
  - 保存 topic list、node graph、RViz screenshot

Day 4: 用 HoloOcean/ocean_t bag 跑 merge_node
  - 先只追求 callback 收到三路数据
  - 再检查 feature image、segmented image
  - 最后检查 PointCloud2 是否非空

Day 5: 做几何 sanity check
  - 直线轨迹下点云方向是否和 GT 一致
  - 单个已知平面/柱体的 range 是否正确
  - camera-sonar overlay 是否随外参微调呈连续变化

Day 6-7: SVIn main branch 只做 baseline/audit
  - build/replay 官方数据
  - 不把 VIO 成功写成声光融合成功
  - 列出 ros1 branch sonar/depth factor 审计任务
```

### 4.6 第一版 demo 的成功/失败判据

| 层级 | 成功判据 | 失败时优先定位 |
|---|---|---|
| 数据层 | HoloOcean/ocean_t 可稳定导出短序列，所有 stamp 单调 | logger、sim time、文件格式 |
| schema 层 | `OculusPing` 字段完整，shape/单位正确 | bearing、range bin、compressed image、frame_id |
| ROS 层 | `merge_node` 同时收到 sonar/camera/odom | topic 名、message type、ROS1/ROS2 bridge |
| 几何层 | sonar cluster 投影到 camera region 附近 | `Ts_c`、camera K/D、坐标系、beam 顺序 |
| 输出层 | `/sonar_camera_reconstruction/cloud` 非空且尺度合理 | CFAR threshold、segmentation threshold、odometry frame |
| 评测层 | 与 GT/scene geometry 的方向、尺度、相对位置一致 | GT frame、ATE/RPE 对齐、map/odom/base_link |

## 5. Pipeline 分阶段计划

### Phase 0：固定基准数据与坐标约定

交付物：

- HoloOcean 场景版本、seed、vehicle trajectory、sensor config。
- camera/sonar/IMU/depth/GT pose 的导出脚本。
- TF tree 图与 frame convention 说明。
- 一段 1–3 分钟的短 bag，便于快速迭代。

建议先做三条 trajectory：

1. 简单直线/平移：用于检查时间戳和尺度。
2. 小闭环：用于 SLAM trajectory overlay。
3. 浊度/低可见度变化：用于展示 sonar 补 geometry。

### Phase 1：跑通 `sonar_camera_reconstruction` 分支

目标：HoloOcean bag -> opti-acoustic reconstruction / pointcloud。

步骤：

1. 用作者 sample bag 跑通原仓库，记录 expected topic 和参数。
2. 导出 HoloOcean camera + sonar + odometry。
3. 写 adapter 到原仓库期望 topic/schema。
4. 跑 `merge.launch`，输出 RViz 截图和 pointcloud。
5. 与 HoloOcean GT / scene geometry 做粗评估：点云是否在正确尺度和方向上。

验收：

- 能用 HoloOcean 数据跑出 fused point cloud / reconstruction。
- 能展示 camera-only 与 sonar-assisted 的差异。
- 有 topic audit、TF audit、launch config、失败日志。

### Phase 2：跑通 `SVIn` 分支

目标：HoloOcean bag -> trajectory / SLAM baseline。

步骤：

1. main branch ROS2 build。
2. 官方 dataset replay，确认 launch 与输出。
3. 检查 sonar/depth 是否实际启用。
4. 审计 `ros1` branch，确认 sonar/depth factor 入口。
5. 将 HoloOcean camera/IMU/depth/sonar 转成 SVIn 可读格式。
6. 与 GT pose 做 ATE/RPE 评估。

验收：

- 至少跑通 visual-inertial baseline。
- 明确 sonar/depth 是否可用；如果不可用，给出代码层阻塞点。
- 给出 SVIn 是否适合作为当前 demo SLAM 后端的判断。

### Phase 3：统一 demo pipeline

目标：一个命令生成数据，一个命令跑 fusion，一个命令出视频和评测。

建议脚本：

```bash
# 1. 录制 HoloOcean 仿真数据
./scripts/record_holoocean_bag.py --scene pier --traj loop --duration 120

# 2. 转为 sonar_camera_reconstruction 可用 ROS1 bag
./scripts/convert_holoocean_to_ros1_bag.py --input outputs/bags/holoocean_loop --target sonar_camera_reconstruction

# 3. 跑 reconstruction demo
./scripts/run_sonar_camera_reconstruction.sh --bag outputs/bags/ao_recon_loop.bag

# 4. 转为 SVIn 可用 ROS2/ROS1 bag
./scripts/convert_holoocean_to_svin_bag.py --input outputs/bags/holoocean_loop --branch ros2-main

# 5. 跑 SVIn
./scripts/run_svin.sh --bag outputs/bags/svin_loop

# 6. 评测 + 导出视频
./scripts/run_eval_trajectory.sh --est outputs/svin/traj.txt --gt outputs/gt/pose.txt
./scripts/record_rviz_video.sh --config configs/demo.rviz
```

## 6. 关键技术风险

1. Sonar message schema 不匹配  
   HoloOcean imaging sonar 输出不一定能直接映射到 OculusPing 或 SVIn 的 Imagenex range topic。需要先读两个 codebase 的实际订阅字段，不要盲目做全量仿真消息。

2. sonar_camera_reconstruction 是 ROS1 主线  
   若 HoloOcean 当前是 ROS2，需要用 ros1_bridge、bag conversion 或离线转换脚本。

3. SVIn main branch 声呐/深度默认禁用  
   当前 SVIn 更适合作为 audit 和 baseline，不保证短期承担完整声光 SLAM demo 主线。

4. 坐标系与时间同步  
   声光融合最容易失败在 TF、timestamp、camera optical frame、sonar frame、NED/ENU 转换。必须先用简单直线轨迹验证尺度和方向。

5. 仿真结果不能直接支撑主 claim  
   HoloOcean demo 可以支撑工程闭环、可视化和消融开发，但后续研究 claim 仍应回到公开真实数据或真实采集数据验证。

## 7. 给工程师的任务拆分

### Engineer A：HoloOcean 数据导出与标准化

- 固定场景、trajectory、sensor config。
- 导出 camera / sonar / IMU / depth / odom / GT pose。
- 输出短 bag 与 topic audit。
- 确认 timestamp 单调、频率稳定、TF 连通。

### Engineer B：sonar_camera_reconstruction 跑通与适配

- 先跑作者 sample bag。
- 记录原仓库订阅/发布 topic。
- 写 HoloOcean -> OculusPing / repo schema adapter。
- 跑 HoloOcean bag，输出 pointcloud/RViz/video。

### Engineer C：SVIn 跑通与适配可行性审计

- build main branch。
- 跑官方 dataset。
- 检查 sonar/depth 是否启用。
- 审计 ros1 branch。
- 给出 HoloOcean 数据接入 SVIn 的最小字段需求与阻塞点。

### Engineer D：评测与 demo 产物

- 统一 outputs 目录。
- 轨迹评测 ATE/RPE。
- pointcloud/map/trajectory overlay。
- 录屏脚本与最终 demo README。

## 8. 最小验收表

| 模块 | 最低验收 | 证据 |
|---|---|---|
| HoloOcean data export | 1–3 min bag，含 camera/sonar/IMU/depth/GT | topic list、频率统计、bag 文件 |
| sonar_camera_reconstruction sample | 作者 sample bag 跑通 | build log、RViz 截图、视频 |
| HoloOcean -> reconstruction | HoloOcean bag 跑出 fused pointcloud | adapter 代码、RViz 截图、点云输出 |
| SVIn main | build + 官方数据 replay | build log、trajectory 或失败日志 |
| SVIn sonar/depth audit | 明确可用/不可用与原因 | branch audit、订阅 topic、代码入口 |
| Evaluation | GT vs estimated trajectory | ATE/RPE 表、trajectory plot |

## 9. 与现有知识库关联

- 水下声光融合感知可扩展 codebase 调研（外部知识库资料）
- Opti-Acoustic Turbid Recon（外部知识库资料）
- Versatile Opti-Acoustic Volumetric Mapping（外部知识库资料）
- HoloOcean（外部知识库资料）
- HoloOcean 2.0 Preview（外部知识库资料）
- 水下声光融合（外部知识库资料）
- 水下数据集与仿真器（外部知识库资料）

## 10. 目标架构演进记录：2026-08-17 评审与升级方案

### 10.1 结论：原串联方案可作为 baseline，但不足以成为最终声光 SLAM

当前方案为：

```text
HoloOcean stereo camera + IMU -> SVIn ROS2 -> VIO pose
HoloOcean camera + imaging sonar + VIO pose
  -> sonar_camera_reconstruction -> fused point cloud
```

这个方案工程上可行，且能快速形成实时轨迹与声光点云演示，但其能力边界必须写清：

1. SVIn ROS2 main 的 sonar/depth mode 尚未启用，实际定位主要是 stereo VIO。
2. Imaging sonar 没有参与位姿估计，无法在视觉退化时反向约束轨迹。
3. `sonar_camera_reconstruction` 使用外部 odometry 放置局部点云；VIO 漂移会直接转化为地图漂移。
4. 输出点云缺少 keyframe/submap 生命周期、回环后重积分和全局地图一致性管理。

因此第一阶段应称为：

> Stereo-VIO pose-guided opti-acoustic reconstruction baseline。

最终系统应称为：

> VIO-prior acoustic-optic pose-graph SLAM with adaptive dense mapping。

在不修改 SVIn 内部滑窗、没有把 camera/IMU/sonar 原始残差放进同一优化器之前，不使用“原始测量级紧耦合”表述。目标架构属于关键帧 pose-graph 层面的半紧耦合：声纳确实能够修改全局轨迹，但 VIO 内部状态仍由 SVIn 独立估计。

### 10.2 三种路线与选择

| 路线 | 定位融合 | 地图生成 | 优点 | 局限 | 定位 |
|---|---|---|---|---|---|
| A. 单向松耦合 baseline | SVIn stereo VIO | `sonar_camera_reconstruction` | 修改量小，最快形成演示 | 声纳不改善定位；不能证明声光 SLAM | 必做的中间里程碑 |
| B. 异步声光 pose graph | VIO relative-pose + conditional sonar factor + depth + loop | 双前端 local cloud + submap fusion | 兼顾实时、地图质量、故障隔离和可解释性 | 需要新增 sonar constraint frontend、scheduler、graph backend、map manager | 推荐目标架构 |
| C. SVIn 内部全紧耦合 | camera reprojection + IMU preintegration + imaging-sonar residual 联合滑窗 | 与状态估计共享稠密表示 | 理论耦合最深，研究价值高 | ROS2 sonar 迁移、残差建模、可观测性和调参风险最高 | 后续研究线，不作为当前 demo 阻塞项 |

选择路线 B，并保留路线 A 作为可持续回归测试的 baseline。路线 C 只有在路线 B 已获得稳定数据、标定、sonar constraint 和评测证据后再启动。

### 10.3 三个异步闭环

系统不要求所有传感器和算法以相同频率串行运行，而是拆为三个有不同实时预算的闭环：

```text
快速定位环（目标 15–30 Hz）
  stereo + IMU -> SVIn -> continuous odom -> base_link

声学修正环（目标 1–5 Hz）
  sonar keyframe registration + depth + loop
    -> pose graph -> map -> odom + optimized keyframe poses

稠密建图环（目标 2–10 Hz）
  stereo local cloud + sonar-grounded local cloud
    -> confidence fusion -> submap -> optimized global map
```

快速定位环优先保证连续性和低延迟；声学修正环允许低频异步更新；稠密建图环使用 bounded queue 和 keyframe 策略，计算不足时丢弃旧的非关键帧，不阻塞定位。

### 10.4 模块边界

#### 10.4.1 Sensor gateway

职责：

- 接收 HoloOcean/ocean_t 的 RGB、stereo、IMU、depth、imaging sonar、GT pose；
- 统一 simulation time、单位、frame convention 和消息 QoS；
- 完成 UE/NED/ENU/ROS optical frame 转换，且每类转换只执行一次；
- 保存规范化 bag，使所有下游模块可脱离 HoloOcean 独立回放；
- 将内部统一 sonar schema 转为 `OculusPing`，而不是让全系统绑定厂商消息。

#### 10.4.2 SVIn wrapper

职责：

- 输入 stereo image + IMU；
- 输出连续 `nav_msgs/Odometry`、path、tracking health；
- 明确输出是 body 还是 IMU frame，并转换为统一 `odom -> base_link`；
- 把相邻关键帧相对位姿和 covariance 交给上层 pose graph；
- 不负责全局 `map` frame，不直接承担 sonar/depth 融合 claim。

#### 10.4.3 Sonar constraint frontend

第一版处理链：

```text
polar sonar image
  -> normalization / CFAR / valid mask
  -> polar-to-Cartesian view
  -> keyframe selection
  -> feature/edge/scan registration
  -> outlier rejection
  -> partial relative pose + covariance + quality
```

输出不应伪装成完整 6-DoF 高置信相对位姿。对普通 2D FLS/imaging sonar，第一版只向图中可靠注入 `x/y/yaw`，对 `z/roll/pitch` 设置低 information 或不添加约束。elevation ambiguity 必须通过 covariance 显式表示，而不是令 elevation 为零后当作精确三维测量。

#### 10.4.4 Reliability scheduler

调度器根据以下信号控制 visual/VIO edge、sonar edge 和 mapping observation 是否进入图或地图：

| 模态 | 健康信号 |
|---|---|
| Visual/VIO | feature count、inlier ratio、track length、reprojection residual、pose covariance、tracking state |
| Sonar | registration fitness、inlier ratio、SNR、entropy、peak sharpness、multipath/shadow suspicion |
| Cross-modal | VIO prediction 与 sonar registration 的 innovation、时间同步误差、运动一致性 |
| Depth | 与当前 z prediction 的 residual、跳变和有效范围 |

第一版采用可解释的状态机：

```text
VISUAL_NOMINAL
  -> VIO 主导，sonar 低权重监测

VISUAL_DEGRADED
  -> 降低 VIO edge information，增强通过质量门的 sonar/depth edge

ACOUSTIC_DEGRADED
  -> 不允许 sonar 接管；维持 VIO/IMU/depth，暂停低置信声学建图

BOTH_DEGRADED
  -> 冻结全局地图融合，允许短时 propagation，等待恢复或重定位

RECOVERY
  -> 与近期高质量关键帧匹配，验证成功后建立新 submap 或连接旧图
```

后续再把 hard state machine 升级为连续 information scheduling；不能一开始只按图像亮度或单个 feature-count 阈值切换模态。

#### 10.4.5 Acoustic-optic pose graph

第一版图变量只包含关键帧位姿，避免重复实现 SVIn 的速度和 bias 状态：

$$
\mathcal{X}=\{\mathbf{T}_{WB_i}\}_{i=0}^{N}.
$$

目标函数：

$$
\mathcal{X}^*=\arg\min_{\mathcal{X}}
\sum_i\|\mathbf{r}^{vio}_{i,i+1}\|^2_{\mathbf{\Lambda}^{vio}_i}
+\sum_k\rho_s\!\left(\|\mathbf{r}^{sonar}_{a_k,b_k}\|^2_{\alpha^s_k\mathbf{\Lambda}^{sonar}_k}\right)
+\sum_i\|r^{depth}_i\|^2_{\lambda_z}
+\sum_l\rho_l\!\left(\|\mathbf{r}^{loop}_l\|^2_{\mathbf{\Lambda}^{loop}_l}\right).
$$

其中：

- `VIO factor`：来自 SVIn 相邻关键帧 relative pose；
- `sonar factor`：来自成像声纳相邻帧或 scan-to-submap registration，只约束可观维度；
- `depth factor`：一维 z prior；
- `loop factor`：视觉或后续声纳 place recognition + registration；
- $\alpha^s_k$：声纳质量和 cross-modal audit 共同决定的 information scale；
- 所有 sonar/loop edge 使用 robust kernel，错误闭环不能直接进入主图。

输出：

- 连续实时 `odom -> base_link` 仍由 SVIn 提供；
- pose graph 只发布低频 `map -> odom` 修正和 optimized keyframe poses；
- 若优化产生大跳变，先更新 submap pose，不直接让控制接口使用跳变的 `map` pose。

#### 10.4.6 自适应双建图前端

`sonar_camera_reconstruction` 不作为唯一地图后端。其前景/最近回波/每 bearing 近似恒距假设适合高浊度、近距离、结构化和平面/准平面目标，但在复杂礁石、管道节点、突出物、凹陷和大纵深场景中会产生几何塌缩或漏掉远处结构。

最终建图采用两条局部几何分支：

```text
清水或中等浊度：
  rectified stereo -> dense stereo depth -> high-resolution visual local cloud

高浊度或视觉退化：
  camera region + imaging sonar range
    -> sonar_camera_reconstruction -> sonar-grounded local cloud

两路 local cloud：
  -> uncertainty/quality weighting
  -> outlier and dynamic-point filtering
  -> TSDF/voxel/surfel submap
  -> optimized global map
```

视觉分支提供高分辨率表面细节；声学分支提供 metric range、视觉空洞和高浊度下的粗几何。两路数据不在二维图像通道上强行拼接，而是在局部三维 submap 中汇合。

#### 10.4.7 Submap manager

地图管理器必须保存“局部观测 + keyframe/submap pose”，不能只保存已经转换并固定到全局坐标、不再保留局部坐标引用的点：

- 每个 local cloud 绑定 keyframe id、capture timestamp、sensor quality 和生成配置；
- 小范围关键帧融合成 submap；
- pose graph 更新后优先修改 submap pose；
- 只有局部几何本身变化或需要高精度离线结果时才重积分；
- live map 与 optimized map 分开发布；
- tracking lost 时关闭全局融合，恢复后建立新 submap；
- 支持离线用 optimized keyframe poses 重新生成最终地图。

该结构避免回环后新旧点云断裂，也避免每次图优化都从第一帧重建全图。

### 10.5 时间、坐标与消息契约

统一 TF tree：

```text
map -> odom -> base_link
                 |- imu_link
                 |- camera_left_link -> camera_left_optical_frame
                 |- camera_right_link -> camera_right_optical_frame
                 |- sonar_link
                 `- depth_link
```

规则：

1. `map -> odom` 仅由 pose-graph backend 发布。
2. `odom -> base_link` 仅由 SVIn wrapper 发布。
3. 静态 sensor TF 来自单一 calibration source，不在 launch、代码和 YAML 中重复手写不同版本。
4. GT 使用独立数据流，只进入 evaluator；默认不广播到生产 TF tree。
5. 所有 stamp 使用 capture simulation time，不使用 callback receipt time 或 wall time。
6. camera/sonar observation 使用插值后的同一时刻 pose；不能直接读取“最新 odometry”。
7. camera-sonar 最近时间差、IMU gap、timestamp monotonicity 和 TF lookup success rate 必须成为自动审计指标。
8. 原始 sensor topic 使用 bounded `SensorDataQoS`；pose/constraint/health 使用 reliable QoS；高带宽队列过载时丢旧帧，不允许无界积压。

### 10.6 Recovery 与断开子图策略

视觉退化常使相邻关键帧匹配断裂。系统不强制当前帧只能匹配上一关键帧：

1. 从近期窗口选择视觉或声学质量最高的候选帧；
2. 使用 VIO prediction 限制 sonar correspondence 搜索区域；
3. registration 通过残差、inlier、运动界限和 cross-modal innovation 多重验证；
4. 验证失败时保留局部 disconnected submap，不添加虚假全局边；
5. 后续由视觉回环、声纳 place recognition 或重新进入共视区域连接子图；
6. 长时间不能连接时，在 UI 中明确显示“局部地图未全局对齐”，不隐藏失败状态。

### 10.7 Linux sonar 问题与双机实时路径

Linux HoloOcean sonar 成像异常不应阻塞 SLAM 主线。短期部署允许：

```text
Windows simulation host
  HoloOcean + ocean_t + normalized sensor publisher
            |
            | ROS2 DDS 或显式网络传输层
            v
Linux perception host
  sensor gateway + SVIn + sonar frontend + graph + mapping + RViz2
```

同时保留 bag 路径：Windows 录制规范化数据，Linux 离线回放。Linux 声纳排查必须先在 HoloOcean Python 原始数组边界定位：

- 原始数组正常、ROS 图像异常：检查 normalization、shape transpose、encoding、compression、QoS；
- raycast/octree 正常、GPU sonar 异常：检查 Linux GPU driver、Unreal RHI、headless/render context；
- 三种声纳都异常：检查 Linux world package、collision mesh、socket、range config 和 octree；
- 官方场景正常、自定义场景异常：问题集中在自定义 world/package/collision；
- Windows/Linux 原始数组均正常但显示不同：问题属于可视化动态范围，不先修改声纳物理参数。

### 10.8 分阶段交付

#### Milestone A：可诊断数据底座

- 固定 scene/seed/trajectory/sensor config；
- 输出 camera/sonar/IMU/depth/GT 的 1–3 分钟 bag；
- 完成 topic/time/TF/calibration audit；
- 使用直线、转弯、小闭环三类轨迹检查尺度和轴向。

#### Milestone B：原始松耦合 baseline

- SVIn 输出 VIO trajectory；
- GT pose + reconstruction 给出地图质量上限；
- SVIn pose + reconstruction 给出端到端 baseline；
- 明确该阶段声纳不参与定位。

#### Milestone C：声纳参与定位

- 实现 sonar keyframe、registration、partial-pose covariance；
- 将 VIO/sonar/depth factor 加入 pose graph；
- 完成 no-sonar 与 sonar-on 轨迹消融；
- 视觉退化片段中证明声纳减少 tracking lost 或降低局部 RPE。

#### Milestone D：自适应双前端建图

- stereo dense local cloud；
- sonar-grounded local cloud；
- confidence-aware submap fusion；
- live map 与 optimized map；
- loop correction 后地图不出现明显断层。

#### Milestone E：实时演示与压力测试

- Windows/Linux 双机或 Linux 单机实时运行；
- clean/mild/critical/severe turbidity 分桶；
- camera dropout、sonar degradation、timestamp delay、packet drop 故障注入；
- RViz/dashboard 同时展示 sensor、health state、轨迹、GT、live map、optimized map 和 latency。

### 10.9 验收指标

#### 轨迹

- ATE、RPE、drift per meter/minute；
- tracking lost count、lost duration、relocalization time；
- pose jerk、velocity jump；
- visual/sonar/depth residual 与 cross-modal innovation；
- `VIO only`、`VIO + depth`、`VIO + sonar`、`VIO + sonar + depth` 四组消融。

初始挑战目标：在 critical/severe visual degradation 片段，加入通过质量门的 sonar factor 后，局部 RPE 或 lost duration 相对 VIO-only 改善至少 20%；清水片段不得因 sonar edge 产生明显退化。具体阈值在首批固定场景上基于 baseline 重新冻结。

#### 地图

- point-to-GT-mesh accuracy/completeness；
- Chamfer/F-score 或 occupancy IoU；
- floaters/outlier ratio；
- loop 前后 map discontinuity；
- visual-only、sonar-grounded、adaptive fusion 三组对比；
- 误差容差同时报告绝对值和 sonar range-bin 归一化值。

#### 实时性

- HoloOcean real-time factor；
- localization、sonar constraint、mapping 的实际 output rate；
- capture-to-pose、capture-to-map 的 P50/P95 latency；
- CPU/GPU/内存占用；
- dropped frame、queue depth 和 graph optimization time。

初始工程目标：定位 15–30 Hz、声学修正 1–5 Hz、地图可视化 2–10 Hz、P95 端到端延迟不超过 200 ms、仿真实时因子不低于 1。若硬件或 HoloOcean GPU 声纳无法同时满足，则先保证定位环实时，并降低稠密建图更新率。

### 10.10 Demo 叙事与 claim 边界

最终演示不只播放点云，而应展示一个可观察的状态变化：

```text
清水：stereo/VIO 主导，地图细节高
  -> 浊度升高：visual health 下降，visual information 降权
  -> sonar registration 通过质量门：声纳因子限制水平漂移
  -> 双目深度失效：sonar-grounded 分支维持粗几何
  -> 视觉恢复：系统进入 recovery，连接新旧 submap
  -> 回环：map -> odom 更新，optimized map 恢复全局一致性
```

HoloOcean 可以证明实时 pipeline、可控退化、工程闭环、消融和故障恢复，但不能单独证明真实海域鲁棒性。后续论文或产品 claim 需要接入 RUSSO/Tank/SonarSweep/Opti-Acoustic 等真实或半真实数据，并最终增加自有水池/海试验证。

### 10.11 对现有知识体系的继承

- Factor Graph 在水下声光融合 SLAM 中的阐述框架（外部知识库资料）：采用异构因子、可观测性边界、information scheduling。
- SLAM 与水下 SLAM 知识体系（外部知识库资料）：把 camera/sonar 看作产生不同物理约束的前端，不做表层图像拼接。
- RUSSO（外部知识库资料）：采用视觉退化触发的 sonar motion anchor，但把硬阈值逐步升级为双模态质量审计。
- Pose-Graph FLS SLAM（外部知识库资料）：对 elevation 不可观维度膨胀 covariance，跳过信息贫乏关键帧，用局部 registration 结果注入全局图。
- Opti-Acoustic Turbid Recon（外部知识库资料）：复用 CFAR、DBSCAN、beam-region projection 和高浊度声纳地基重建，同时限制其近距离/前景/准平面 claim。
- OASIS（外部知识库资料）：借鉴增量 voxel/submap 和实时预算，但不接受单一固定体素分辨率作为高质量最终地图。
- Sonar-MASt3R（外部知识库资料）：借鉴近期最佳关键帧 recovery、断开子图和声学 metric anchor；不直接继承其固定基、小工作空间和基础模型依赖。
- 仿真 + 声光融合 SLAM 难点清单（外部知识库资料）：保留 P0 time/TF/schema 审计、GT 输入隔离、sim-to-real claim 边界和分层失败日志。

## 11. 代码审计修订附录：2026-08-18

> 本节基于对 `SVIn`、`sonar_camera_reconstruction`、`ocean_t` 三个仓库的逐文件代码审计（此前多处判断仅基于 README 或早期抽样审阅）。完整分析见[平台架构设计第 22 节](../acoustic-optic-slam-platform-architecture-2026-08-17.md#22-2026-08-18-三方代码库审计与架构细化)，本节只列出对本文 Phase 划分和执行顺序有直接影响的修订；这些修订优先于正文中的早期事实假设。

### 11.1 对本文既有判断的修正

1. SVIn ROS 版本：确认 `main` 分支为 ROS2 Jazzy（非 Humble），第 3.3/4.4 节的表述据此收敛。
2. SVIn sonar/depth 现状：并非"大量待实现"。Ceres 残差层（`SonarError`/`SonarParameterBlock`/`addSonarMeasurement`）已完整实现，main 分支只是 ROS2 `Subscriber.cpp` 里的 sonar/depth 回调被注释；`ros1` 分支的 sonar 回调实际是激活的（订阅 `/imagenex831l/range`）。第 4.4 节 Phase 2/3 的"审计"工作量应下修，Phase 3 的"HoloOcean imaging sonar 降维成 Imagenex range-like observation"判断依然成立，但对接目标从"探索是否可行"变为"补一层 ROS2 消息桥接"这样更具体的工程任务。Depth 因子在两个分支都未接线，工作量不变。
3. `sonar_camera_reconstruction` 构建依赖：`package.xml` 声明依赖同实验室 `bruce_slam` 包，不能独立编译。第 4.5 节 "Day 3: 跑 sonar_camera_reconstruction sample bag" 之前必须先插入一步解决该依赖，原时间表偏乐观。
4. `sonar_camera_reconstruction` 姿态处理：`merge.py` 的点云旋转显式丢弃 pitch，只用 roll+yaw；`merge.launch` 还隐藏发布一条 `map -> odom` 恒等 static TF。第 10.4.6 节"自适应双建图前端"里 sonar-grounded 分支的适用范围应明确限定为近似水平姿态场景。
5. `ocean_t` 现状：`ocean_t/src/svin2_pipeline.py` 并非对接官方 SVIn，而是团队自研的 Python/scipy 简化紧耦合估计器原型，其 FLS 前端对 elevation 使用 `np.random.uniform` 随机赋值后当真实 3D 点使用，直接违反平台架构第 6/21 节的不变量。这意味着本文 Phase 2（"跑通 SVIn 分支"）尚未真正开始。`ocean_t` 里现有的紧耦合原型不能等同于 Phase 2 的交付物，且该原型需要先整改（去随机化 elevation、修复每帧重新播种导致的不可复现问题）才能作为任何契约验证的参考。

### 11.2 对执行顺序的影响

- 第 4.5 节"第一周执行顺序"中 Day 3 之前需插入 `bruce_slam` 依赖处理。
- 第 10.8 节 Milestone B（原始松耦合 baseline）应明确使用**官方 SVIn**跑通，与 `ocean_t` 现有的自研原型分开记录，避免验收时把原型的输出误当作 SVIn baseline。
- 第 10.8 节 Milestone C（声纳参与定位）的技术路线不变（仍走 pose-graph 层级的路线 B），但如果后续要评估路线 C（SVIn 原生紧耦合）的可行性，可参考平台架构文档第 22.2 节新增的限时 spike 建议，其成本已被证明低于本文最初预估。
