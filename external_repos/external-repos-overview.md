---
title: 外部参考仓库与桥接代码概览
type: external-code-audit
status: current-reference
updated: 2026-08-19
verified_against: 919e1f0
---

# 外部参考仓库与桥接代码概览

> **用途与边界**：本文帮助开发者理解 `external_repos/` 中各来源的角色、接口、
> 验证范围和移植风险。所有子目录都是只读参考；恢复方法见
> [external_repos 恢复说明](./README.md)，已经移植的代码出处以 [`NOTICE`](../NOTICE)
> 为准。本文对六个本地目录给出角色摘要，只对 `holoocean-ros` 与
> `holoocean_bridge` 提供逐模块代码说明。

## 阅读导航

- [六个外部目录的角色摘要](#外部参考仓库简介)
- [`holoocean-ros` 详细说明](#holoocean-ros-代码简介)
- [`holoocean_bridge` 详细说明](#holoocean_bridge-代码简介)
- [仓库恢复与快照](./README.md)
- [移植来源与许可证](../NOTICE)

## 外部参考仓库简介

### `HoloOcean`

`HoloOcean` 是基于 Unreal Engine 的水下机器人仿真器本体，提供 Python API、载体
动力学、环境和相机/IMU/DVL/深度/多种声呐传感器。当前本地快照为 `49e70552`。
非 UE 代码采用 MIT 许可，UE 派生代码和资产还受 Epic Unreal Engine EULA 约束。

在 `uw_slam` 中，它是仿真器行为与 Python API 的只读参考；正式数据边界由
`adapters/holoocean` 或 `holoocean-ros` 提供，而不是让算法模块直接依赖仿真器源码。

### `holoocean-ros`

`holoocean-ros` 是 HoloOcean 官方提供的 ROS 2 接口包，用于把 HoloOcean 海洋机器人仿真器接入 ROS 2 网络。它从 JSON 场景配置创建并推进仿真环境，将相机、成像声呐、DVL、深度计等传感器数据发布为 ROS 2 话题，同时通过话题和服务接收载体控制、传感器调整、环境重置及控制模式切换命令，并发布 `/clock` 仿真时间。

仓库主要包含三个 ROS 2 包：

- `holoocean_main`：仿真核心节点，负责加载场景、推进环境以及完成 HoloOcean 数据与 ROS 2 消息之间的转换。
- `holoocean_interfaces`：自定义消息与服务定义，包括 `ImagingSonar`、`AgentCommand` 和 `SetControlMode` 等。
- `holoocean_examples`：摇杆控制、航点跟踪、深度/航向/速度控制等使用示例。

该仓库适合作为标准化的“仿真器—ROS 2”桥接层。它支持 ROS 2 Humble 和 Jazzy，采用 MIT 许可证；在当前 `uw_slam` 工程中，它是实际使用的 HoloOcean 仿真入口，其成像声呐话题由项目适配层接入后续 SLAM 流水线。

### `ocean_t`

`ocean_t` 是团队早期开发的 HoloOcean AUV 多传感器实验原型。它不经过 ROS 2，而是直接调用 HoloOcean Python API，在单体 Python 应用中完成仿真控制、传感器数据采集、实时显示和 SLAM 实验。

其主要模块包括：

- `src/main.py`：启动场景、键盘控制 AUV，并显示和录制相机、声呐、IMU、DVL、GPS、深度及位姿数据。
- `src/recorder.py`：按采集会话保存 CSV、NPY、PNG 和元数据。
- `src/water_control_panel.py`：通过 PyQt5 调整水体颜色和水下雾效。
- `src/svin2_pipeline.py`：早期的声—视—惯 SLAM 实验管线，包含多传感器时间对齐、视觉里程计、声呐点云、因子图优化和回环检测，并输出 PCD 地图及 TUM 格式轨迹。

该仓库更适合回顾早期实验流程、数据格式和算法原型，不宜作为当前系统的集成入口。它来自团队内部 GitLab，目前不再维护，相关职责已由 `uw_slam` 中的 HoloOcean 与 ROS 2 适配层取代；仓库根目录当前也未提供独立许可证文件。

### `SVIn`

`SVIn` 是面向水下实时导航的紧耦合声呐—视觉—惯性—深度 SLAM 系统。它在 OKVIS 视觉惯性里程计基础上加入声呐量程和深度信息，使这些观测与相机、IMU 状态在同一优化框架中联合估计，并通过后端回环检测和位姿图优化抑制长时间运行产生的轨迹漂移。

仓库主要包含两个模块：

- `okvis_ros`：由 OKVIS 改造而来的多传感器紧耦合前端与滑动窗口优化模块，包含相机、IMU、声呐及深度观测的同步和残差模型。
- `pose_graph`：基于 BRIEF/DBoW2 的回环候选检索、几何验证和位姿图优化模块。

当前主分支已迁移到 ROS 2 Jazzy，但上游 README 明确说明旧声呐自定义消息尚未完全适配，声呐和深度模式默认关闭，因此不能把“可编译的 ROS 2 主分支”等同于“完整声—视—惯—深度功能已经可用”。在当前 `uw_slam` 工程中，`SVIn` 是只读算法参考，主要移植了其 `SonarError` 声呐量程残差公式，并不是直接构建或运行依赖。仓库采用 GPLv3 许可证。

### `sonar_camera_reconstruction`

`sonar_camera_reconstruction` 是论文 *Opti-Acoustic Scene Reconstruction in Highly Turbid Underwater Environments*（2025）的配套代码，目标是在光学图像受浑浊水体影响时，融合单目相机、成像声呐和机器人里程计，生成水下场景三维点云。

其处理流程可以概括为：

1. 对成像声呐执行 SOCA-CFAR 目标检测，将极坐标声呐图转换到笛卡尔坐标，并进行离群点过滤、降采样和 DBSCAN 聚类。
2. 对单目图像进行自适应阈值处理和区域分割，提取可能的目标区域。
3. 利用声呐—相机外参和相机内参，把声呐候选点投影到图像中进行跨模态关联，再结合里程计将结果变换到地图坐标系并累积为点云。

当前检出的代码是 ROS 1 Noetic 版本，核心包为 `sonar_camera_reconstruction_pkg`，另有 `sonar_oculus` 定义 Oculus 成像声呐消息；上游同时维护单独的 ROS 2 分支。在当前 `uw_slam` 工程中，该仓库同样是只读算法参考，声呐前端参考并移植了其中的 CFAR 检测、极坐标转换和 DBSCAN 聚类思路。仓库采用 MIT 许可证。

### 六个本地目录的定位

| 目录 | 角色与来源 | 许可证 | 快照/可获取性 | 在 `uw_slam` 中的用途 | 已验证边界 |
|---|---|---|---|---|---|
| `HoloOcean` | BYU 官方仿真器本体；公开 GitHub | 非 UE 代码 MIT；UE 部分受 Epic EULA 约束 | `49e70552` | 仿真器行为与 Python API 参考 | 本地源码审阅；未在本机运行 UE5 仿真 |
| `holoocean-ros` | BYU 官方 ROS2 接口；公开 GitHub | MIT | 本地副本没有 `.git` 元数据 | 当前 ROS2 仿真入口与 `ImagingSonar` 消息来源 | 主仓库桥接节点已编译/独立启动；未接真实数据流 |
| `ocean_t` | 团队早期单体实验原型；内部 GitLab | 未提供独立许可证 | `f9333e94`；仅局域网可取 | 历史采集、坐标与算法原型审计 | 只读代码审计，不作为当前入口 |
| `SVIn` | 水下多传感器 SLAM；公开 GitHub | GPLv3 | `fcda5466` | `SonarError` 残差来源与 VIO baseline 参考 | 残差移植有单测；当前 provider 未连接真实 SVIn |
| `sonar_camera_reconstruction` | 声光场景重建论文代码；公开 GitHub | MIT | `9acc41df` | CFAR、极坐标转换与 DBSCAN 来源 | 移植前端有单测；上游完整 baseline 未在本仓库跑通 |
| `holoocean_bridge` | 同事维护的旧版兼容层；无公开地址 | 未确认 | 本地副本没有 `.git` 元数据 | 消息、TF、相机参数和旧启动链参考 | 逐文件审阅；无配套测试和 live 验证 |

简而言之，`HoloOcean` 产生仿真世界，`holoocean-ros` 把数据送入 ROS2，
`holoocean_bridge` 兼容旧版下游程序；`ocean_t` 记录早期单体实验，`SVIn` 提供状态估计
参考，`sonar_camera_reconstruction` 提供声呐前端和声光点云重建参考。

---

## `holoocean-ros` 代码简介

### 项目定位

`holoocean-ros` 是 HoloOcean 的 ROS 2 接口包，用于把基于 Unreal Engine 5 的海洋
机器人仿真接入 ROS 2。它本身不实现 SLAM，而是为上层算法提供相机、IMU、DVL、
深度计和成像声呐等仿真数据，同时接收载体及传感器控制指令。

基本数据流如下：

```text
ROS 2 控制指令
      ↓
holoocean_node
      ↓
HoloOcean / Fossen 动力学仿真
      ↓
传感器数据编码
      ↓
ROS 2 Topics + /clock
```

### 目录结构

该仓库主要包含三个 ROS 2 包：

- `holoocean_main`：核心桥接节点。读取场景 JSON，创建 HoloOcean 环境，在后台线程
  中持续推进仿真，并发布传感器数据。
- `holoocean_interfaces`：自定义消息和服务，包括 `AgentCommand`、
  `SensorCommand`、`ImagingSonar`、DVL 波束距离消息和控制模式切换服务。
- `holoocean_examples`：手柄控制、多载体控制、航点跟踪、深度/航向/速度控制和
  相机 HUD 等示例节点。

此外：

- `config/` 和 `holoocean_main/config/` 保存载体及仿真场景配置。
- `docker/` 提供开发和运行环境的 Docker 配置。
- `media/` 保存 README 使用的演示图片。

### 核心代码

#### ROS 2 主节点

[`holoocean_node.py`](holoocean-ros/holoocean_main/holoocean_main/holoocean_node.py)
定义 `HoloOceanNode`，主要负责：

- 声明并读取场景路径、视口、渲染质量等 ROS 参数；
- 订阅载体、传感器、深度、航向和速度控制指令；
- 提供重置仿真及切换控制模式的服务；
- 在后台线程中逐步推进仿真；
- 发布 ROS 仿真时钟 `/clock`。

#### HoloOcean 封装层

[`holoocean_interface.py`](holoocean-ros/holoocean_main/holoocean_main/interface/holoocean_interface.py)
封装 HoloOcean Python API，负责：

- 加载场景并调用 `holoocean.make()` 创建仿真环境；
- 处理单载体和多载体场景；
- 接入 Fossen 水下航行器动力学与自动驾驶控制；
- 在每个仿真步执行控制、调用 `env.tick()` 并发布传感器数据；
- 处理深度、航向、速度、执行器及传感器旋转命令。

#### 传感器消息编码

[`sensor_data_encode.py`](holoocean-ros/holoocean_main/holoocean_main/interface/sensor_data_encode.py)
把 HoloOcean 返回的 NumPy 数据转换为 ROS 消息。目前覆盖的主要传感器包括：

- IMU 和 IMU bias；
- DVL 速度和四波束距离；
- 深度、位置、姿态、GPS 和动力学真值；
- RGB 相机、磁力计和激光测距；
- GPU/Raycast 成像声呐。

成像声呐数据会被展平为 `holoocean_interfaces/ImagingSonar` 消息，这也是当前
`uw_slam` 声呐处理链路关注的主要输入之一。

### ROS 2 接口

节点通常运行在 `/holoocean` 命名空间下。传感器话题名称采用：

```text
/holoocean/<agent_name>/<sensor_name>
```

例如：

```text
/holoocean/auv0/IMUSensor
/holoocean/auv0/DVLSensorVelocity
/holoocean/auv0/GPUImagingSonar
```

主要输入包括：

- `command/agent`：执行器或推进器控制；
- `command/sensor`：传感器旋转等控制；
- `depth`、`heading`、`speed`：Fossen 自动驾驶目标值；
- `debug/points`：在仿真环境中绘制调试点。

主要服务包括：

- `reset`：重置仿真环境；
- `control_mode`：切换手动、深度、航向等控制模式。

### 启动方式

在 ROS 2 工作空间中构建并加载环境后，可通过下面的命令启动：

```bash
ros2 launch holoocean_main holoocean_launch.py
```

默认启动文件会读取 `holoocean_main/config/ros_params.yaml`，该参数文件再指定具体
的场景 JSON。

### 与 `uw_slam` 的关系

在当前工程中，`holoocean-ros` 是仿真数据源和 ROS 2 接口层，而不是 SLAM 算法层。
它生成声呐、视觉、惯性、DVL 和深度等数据，随后由主仓库中的 HoloOcean/ROS 适配器
接入声光融合 SLAM 管线。

按照主仓库约定，`holoocean-ros/` 是只读的第三方上游代码，不应直接修改；需要的
功能应通过主仓库中的 adapter 接入，或在合规情况下移植实现。

### 已知限制与验证边界

- 当前本地副本没有独立 `.git` 元数据，不能从目录本身确认确切 upstream commit；恢复
  或更新时应记录快照。
- `uw_holoocean_sonar_bridge_node` 已完成 ROS2 Jazzy 编译、链接和独立启动验证，但
  没有连接真实 `holoocean_main`/UE5 数据流。
- `holoocean_interfaces` 不在公共 ROS2 包索引中，需要在仓库外的 colcon workspace
  单独构建；这些生成物不属于 `uw_slam` 版本控制范围。
- 上游启动命令只是参考，不代表本仓库已验证完整仿真、控制和所有传感器话题。

---

## `holoocean_bridge` 代码简介

### 项目定位

`holoocean_bridge` 是建立在官方 `holoocean-ros` 之上的二次适配包。它不负责实现
HoloOcean 仿真引擎，而是把官方 ROS 2 接口输出的传感器数据转换成 SVIn 和声呐重建
程序能够直接消费的格式。

主要数据流如下：

```text
holoocean-ros
  ├─ ImagingSonar
  │      ↓
  │  sonar_adapter
  │      ↓
  │  sonar_oculus/OculusPing
  │      ↓
  │  sonar_camera_reconstruction
  │
  ├─ 左/右相机 + IMU
  │      ↓
  │     SVIn
  │
  └─ FrontCamera
         ↓
     CompressedImage
         ↓
     声光融合着色/重建
```

### 与官方 `holoocean-ros` 的区别

| 对比项 | `holoocean-ros` | `holoocean_bridge` |
|---|---|---|
| 定位 | 官方通用 ROS 2 接口 | 面向已有 SLAM 程序的兼容层 |
| 主要任务 | 推进仿真、发布原始传感器数据 | 消息转换、TF、相机标定和流程编排 |
| 声呐输出 | `holoocean_interfaces/ImagingSonar` | `sonar_oculus/OculusPing` |
| 通用程度 | 支持多种载体和场景 | 主要针对 `auv0`、SVIn 和声呐重建基线 |

因此，两者不是相互替代关系，而是上下游关系：`holoocean_bridge` 依赖官方
`holoocean_main` 和 `holoocean_interfaces`，并在其输出之上补充下游所需的兼容逻辑。

### 核心模块

#### 声呐消息适配

[`sonar_adapter_node.py`](holoocean_bridge/holoocean_bridge/sonar_adapter_node.py)
是该项目最关键的节点，负责把 HoloOcean 的成像声呐输出转换成
`sonar_oculus/OculusPing`：

1. 订阅 `/holoocean/auv0/ImagingSonar`；
2. 从 `image_range` 展平数组恢复二维声呐强度图；
3. 沿方位角方向左右翻转，使声呐图与 bearings、点云和相机画面方向一致；
4. 把 `[0, 1]` 浮点强度转换成 `uint8` 并进行 JPEG 压缩；
5. 根据场景配置生成 bearings、量程和距离分辨率；
6. 发布 `/sonar_oculus_node/M750d/ping`，供声呐重建包消费。

#### 坐标转换与传感器外参

[`coord_transform.py`](holoocean_bridge/holoocean_bridge/coord_transform.py)
从 HoloOcean 场景 JSON 中提取相机、声呐和深度计配置，构建齐次变换矩阵，并处理
HoloOcean/Unreal 与 ROS 坐标约定之间的转换。

它还提供：

- 传感器坐标到载体坐标的点变换；
- 载体坐标到世界坐标的点变换；
- 位姿、旋转矩阵和 SE(3) 矩阵之间的转换；
- 相机内参矩阵与双目基线信息。

[`tf_publisher_node.py`](holoocean_bridge/holoocean_bridge/tf_publisher_node.py)
使用这些外参发布静态 TF，包括：

```text
base_link → CameraLeftSocket
base_link → CameraRightSocket
base_link → CameraFrontSocket
base_link → sonar_socket
base_link → DepthSocket
base_link → IMUSocket
```

#### 相机接口适配

[`camera_info_node.py`](holoocean_bridge/holoocean_bridge/camera_info_node.py)
根据场景配置生成 `sensor_msgs/CameraInfo`。代码假定 HoloOcean 使用无畸变针孔模型，
并采用：

```text
fx = fy = width / 2
cx = width / 2
cy = height / 2
```

[`image_to_compressed_node.py`](holoocean_bridge/holoocean_bridge/image_to_compressed_node.py)
通过 `cv_bridge` 和 OpenCV 把官方接口发布的原始 `sensor_msgs/Image` 转换为
`sensor_msgs/CompressedImage`。默认把前视相机图像发布到：

```text
/camera/image_raw/compressed
```

供后续声光融合重建进行点云着色或图像关联。

#### 载体控制与辅助节点

[`teleop_keyboard_node.py`](holoocean_bridge/holoocean_bridge/teleop_keyboard_node.py)
提供 HoveringAUV 键盘控制，把前进、侧移、偏航和升沉指令混合成八个推进器命令：

```text
W/S：前进/后退
A/D：左移/右移
Q/E：左转/右转
R/F：上浮/下潜
空格：紧急停止
```

此外还包含：

- `pose_display_node.py`：显示 HoloOcean 真值位置和姿态；
- `recorder.py`：把 IMU、DVL、位姿等数据写入 CSV，并把相机和声呐帧保存为
  PNG/NPY；
- `scripts/eval_svin_holoocean.py`：从 rosbag2 提取 SVIn 和真值轨迹，生成 TUM
  格式数据；
- `scripts/compute_ts_c.py`：计算声呐到前视相机的外参；
- `scripts/query_socket_poses.py`：查询 HoloOcean 各个 socket 的实际位姿；
- `scripts/move_auv_test.py`：测试 HoveringAUV 八推进器运动控制。

### 场景配置

默认场景见
[`holoocean_scenario.json`](holoocean_bridge/config/holoocean_scenario.json)，主要配置为：

- 环境：`Dam`；
- 载体：`HoveringAUV`，名称为 `auv0`；
- 仿真频率：100 Hz；
- 渲染频率：30 FPS；
- 传感器：位置、姿态、IMU、DVL、深度计、左/右/前视相机和 Raycast 成像声呐；
- 声呐：512 个距离 bin、256 个方位 bin、0.5–30 m 量程、120° 水平视场角。

目录中的 `holoocean_scenario_01.json`、`02.json` 和 `03.json` 是其他场景或实验配置。

### 启动方式

#### 只启动 HoloOcean 和适配节点

```bash
ros2 launch holoocean_bridge holoocean_bridge.launch.py
```

该启动文件会运行：

- 官方 `holoocean_main` 节点；
- 声呐适配节点；
- 静态 TF 发布节点；
- 前视相机图像压缩节点；
- 左、右、前三个相机的 `CameraInfo` 节点。

#### 启动旧版完整基线

```bash
ros2 launch holoocean_bridge pipeline.launch.py
```

该启动文件按顺序拉起：

1. HoloOcean 仿真及桥接；
2. SVIn 定位节点；
3. `sonar_camera_reconstruction` 的融合重建和 RViz。

SVIn 会延迟约 10 秒启动，以等待 HoloOcean 引擎和相关传感器话题就绪。

键盘控制需要交互式终端，应单独运行：

```bash
ros2 run holoocean_bridge teleop_keyboard
```

### 已知限制与风险

这是同事维护的原型适配包，不是正式发布的通用组件，使用或移植时需要注意：

- 没有公开的 clone 地址，也没有项目 README；
- `package.xml` 和 `setup.py` 中的许可证仍为 `TODO`，移植前必须确认授权；
- 多处默认绑定 `auv0`、固定话题名和固定传感器 frame；
- 键盘控制逻辑针对 HoveringAUV 的八推进器布局；
- 相机内参使用简化的无畸变针孔模型；
- 坐标转换、双目外参以及声呐左右翻转都属于高风险标定细节，不能未经验证直接复用；
- 当前没有发现配套的自动化测试目录。

其中 `CoordTransformer.get_all_extrinsics()` 当前让右相机复用左相机变换矩阵，因此它
不能被视为已经完成验证的精确双目标定结果。

### 与当前 `uw_slam` 的关系

`holoocean_bridge` 主要服务于旧版 SVIn 与 `sonar_camera_reconstruction` 基线，展示了
如何把官方 HoloOcean 话题转换为这些历史程序需要的格式。

当前 `uw_slam` 新架构并不直接依赖这个包，而是在主仓库自己的 HoloOcean 和 ROS 2
adapter 中实现正式接入。这个目录的主要用途是：

- 参考旧链路使用的话题和消息格式；
- 核对声呐图像翻转、压缩和 bearings 生成逻辑；
- 参考历史 TF、相机内参及声呐—相机外参处理；
- 运行和对比旧版 SVIn/声呐重建基线。

按照主仓库约定，`holoocean_bridge/` 是只读参考目录，不应直接修改。
