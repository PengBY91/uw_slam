# ROV 到货前准备第四周推进设计

## 目标

在不依赖真实 ROV、HoloOcean Windows 主机或尚未到货设备的前提下，推进第四周计划中可本地验证的部分，并为必须在 Windows/HoloOcean 上执行的部分留下可复现的运行入口和明确的待验收项。

## 范围

本轮覆盖第四周排期中的全部任务：

- PREP-B-01：注册 `imu_preintegration` 估计模式，接入回放管线，补充 IMU 合成输入和端到端 demo。
- PREP-B-02：实现带电流门控的绝对航向因子及回放接入。
- PREP-C-03：实现 MAVLink 遥测输入、设定值命令输出、外部导航回灌和 fail-closed 测试所需的本地能力。
- PREP-A-13：先完成 MAVLink 与 IMU 两路设备伪装流；声呐和相机伪装流保留接口边界，不提前假定厂家格式。
- PREP-A-09：完成相机退化模型/离线评估工具；真实 HoloOcean 指标留待 Windows 运行。
- PREP-A-08：完成水池关卡的可加载配置、任务评分/稀疏控制点评测接入；真实 UE5 打包加载留待 Windows 验收。
- PREP-A-12：完成手柄到 `PilotCommand`/设定值的训练入口、录制和评分编排；真实飞手训练留待 HoloOcean 主机。

不在范围内：修改 `external_repos/`、引入 DVL、修改 ArduSub 固件、声呐 SDK 私有实现、双目相机采购决策，以及清理或覆盖已有工作区改动。

## 实施顺序

采用“可验证优先”顺序：

1. 先完成 B-01 的配置选择器、回放分支、合成 IMU 输出和端到端 demo，建立算法验收基线。
2. 完成 B-02 航向因子，并用合成/回放测试验证正常航向、高电流跳过和 anchor 初值行为。
3. 完成 C-03 的 MAVLink adapter 契约和离线/模拟链路测试。
4. 完成 A-13 的 MAVLink/IMU 伪装流，复用 C-03/D-03 的 wire format；真值只走独立评分通道。
5. 完成 A-09、A-08、A-12 的本地脚本、配置和测试；把真实 HoloOcean/UE5/手柄运行列为外部验收。
6. 更新规格文档、仿真规格和 traceability，逐项记录“已本地验证”与“待 Windows/HoloOcean 验证”的差异。

## 设计要点

### 算法链路

`estimator_mode: imu_preintegration` 只改变相对位姿证据来源，不改变现有 `Pose3` 图的公共接口。回放管线按时间顺序聚合 IMU 样本，在关键帧之间生成预积分量测，并与已有深度、声呐 range 因子共同求解；纯视觉路径必须保持原有行为。

航向因子只在残差内部从四元数计算绕重力轴的 yaw 差，不引入欧拉角状态。σ 由基础噪声和推进器电流计算；超过门限的观测直接跳过并记录计数。anchor 的 yaw 使用首个有效航向观测初始化，避免与绝对先验冲突。

### 设备链路

MAVLink 和 IMU 伪装流遵循“同一套真机 adapter、不同后端”。伪装流只输出设备线上格式，不携带真值；时间戳使用仿真时钟字段。MAVLink 路优先复用已完成的 ArduSub SITL 桥，无法运行真实 HoloOcean 时使用离线/模拟 transport 验证消息契约、命令方向和 fail-closed 行为。

### 仿真与训练

相机退化、测试水池和飞手训练均以现有 manifest、`PilotCommand`、录制器和评分器为边界，避免新增一套旁路格式。Windows 侧只能验证的项目必须提供命令、输入配置、预期输出和失败诊断；本机测试不得把“脚本可运行”误报为“真实 HoloOcean 已验收”。

## 验收与验证

- C++：编译、全量 CTest、`synthetic_smoke` 回归、IMU-only demo ATE ≤ 0.15 m。
- Python：`adapters/holoocean` 单测，新增伪装流/退化/训练编排测试。
- 架构：`tools/lint/check_layer_dependencies.py` 和实时 traceability 检查。
- 文档：第四周每项任务明确状态、证据路径、外部待验收条件。
- 外部验收：Windows HoloOcean 5 分钟/30 分钟运行、UE5 水池关卡加载、相机退化指标、飞手完整任务训练分别保留为待执行项，除非当前环境实际提供对应能力。

## 风险控制

- 共享配置或接口改动先通过现有测试锁定行为，再实现新分支。
- 不将尚未验证的 HoloOcean 动力学、传感器噪声或厂家协议写成已确认事实。
- 保持已有用户改动，所有修改使用小范围补丁；不进行重置、清理或提交。

