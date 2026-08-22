# uw_slam 生产就绪度审计与阶段路线图

> 状态：**当前审计基线 + 粗粒度执行路线**  
> 核对日期：2026-08-21  
> 核对范围：当前工作区源码、构建与测试工具、合成数据回放，以及本机保留的一份真实
> HoloOcean 录制。当前工作区含未提交修改，因此本文记录的是“审计时刻的实际状态”，
> 不是某个已发布版本的能力声明。

本文回答两个问题：当前仓库距离“水下声光融合、SLAM、重建研发团队可长期依赖的生产
与测试工具链”还有多远，以及应该按什么顺序补齐。它不替代
[长期架构设计](./acoustic-optic-slam-platform-architecture-2026-08-17.md)；架构文档定义
长期边界，本文负责记录当前差距、阶段优先级和验收门。

## 1. 执行摘要

当前代码不需要推倒重来。Protobuf 领域契约、MCAP 回放、模块依赖边界、确定性测试和
若干真实算法切片已经构成了有价值的工程底座。主要问题不是架构方向错误，而是架构层
明显领先于实现层：schema 和接口已经描述了接近生产系统的能力，在线运行、真实数据、
估计后端、稠密重建和质量门禁仍主要停留在骨架或合成验证阶段。

按不同目标衡量，当前成熟度约为：

| 目标层级 | 当前成熟度 | 含义 |
|---|---:|---|
| 可扩展研究代码骨架 | 65%–75% | 分层、契约、合成垂直切片和基础测试已经成立 |
| 团队内部稳定研发/测试平台 | 30%–40% | 可以开发和验证小模块，但真实数据、统一基准和自动门禁不足 |
| 可长期在线运行的集成原型 | 20%–30% | ROS2、调度、故障恢复和资源预算尚未形成闭环 |
| 现场可部署的生产系统 | 15%–25% | 真实环境鲁棒性、重建质量、运维与海试证据仍缺失 |

因此，距离团队生产与测试工具链仍有约 60%–70% 的关键工作，而且剩余工作集中在最依赖
真实数据、系统集成和长期验证的部分。

> 以上百分比是审计者基于第 3 节分项评分和第 6 节六项完成定义的主观整体判断，
> 不是从某个自动化指标（如测试覆盖率）折算出来的，区间宽度本身就反映了这种不
> 确定性。用于团队内部对齐足够，不建议在没有说明口径的情况下对外引用这些数字。

## 2. 本次审计的实际证据

### 2.1 构建与测试

| 检查 | 审计结果 | 解释 |
|---|---|---|
| `cmake --build build -j2` | 通过 | 当前 C++ 工作区可以完整构建 |
| `ctest --test-dir build --output-on-failure` | 112/112 通过 | 合同、单元、合成集成和依赖 lint 均通过 |
| `tools/lint/check_no_ros_in_core.sh` | 通过 | C++ 分层依赖及 ROS/vendor 隔离仍成立 |
| HoloOcean Python pytest（`uv run pytest`） | 35/35 通过 | 复核：本文写作时描述的 1 个失败（`test_record_session.py` 仍用旧式正值 fixture）实际已在
  `79d9be5`（Fix real-data VO pipeline: depth sign bug...）中随深度符号修订一并收口；重新
  跑一遍工作区确认当前不存在这个失败，下方一段按已收口改写，不再是待办 |
| `git diff --check` | 通过 | 已跟踪改动未发现空白错误 |

> 复核结论：深度符号 fixture 不一致已经修复（见 `test_record_session.py` 里
> `DepthSensor: np.array([-5.0])` → `depth_m == 5.0` 的负入正出约定，与
> `test_state_conversion.py` 一致）。C++/Python/lint 三项此刻都是全绿——这意味着
> P0 清单里"修复深度符号 fixture 不一致"这一条已经完成，不再需要单独执行；P0
> 剩余工作是本节其余的 gate/manifest/CI 收口项。

### 2.2 合成回放

| 路径 | 结果 |
|---|---|
| `black_box_vio` 合成回放 | 12 个 keyframe、36 个声呐 range factor、6 次迭代收敛，ATE RMSE `0.0666 m` |
| `stereo_landmark_vo` 合成回放 | 11 个 keyframe、33 个声呐 range factor、7 次迭代收敛，ATE RMSE `0.0608 m` |
| 光学立体深度 smoke | RMSE `0 m`、coverage `0.9289`，干净的常视差合成图通过 |

这些结果证明相机/声呐前端、因子构建、小规模位姿图、地图 evidence 和评测接口能够组合
运行。但两条回放中的稠密声光关联分别为 `0 accepted / 12 rejected` 和
`0 accepted / 11 rejected`。也就是说，定位图使用了声呐距离约束，但回放产生的数百万
地图点基本是光学深度 evidence；稠密重建没有在这条端到端路径中获得有效的声学修正。

### 2.3 九场景声光矩阵

审计使用固定 seed、每场景 5 次试验。主要现象包括：

- `low_texture_sonar_visible` 有 4/5 次关联接受；
- `turbid_sonar_visible` 为 0/5 接受，全部因为没有候选；
- 多个场景的 optical/full 与 fused/full RMSE 在打印精度下完全相同；
- 有效场景的 P95 处理延迟约为 278–308 ms，高于长期架构提出的 200 ms 初始目标；
- 程序仍返回成功。

当前场景矩阵只在接受数达到 5 时检查 false-fusion rate。困难场景接受数为 0 时不会失败；
确定性集成测试还会忽略矩阵程序自己的 gate 退出码。因此它目前主要保护"结果可重复"，
没有充分保护"算法有效"。

> **P0 执行中的复核**（在 `apps/acoustic_optic_scenario_matrix.cpp` 里补上"接受数
> 达到 0 时也必须失败"的最低覆盖 gate 后，用 `--seed 4242 --trials-per-scenario 20`
> 实测得到的完整 9 场景结果，比上面这份基线审计描述的更严重）：9 个场景里，
> `kSonarDropout`/`kOpticalInvalidRegion` 两个按设计就该 0 接受（分别是"无声呐"和
> "光学证据整体失效"消融，见 `acoustic_optic_scenarios.cpp` 的场景构造注释），
> 应排除在这条新 gate 之外；但剩下 7 个里只有 `low_texture_sonar_visible`
> （17/20）、`repeated_structure`（20/20）、`turbid_sonar_visible`（5/20）产生了非零
> 接受——**`clean_textured`（本该是最干净的基线场景）、`elevation_stress`、
> `time_offset_fault`、`extrinsic_perturbation` 全部 0/20**。`clean_textured` 20 次
> 全部落在 AMBIGUOUS（而不是 NO_CANDIDATE），根因定位到
> `src/frontends/acoustic_optic_associator.cpp:155-162`：候选打分只用声呐
> range/bearing 残差，不折算光学侧的纹理/置信度，遇到深度在 patch 内部保持均匀的
> "干净"目标块时，投影到相机的多个声呐弧采样点会落在几乎同分的候选像素上，
> 触发 `ambiguity_margin` 而被判 AMBIGUOUS——这更像是关联评分本身缺一个可用的
> 消歧信号，而不是这个场景的构造有问题。这是一处需要独立调查/可能需要改评分逻辑
> 的算法问题，不是能在 P0 顺手调个阈值了事的——因此新加的最低覆盖 gate 已经写进
> 代码但**没有**接入 `ctest`（接入的话现在会立刻常红，且用调阈值让它变绿属于本文
> 10 节风险表明确警告过的反模式）。P1/P2 阶段处理关联/建图相关工作时应该把这个
> 根因分析一并领走。

### 2.4 真实 HoloOcean 录制回放

本机保留的真实录制约 76 MB、50 个 keyframe，只包含双目图像、GT pose 和 depth，
没有声呐、IMU 或 DVL。用当前 `real_holoocean_vo.yaml` 回放得到：

- 49 条双目 VO 相对位姿；
- 50 条深度因子；
- 0 条声呐因子；
- 求解器运行 30 次后 `stalled`；
- 对齐后的 ATE RMSE 为 `0.5596 m`；
- 稠密地图点为 0。

这份数据说明真实双目 VO 已经能够产生连续相对位姿，但真实重建尚未跑通。当前相机模型
明确假设输入已经去畸变和极线校正，而真实 rig 包含畸变、非理想基线方向，录制路径也
没有形成与前端一致的 rectification contract。由于该 bag 未纳入版本化数据集，它只能
作为审计证据，尚不能作为团队长期回归基准。

## 3. 分项成熟度

| 维度 | 评分 | 已有基础 | 主要缺口 |
|---|---:|---|---|
| 架构与领域契约 | 7/10 | Protobuf、模块 DAG、边界 lint、typed evidence | 契约版本迁移、单位/符号约束和兼容策略不足 |
| 声呐前端 | 5/10 | CFAR、极坐标转换、DBSCAN、range factor | 真实声呐数据、registration、部分位姿协方差、环境自适应 |
| 光学 VO/VIO | 3/10 | 合成双目 VO、Harris/NCC/RANSAC | rectification、IMU 预积分、滑窗、边缘化、真实退化处理 |
| 声光融合定位 | 3/10 | range-only 因子进入位姿图 | 联合路标、可靠性调度、声呐 registration、消融证据 |
| 声光融合重建 | 2/10 | 像素级后验深度、点云 evidence handoff | 声学有效覆盖、稠密几何、地图融合、重积分和地图评测 |
| SLAM 后端 | 2/10 | 小规模 Eigen LM、FactorBuilder 接口 | 流形稀疏求解、鲁棒核、fixed-lag、协方差、回环/重定位 |
| 在线运行时 | 2/10 | bounded queue、状态机和 lane 原语 | scheduler、异步数据流、背压、降级、恢复、实时预算 |
| 真实数据与标定 | 3/10 | HoloOcean recorder、相机标定工具、canonical writer | 全传感器录制、版本化数据、time/TF audit、公开数据集 adapter |
| 测试与评测 | 4/10 | 112 个 CTest、确定性回放、场景矩阵 | 非放空门禁、真实 benchmark、地图质量、长稳和故障注入 |
| 工程生产化 | 2/10 | CMake、pytest、验证脚本 | CI、sanitizer、coverage、包发布、依赖锁定、完整 manifest |

## 4. 值得保留的工程资产

后续应继续沿用以下基础，而不是重建另一套平行框架：

1. Protobuf 作为跨语言唯一领域契约；
2. MCAP 作为 live/replay 统一证据载体；
3. `domain → core → algorithms/runtime/adapters → apps` 的单向依赖；
4. frontend、FactorBuilder、ResidualBlock 和 StateStore 的职责分离；
5. 显式 seed 和逐字节确定性回放；
6. 地图 evidence 保存在局部坐标系，后端修正通过 keyframe pose 传播；
7. 外部仓库只读、移植代码保留 provenance 和许可证记录；
8. 分层配置与实验入口。

这些资产使项目不需要架构重写。后续投入应优先填充实现和证据面，避免继续扩充只有
schema、enum 或文档而没有运行闭环的新抽象。

## 5. 主要差距

### 5.1 估计后端仍是契约验证器

当前求解器使用稠密线性代数和原始 7 参数位姿更新，再归一化四元数。它适合验证小规模
合成图和 FactorBuilder 接口，不适合真实长序列、滑窗和在线优化。缺少 SE(3) 流形更新、
稀疏求解、鲁棒核、联合速度/bias/路标、边缘化、协方差与可观测性诊断。

### 5.2 当前不是完整 VIO/SLAM

默认相对位姿来自 ground-truth+noise 桩；另一条路径是纯双目 VO，不消费 IMU。虽然
schema 已有 IMU preintegration、velocity、bias 和 sonar registration 类型，但没有对应
在线估计链。当前系统更准确的定义是“相对位姿先验 + depth + 声呐 range-only 的离线
批量位姿图”。

### 5.3 重建层主要是 evidence 交接

`SubmapManager` 当前只维护局部点云 evidence 并按最新位姿变换，没有 TSDF、surfel、
occupancy、mesh、遮挡/自由空间、地图裁剪或重积分实现。稠密声光融合只会尝试为少量
像素做后验深度修正，尚未形成声呐覆盖区域的稳定几何生成和置信度融合。

### 5.4 运行时原语尚未组成在线系统

`replay_demo` 仍是离线批处理：多次遍历 bag、构图后统一求解。状态机和 queue 没有被
实际 scheduler 消费；ROS2 声呐节点也只完成 transport，没有驱动前端、估计器和地图。

### 5.5 验收门禁可以在无有效输出时通过

目前主要集成测试验证程序运行和确定性，没有强制检查：

- solver 必须收敛；
- ATE/RPE 必须低于阈值；
- 困难场景必须产生最低有效关联覆盖；
- 融合必须相对 baseline 带来可量化收益；
- P95 latency 必须满足预算；
- 地图必须包含有效点且达到精度/完整度要求。

### 5.6 可复现记录尚未落地

本次生成的 RunManifest 中 git commit、配置/标定/model hash、OS、CPU、GPU、seed 和
起止时间均为空。C++ 外部依赖也存在直接跟踪上游 `main` 的情况。当前可以重复运行同一
工作区，但还不能可靠复原数周前或另一台机器上的一次实验。

## 6. 团队生产与测试平台的完成定义

达到“内部稳定研发/测试平台”至少需要同时满足以下六项：

1. **数据闭环**：camera、sonar、IMU、depth、GT 能以统一时间/坐标/标定契约录制并回放；
2. **算法闭环**：至少一条真实 VIO/SLAM + 声呐修正 + 重建路径能在固定真实数据上运行；
3. **评测闭环**：轨迹、融合、地图、延迟、资源和故障指标都有不可放空的自动门禁；
4. **在线闭环**：同一套 core 能被 replay 和 ROS2 live 路径驱动，且有背压、降级和恢复；
5. **可复现闭环**：每次运行绑定代码、配置、标定、模型、数据和环境版本；
6. **工程闭环**：CI、静态/动态分析、包构建、固定数据回归和长稳测试成为合入条件。

## 7. 阶段路线图

阶段间有依赖关系，但平台工程、数据准备和部分算法 spike 可以并行。下列周期按 3–5 名
有经验的算法/平台工程师估算。

### P0：收口当前基线与质量门（2–4 周）

**目标**：让当前能力成为可信、可重复、不会放空通过的研发基线。

主要工作：

- ~~修复深度符号 fixture 不一致，使 C++、Python、lint 全绿~~——复核已确认完成
  （见 2.1），当前工作区三项均为全绿；
- 在 schema/验证函数中明确深度正方向、坐标 frame、单位和 rectified 语义；
- 为 replay 增加求解收敛、最小匹配数、ATE/RPE 和非空地图 gate；
- 为场景矩阵增加最低有效覆盖、baseline 改善和 P95 latency gate；
- 让确定性测试保留矩阵真实 gate 失败，而不是统一忽略退出码；
- 填充 RunManifest 的代码、配置、标定、数据、seed、环境与起止时间；
- 将 MCAP 等外部依赖固定到 commit/tag；
- `.github/workflows/ci.yml`（`tools/verify_pipeline.sh`）已经在跑
  build/CTest/pytest/lint/合成 replay 全套——P0 不是从零建 CI，而是把上面几条
  非放空 gate 接进这条已有流水线，让它们在 CI 里真正失败，而不是本地手跑才发现。

阶段验收：

- 工作区所有测试和 lint 全绿；
- 人为让 solver stalled、地图为空或场景无有效关联时，相关 gate 必须失败；
- 同一 manifest 能定位到完整代码、配置、标定和数据版本；
- clean checkout 在受支持环境中可一次完成配置、构建和验证。

### P1：真实离线多传感器闭环（4–8 周）

**目标**：先把真实数据离线链路跑通，再扩展在线系统。

主要工作：

- 固定直线、转弯、小闭环三类 1–3 分钟 HoloOcean 场景；
- 录制包含双目、成像声呐、IMU、depth、GT 的 canonical MCAP；
- 完成 topic、capture/receive time、clock domain、TF 和标定 audit；
- 建立去畸变和双目极线校正，明确 raw/rectified 图像契约；
- 使实验配置中的 frontend、factor builder、estimator 和 map backend 选择真正生效；
- 增加至少一个公开水下数据集 adapter，验证 schema 不只适配 HoloOcean；
- 固定真实数据的 VIO-only、VIO+depth、VIO+sonar、full fusion 四组 baseline。

阶段验收：

- 三类固定轨迹均能从同一 bag 产出轨迹、因子统计、健康报告和非空地图；
- solver 在真实数据上必须收敛，不得 stalled——当前 50 帧真实录制回放已经是
  30 次迭代后 stalled（见 2.4），这条不能被"轨迹和地图都非空"掩盖过去，必须
  和 P0 对合成数据的收敛 gate 同等强制，否则重演 5.5 节批判的放空模式；
- 真实双目深度具有非零有效覆盖，不再出现 50 帧全部 0 地图点；
- 回放不得使用 GT 生成算法输入，GT 只进入 evaluator；
- 所有 baseline 结果和产物由 manifest 关联并纳入固定回归数据集；
- 无声呐或声呐失效时系统明确降级，不崩溃、不伪造声学收益。

### P2：可信 SLAM 后端与声学修正（8–16 周）

**目标**：从小规模位姿图验证器升级到真实序列可用的估计平台。

主要工作：

- 选择并接入 Ceres 或 GTSAM，使用 SE(3) 流形和稀疏线性求解；
- 实现 fixed-lag/sliding-window、边缘化、鲁棒核和异常因子管理；
- 接入成熟 VIO，或完整实现 IMU preintegration、velocity 和 bias 状态；
- 实现声呐 keyframe、registration、可观测子空间和 partial-pose covariance；
- 建立稳定 landmark track/association，评估联合路标优化；
- 实现 reliability scheduler、information cap 和跨模态冲突处理；
- 实现回环、断图恢复和重定位的最小闭环。

阶段验收：

- 固定真实数据集中不得出现未被 gate 捕获的 solver stall；
- 报告 ATE、RPE、drift、lost count/duration、relocalization time 和残差统计；
- 在 critical/severe 视觉退化片段，full fusion 相对 VIO-only 的局部 RPE 或 lost duration
  改善至少 20%；
- clean 场景中加入声呐后的轨迹指标退化不超过 5%；
- 四组消融均使用相同数据、配置基线和统一 evaluator。

### P3：重建后端与在线生产化（8–16 周，可与 P2 后半段并行）

**目标**：形成可在线运行、可评测、可恢复的声光重建系统。

主要工作：

- 选择 TSDF、surfel 或 occupancy 中一个作为首个正式几何后端；
- 实现 visual-only 和 sonar-grounded 两条局部几何路径；
- 实现 uncertainty-aware 融合、自由空间/遮挡处理和异常点抑制；
- 实现 pose correction 后的 submap transform/reintegration 与 stale evidence 管理；
- 将 bounded queue、四条 lane 和三个状态机接入实际 scheduler；
- 打通 ROS2 sensor gateway → frontend → estimator → mapping → evaluator/recorder；
- 增加 watchdog、背压、降级顺序、恢复和资源预算；
- 增加 ASan/UBSan/TSan、coverage、静态检查、包构建和 60 分钟 soak test。

阶段验收：

- 地图报告 accuracy/completeness、Chamfer/F-score、outlier ratio 和 loop discontinuity；
- localization 15–30 Hz、声学修正 1–5 Hz、地图输出 2–10 Hz；
- 目标硬件上 capture-to-pose/map P95 不超过 200 ms；
- camera/sonar dropout、时间延迟、packet drop 和队列过载均触发可观察降级；
- 60 分钟持续运行无崩溃、无无界内存增长，关键队列和 dropped frame 可追踪。

### P4：现场验证与发布门（持续 2–4 个月）

**目标**：把“仿真和固定数据可用”升级为“真实水下任务可依赖”。

主要工作：

- 水池、近岸和目标海况分层采集；
- clean/mild/critical/severe 浊度及不同声速/混响条件分桶；
- 标定重复性、温漂、时钟漂移和传感器重启测试；
- sim-to-real holdout、跨场景/跨载体验证和 failure mining；
- 发布包、升级/回滚、运行手册、故障手册和数据留存策略。

阶段验收：

- 预先冻结的 holdout 任务通过，调参数据与验收数据隔离；
- 至少两套独立标定/数据采集批次满足相同门限；
- 现场失败能由 bag、manifest、health 和日志完整复盘；
- 发布版本具有明确支持矩阵、已知限制和可回滚产物。

## 8. 建议团队配置与投入

建议最小核心配置：

| 角色 | 主要责任 |
|---|---|
| 技术负责人/SLAM 后端 | 状态建模、求解器、可观测性、回环与决策门 |
| 视觉/VIO 工程师 | 标定、rectification、VO/VIO、视觉退化与评测 |
| 声呐/融合工程师 | CFAR/registration、声学因子、可靠性与声光关联 |
| 重建工程师 | 稠密深度、submap、TSDF/surfel/occupancy 与地图指标 |
| 平台/测试工程师 | ROS2、录制回放、CI、基准、可观测性和发布工具 |

数据/仿真工程可作为第六角色，或由视觉/平台角色在 P0–P1 共同承担。

粗略投入：

- 达到内部稳定研发/测试平台：约 6–12 工程人月；
- 达到可长期在线运行的集成原型：约 10–18 工程人月；
- 达到有现场证据的生产系统：约 12–24+ 工程人月；
- 4 人核心团队按部分并行执行，内部研发平台约需 2–4 个月，在线集成原型约需
  5–8 个月，有现场证据的生产系统约需 9–15+ 个月。工程人月不包含等待船期、场地和
  传感器返修的时间，现场环境和数据获取仍是最大日历不确定因素。

## 9. 最近两周建议动作

1. 收口深度符号测试，恢复完整 green baseline；
2. 给 replay 和场景矩阵增加第一批非放空 gate；
3. 完整填充 RunManifest 并固定 MCAP 依赖；
4. 用现有 50 帧真实 bag 建立 rectification/有效视差诊断报告；
5. 冻结一份全传感器 HoloOcean bag 录制规格并完成首轮采集；
6. 为三条固定轨迹建立统一结果表，至少包含输入计数、因子计数、solver 状态、ATE/RPE、
   地图点数、有效覆盖和 P95 latency；
7. 将上述检查接入 `tools/verify_pipeline.sh`，让已有的 `.github/workflows/ci.yml`
   直接复用（CI 骨架已存在，缺的是这些非放空 gate，不是流水线本身）。

这两周不建议同时启动多个地图后端、3DGS、学习模型推理或复杂 dashboard。当前最大收益
来自让真实数据、基础算法和自动验收形成闭环，而不是继续扩展展示层或未来接口。

## 10. 风险与决策门

| 风险 | 影响 | 缓解与决策门 |
|---|---|---|
| 团队规模与第8节配置不符 | 第7节所有周期估算均按3–5人核心配置计算；`git log` 显示至今全部提交出自单一作者，团队尚未组建 | 在启动 P1 前明确团队组建时间表；组建进度落后时按比例下调阶段目标而非压缩验收门槛 |
| 全传感器真实数据不足 | 无法判断声光收益和 sim-to-real 差距 | P1 完成固定数据集前，不宣称真实声光 SLAM 已闭环 |
| 标定/时间错误被算法吸收 | 产生看似收敛但系统性错误的轨迹和地图 | 所有算法调参前先通过 time/TF/calibration audit |
| 过早自研完整求解器 | 消耗大量时间且难以达到成熟库稳定性 | 用限时 benchmark 在 Ceres/GTSAM 中选择，不继续扩展 v1 稠密 LM |
| 困难场景 gate 放空 | 测试全绿但系统没有有效输出 | gate 同时检查覆盖率、拒绝原因和质量收益，0 accepted 不能视为成功 |
| 地图只追求点数 | 数百万错误点掩盖零声学贡献 | 强制报告来源 mask、accuracy/completeness 和 outlier ratio |
| 配置存在但不驱动实现 | 实验记录与实际算法不一致 | 未识别或未实现的算法选择必须启动失败，不能静默回退 |
| 依赖和数据未版本化 | 无法复现实验和定位回归 | P0 完成依赖 pin、数据 ID 和完整 manifest 后再积累正式 benchmark |
| GPLv3 移植边界 | 若最终交付形态要求非 GPL（对外分发闭源/商业化），现有 SVIn 移植（`sonar_range_residual`）会成为事后难以拆除的依赖 | **在 P1 结束前**（而非"发布前"）拍板最终交付是否要求非 GPL；若是，立即规划替换/重新推导该因子的路径——越晚决定，替换成本越高。每次移植继续更新 NOTICE |

## 11. 计划维护方式

- 每完成一个阶段，更新本文件的实测基线、成熟度和下一阶段门限；
- 具体实施任务单独写入 `docs/superpowers/plans/`，本文件只保留团队级路线；
- 数字必须来自可重复命令或版本化数据，不以一次人工观察替代回归证据；
- 新增能力时同时说明适用数据域、失败模式和验收门，不以“代码已存在”代替“能力已交付”；
- 若目标、硬件或传感器组合改变，先更新完成定义和 P1 数据规格，再调整算法路线。
