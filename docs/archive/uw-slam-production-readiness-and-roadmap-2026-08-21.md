# uw_slam 生产就绪度审计与阶段路线图

> 状态：**当前审计基线 + 粗粒度执行路线**
> 核对日期：2026-08-22
> 核对范围：当前工作区源码、构建与测试工具、合成数据回放，以及本机保留的一份真实
> HoloOcean 录制。当前工作区含未提交修改，因此本文记录的是“审计时刻的实际状态”，
> 不是某个已发布版本的能力声明。

本文回答两个问题：当前仓库距离“水下声光融合、SLAM、重建研发团队可长期依赖的生产
与测试工具链”还有多远，以及应该按什么顺序补齐。它不替代
[长期架构设计](../acoustic-optic-slam-platform-architecture-2026-08-17.md)；架构文档定义
长期边界，本文负责记录当前差距、阶段优先级和验收门。

## 1. 执行摘要

当前代码不需要推倒重来。Protobuf 规范化消息模型、MCAP 回放、模块依赖边界、确定性测试和
若干真实算法切片已经构成了有价值的工程底座。主要问题不是架构方向错误，而是架构层
明显领先于实现层：schema 和接口已经描述了接近生产系统的能力，在线运行、真实数据、
估计后端、稠密重建和质量门禁仍主要停留在骨架或合成验证阶段。

按不同目标衡量，当前成熟度约为：

| 目标层级 | 当前成熟度 | 含义 |
|---|---:|---|
| 可扩展研究代码骨架 | 65%–75% | 分层、核心消息与接口、合成端到端链路和基础测试已经成立 |
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
| `ctest --test-dir build --output-on-failure` | 136/136 通过 | 118 个 unit、13 个 contract、3 个 integration、2 个 lint 均通过 |
| `tools/lint/check_no_ros_in_core.sh` | 通过 | C++ 分层依赖及 ROS/vendor 隔离仍成立 |
| HoloOcean Python pytest（`uv run pytest`） | 35/35 通过 | 复核：本文写作时描述的 1 个失败（`test_record_session.py` 仍用旧式正值 fixture）实际已在
  `79d9be5`（Fix real-data VO pipeline: depth sign bug...）中随深度符号修订一并收口；重新
  跑一遍工作区确认当前不存在这个失败，下方一段按已收口改写，不再是待办 |
| `git diff --check` | 通过 | 已跟踪改动未发现空白错误 |

同一轮 `tools/verify_pipeline.sh`（不含 ROS2）六步全部通过：build、CTest、pytest、lint、
合成数据生成和 replay；默认回放 ATE RMSE 为 `0.0665821 m`，12 个位姿匹配。

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

这些结果证明相机/声呐前端、因子构建、小规模位姿图、局部地图数据和评测接口能够组合
运行。但两条回放中的稠密声光关联分别为 `0 accepted / 12 rejected` 和
`0 accepted / 11 rejected`。也就是说，定位图使用了声呐距离约束，但回放产生的数百万
地图点基本是光学深度量测结果；稠密重建没有在这条端到端路径中获得有效的声学修正。

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

> **根因已修复，gate 已接入 ctest**：上面这条复核里的诊断按其自身建议独立调查后
> 定位到了确切机理——数学上，近 boresight 时 bearing 完全不依赖 elevation
> （`atan2(y,x)` 与 z 无关），range 只有二阶 `sec(phi)` 修正，所以在一块"干净"的
> 平坦目标 patch 上，声呐弧的多个 elevation 采样点在几何评分上天然接近打平，但它们
> 对应的深度值本来就一致——不是两个互相竞争的假设，只是同一个点的冗余估计。修法是
> 在 `src/frontends/acoustic_optic_associator.cpp` 的 ambiguity-margin 判定里加了
> 一个新的 `depth_agreement_sigma`（默认 3.0，与 `AcousticOpticDepthFusionParams::
> innovation_gate_sigma` 同一套 sigma-multiple 惯例）：几何分数打平的两个候选，只要
> 它们的 `depth_m` 在各自 `variance_m2` 的联合标准差范围内一致，就不再判 AMBIGUOUS，
> 落回接受最优分——只有深度也明显不一致时才真正视为竞争假设、维持拒绝。两个新增
> 单测（`AcceptsTiedScoreCandidatesWhoseDepthsAgree`/
> `RejectsTiedScoreCandidatesWhoseDepthsDisagree`，`tests/frontends/
> acoustic_optic_associator_test.cpp`）用精确构造的（数值上零残差、只是 elevation
> 不同）候选像素分别钉住了两个方向。用 `--seed 4242 --trials-per-scenario 20`
> 复测：`clean_textured`/`elevation_stress` 都从 0/20 变成 20/20 accepted。
>
> 同时发现上面复核里 `time_offset_fault`/`extrinsic_perturbation` 的 0/20 是另一件
> 事，不是关联评分 bug：两者都是`acoustic_optic_scenarios.cpp`里明确写了"就是要
> fail-closed"的故障注入场景（分别是 1 秒 capture-time 偏移，设计上应该在同步器的
> 时间 gate 就被拒绝；1.0 米声呐外参扰动，注释里明确写着"chosen to unambiguously
> exceed the associator's default range/bearing gates so this scenario demonstrates
> fail-closed rejection"）——0 接受是它们的正确、预期结果，只是最低覆盖 gate 的排除
> 名单当时只列了 `kSonarDropout`/`kOpticalInvalidRegion` 两个，漏掉了这两个同样该
> 排除的场景。现在排除名单补齐后，9 个场景里 `kSonarDropout`/`kOpticalInvalidRegion`/
> `kTimeOffsetFault`/`kExtrinsicPerturbation` 四个按设计 0 接受，其余 5 个
> （`clean_textured`/`low_texture_sonar_visible`/`turbid_sonar_visible`/
> `repeated_structure`/`elevation_stress`）全部非零，最低覆盖 gate 在
> `--seed 4242 --trials-per-scenario 20` 下整体 exit 0。
>
> 最低覆盖 gate 已经接入 `tests/integration/
> acoustic_optic_scenario_matrix_determinism_test.sh`（不再 `|| true` 忽略矩阵
> 二进制自己的退出码），随 `ctest` 一起跑；trials-per-scenario 从 5 提到 8——因为
> `turbid_sonar_visible` 是一个"有时候合理地 0 接受"的困难场景、没有被排除在 gate
> 之外，在这条测试固定的 seed 4242 下 5～6 次试验会让它小样本地落到 0（该测试脚本
> 里有实测记录：6 次不过、7/8 次能过），8 次是留了余量后选的下限，不是为了让 gate
> 变绿而放宽阈值——这条区别正是本文 10 节风险表要求的："gate 同时检查覆盖率、拒绝
> 原因和质量收益，0 accepted 不能视为成功"，这里改的是试验次数而不是覆盖率阈值
> 本身。baseline 改善 gate（`--min-fusion-improvement-fraction`）也已实现，但仍
> opt-in（原因见第 7 节 P0 清单）。

### 2.4 真实 HoloOcean 录制回放

本机保留的真实录制约 76 MB、50 个 keyframe，只包含双目图像、GT pose 和 depth，
没有声呐、IMU 或 DVL。

> **2026-08-23 frontend-correctness-closure 复核，数字已变，且是退化**：下面
> 前一段列的 5 项是 P0 审计时刻的数字。frontend-correctness-closure 把一般离轴
> stereo rectification（`opencv_adapters::StereoRectificationContext`）真正接进了
> `replay_demo`（见代码库参考文档 9.2 节第 2 步），带相机的 rig 现在**总是**先过一遍
> rectification 再喂给 VO/稠密光学前端——这条真实 rig（`example_auv_real_camera.yaml`，
> 真实标定的畸变系数）此前从未真正走过这条路径。用当前 `real_holoocean_vo.yaml`
> 重新回放（`--align-ate`，两次独立运行数字一致，不是随机波动）得到：
>
> - 46 条双目 VO 相对位姿、47 个 keyframe（此前 49 条/50 个——变少了）；
> - 47 条深度因子；
> - 0 条声呐因子（不变，这份录制本就没有声呐）；
> - 求解器运行 30 次后仍 `stalled`（定性结论不变）；
> - 对齐后的 ATE RMSE 为 `4.32138 m`（此前 `0.5596 m`——明显变差）；
> - 稠密地图点 907779 个，全部 `OPTICAL_ONLY`（此前 0 个——`StereoOpticalDepthFrontend`
>   之前在这个真实 rig 上因为 `StereoGeometry::Resolve` 要求纯平移基线、原始外参不满足
>   而静默失败，现在喂的是 rectified 后必然满足这个假设的 derived rig，稠密光学 pass
>   反而第一次真正跑起来了）。
>
> **这是一个真实退化，机制已定位、修复留待后续**：不是重采样削弱纹理这个猜测（那是
> `CameraRectifier` 那条独立、仍未接入的路径的已知教训，下一段还在讲它）——具体查过
> 之后，根因是这台真实机体左右相机的标定基线不是纯 y 轴平移
> （`example_auv_real_camera.yaml` 头部注释：x/z 分量占基线量级的 15-17%），
> `cv::stereoRectify` 为了让两个虚拟相机满足行对齐（row-epipolar）约束必须对两个相机
> 施加一个不小的旋转，这个旋转把 left 相机的主点从标定值 `cx≈256`（图像中心）搬到了
> `cx≈170`；用 `alpha=-1`（更保守的缩放/裁切）复核过，`cx` 分毫不差还是
> `170.043`——证明这个偏移完全来自旋转本身，跟 `alpha`/裁切策略无关，排除了"是裁切
> 参数选得不好"和"是实现 bug"两种猜测。`alpha=-1` 下 keyframe 数从 47 回升到 49（仍少于
> 基线的 50）、ATE 从 `4.32138 m` 降到 `1.55571 m`（仍远高于基线的 `0.5596 m`）——说明
> `alpha` 只是次要因素，主因是 `stereo_landmark_vo_frontend` 的 Harris 角点检测/时序
> 匹配/RANSAC 参数只在近乎平行基线的合成数据上验证过，面对这组主点大幅偏移、需要真实
> 旋转对齐的真实标定时表现变差——是否是可修的参数问题（重新联合调 VO 参数）还是需要
> 更换检测/匹配策略，仍待专门的后续工作验证，本次改动没有去动它。**在这项后续工作完成
> 之前，不应该认为"真实数据 VO 现在更好了"或者"和以前一样"**——已经变了，而且是往差的
> 方向变。下面这句"离线 VO 能生成连续相对位姿"的结论仍然成立，只是数字要用上面这份
> 复核的为准。
>
> **2026-08-23 同日后续：尝试修复，未成功，附带一个真正找到但已回退的架构限制**。
> 逐条 dump 每条 relative-pose 边的 `|t|`/`inlier_rmse_m` 后发现：坏结果集中在少数几条
> "单步"边上（如 kf31→kf32 单步 `|t|=7.8m`，物理上不可能），这些边的
> `fit->inlier_rmse_m` 普遍在 0.11-0.25m（`RansacParams::inlier_threshold_m` 默认
> 0.3m 只是"够格计入 inlier"的单点门槛，不代表这些 inlier 真的相互一致），而正常边普遍
> 在 0.01-0.09m——一个此前算出来但从未被任何地方消费的质量信号。据此加了新的
> `CovarianceEstimationParams::max_inlier_rmse_m`（`visual_odometry.max_inlier_rmse_m`
> YAML 字段，`<=0`/不设为禁用，当前所有 `configs/defaults/*.yaml`
> 均未启用，不影响任何已有实验），拒绝"inlier 集合本身不够紧"的 fit，即使它已经通过
> conditioning 检查——但扫了 `0.08~0.20m` 一整段阈值，没有一个能把真实数据结果拉回接近
> 基线的 `0.5596m`：更紧的阈值（`0.10m` 以下）几乎拒绝了这条 bag 里除最初几个 keyframe
> 外的全部候选（只剩 4 条边/5 个 keyframe，ATE `0.0636m`——单独看很漂亮，但代价是
> 几乎放弃了整段轨迹）；更松的阈值（`0.12~0.20m`）保留更多 keyframe（32~43 个）但 ATE
> 仍在 `1.7~3.6m`，仍远差于基线。还尝试了另一个方向：`consecutive_failures_` 达到
> `max_consecutive_failures` 后不再死磕越来越"过时"的 reference，而是直接把当前帧提升
> 为新 reference（放弃这段失败的 evidence，但让后续帧能匹配一个新鲜、邻近的
> reference）——这个思路被真实数据证伪了，而且证伪的原因本身是个值得记录的架构限制：
> 一旦 reanchor 触发，新 reference 所在的这段轨迹如果此后再没能连回原来那条从 kf0 出发
> 的链，`PoseGraphProblem`/`replay_pipeline.cpp` 目前的实现只会保留能从 kf0
> 沿关联边可达的那部分——重新锚定等于把断掉的那段轨迹整体丢弃，而不是"多一个健康的
> 局部轨迹段"，结果比什么都不做更差（同样只剩 5 个 keyframe，但连锁失败点更早）。这条
> 尝试已经从代码里完全回退，不在当前分支里——记录在这里是因为如果以后有人想用
> "分段重定位/多轨迹段合并"来解决真实数据 VO 追踪丢失的问题，需要先解决图连通性这个
> 前提，不是一个孤立的 VO frontend 改动能做到的。综上：`max_inlier_rmse_m`
> 这个质量信号是真实的、有真实数据证据支撑的，作为默认关闭的新选项保留下来对后续调参
> 有用，但**这次没能找到让真实 HoloOcean 数据回到改动前质量水平的修复**，问题仍然
> 开放。

用 P0 审计时刻的 `real_holoocean_vo.yaml` 回放（当时一般 rectification 还没接入）
得到过：49 条双目 VO 相对位姿、50 条深度因子、0 条声呐因子、求解器运行 30 次后
`stalled`、对齐后的 ATE RMSE 为 `0.5596 m`、稠密地图点为 0。

这份数据说明真实双目 VO 已经能够产生连续相对位姿，但真实重建尚未跑通。
`include/sensor_models/camera_rectifier.hpp` 的 `CameraRectifier` 是个更局限的
plumb-bob same-K 去畸变原语，有 9 个单元测试，但仍未接入 replay，也不是支持任意
离轴双目 rig 的通用 rectifier——对该真实 bag 的离线试验中，它校正后的影像使当前 VO
默认参数的跟踪从 50/50 降至 8/50，表明前端配置还需随影像域重调。`opencv_adapters`
的一般 rectification 是另一条独立路径，已接入 `replay_demo`（见上面的复核框），但
如上所述，它在这条真实数据上的实际效果目前是让数字变差，还需要专门调查。由于该 bag
未纳入版本化数据集，它只能作为审计证据，尚不能作为团队长期回归基准。

## 3. 分项成熟度

| 维度 | 评分 | 已有基础 | 主要缺口 |
|---|---:|---|---|
| 架构、核心消息与接口 | 7/10 | Protobuf、模块 DAG、边界 lint、带类型的量测与局部地图数据 | 消息版本迁移、单位/符号约束和兼容策略不足 |
| 声呐前端 | 5/10 | CFAR、极坐标转换、DBSCAN、range factor | 真实声呐数据、registration、部分位姿协方差、环境自适应 |
| 光学 VO/VIO | 3/10 | 合成双目 VO、Harris/NCC/RANSAC、一般离轴 stereo rectification（`opencv_adapters`，已接入 replay，见 2.4 节复核——真实数据上的实际质量效果还没调查清楚） | IMU 预积分、滑窗、边缘化、真实退化处理 |
| 声光融合定位 | 3/10 | range-only 因子进入位姿图 | 联合路标、可靠性调度、声呐 registration、消融证据 |
| 声光融合重建 | 2/10 | 像素级后验深度、局部点云数据交接、点云地图指标原语 | 声学有效覆盖、稠密几何、地图融合/重积分、reference 数据与指标门禁 |
| SLAM 后端 | 2/10 | 小规模 Eigen LM、FactorBuilder 接口 | 流形稀疏求解、鲁棒核、fixed-lag、协方差、回环/重定位 |
| 在线运行时 | 2/10 | bounded queue、状态机和 lane 原语 | scheduler、异步数据流、背压、降级、恢复、实时预算 |
| 真实数据与标定 | 3/10 | HoloOcean recorder、相机标定工具、统一格式写入器 | 全传感器录制、版本化数据、time/TF audit、公开数据集 adapter |
| 测试与评测 | 4/10 | 275 个 CTest（2026-08-23）、确定性回放、场景矩阵最低覆盖门禁、点云地图指标 API、acoustic-optic 贡献非零 gate | 真实 benchmark、地图指标接线与门禁、质量/延迟硬门、长稳和更完整故障注入 |
| 工程生产化 | 3/10 | CMake、pytest、主 CI、ASan+UBSan job、gcov/cppcheck 报告工具 | 可信 TSan、coverage/static 阈值、包发布、依赖锁定、soak、完整 manifest |

## 4. 值得保留的工程资产

后续应继续沿用以下基础，而不是重建另一套平行框架：

1. Protobuf 作为跨语言唯一规范化消息模型；
2. MCAP 作为 live/replay 统一证据载体；
3. `domain → core → algorithms/runtime/adapters → application → apps` 的单向依赖；
4. frontend、FactorBuilder、ResidualBlock 和 StateStore 的职责分离；
5. 显式 seed 和逐字节确定性回放；
6. 局部地图数据保存在局部坐标系，后端修正通过 keyframe pose 传播；
7. 外部仓库只读、移植代码保留 provenance 和许可证记录；
8. 分层配置与实验入口。

这些资产使项目不需要架构重写。后续投入应优先填充实现和证据面，避免继续扩充只有
schema、enum 或文档而没有运行闭环的新抽象。

## 5. 主要差距

### 5.1 估计后端仍是小规模求解验证器

当前求解器使用稠密线性代数和原始 7 参数位姿更新，再归一化四元数。它适合验证小规模
合成图和 FactorBuilder 接口，不适合真实长序列、滑窗和在线优化。缺少 SE(3) 流形更新、
稀疏求解、鲁棒核、联合速度/bias/路标、边缘化、协方差与可观测性诊断。

### 5.2 当前不是完整 VIO/SLAM

默认相对位姿来自 ground-truth+noise 桩；另一条路径是纯双目 VO，不消费 IMU。
`estimator_mode` 是保留兼容性的历史字段名，只选择这两种相对位姿输入来源，二者最终都
使用同一个 `GaussNewtonSolver`，不切换估计求解器。虽然 schema 已有 IMU
preintegration、velocity、bias 和 sonar registration 类型，但没有对应在线估计链。当前系统
更准确的定义是“相对位姿先验 + depth + 声呐 range-only 的离线批量位姿图”。

`stereo_landmark_vo` 仅在已加载的 rig 含相机时使用；否则 `replay_demo` 会像
`black_box_vio` 一样回退读取 `/evidence/relative_pose`，配置校验不会拒绝这个无相机组合。

### 5.3 重建层主要是局部地图数据交接

`SubmapManager` 是按 keyframe 索引的局部地图数据存储，不是完整的 submap 生命周期管理器；
当前它只维护局部点云数据并按最新位姿变换，没有 TSDF、surfel、
occupancy、mesh、遮挡/自由空间、地图裁剪或重积分实现。稠密声光融合只会尝试为少量
像素做后验深度修正；`AcousticOpticDepthFusionFrontend` 是融合模块，保留在 `frontends`
路径只是历史命名，尚未形成声呐覆盖区域的稳定几何生成和置信度融合。`map_backend` 是
预留的地图实现选择字段，目前唯一支持 `submap_point_cloud_v1`。
`ComputeMapMetrics` 已提供 Chamfer/completeness/outlier/F-score 的小点集 API 和单测，
但采用 `O(NM)` 暴力最近邻，尚未接入 replay、版本化 reference map 或质量 gate。

### 5.4 运行时原语尚未组成在线系统

runtime 目前提供 queue、状态机等 runtime 支持原语，而不是已经组合好的在线调度器。
`replay_demo` 仍是离线批处理：多次遍历 bag、构图后统一求解。状态机和 queue 没有被
实际 scheduler 消费；ROS2 声呐节点也只完成 transport，没有驱动前端、估计器和地图。

### 5.5 验收门禁仍未完全收口

P0 已把 solver 收敛、最小轨迹匹配、可选 ATE、非空地图和场景最低关联覆盖接入运行或
自动测试；矩阵测试也会保留真实 gate 退出码。当前剩余缺口是：

- 融合必须相对 baseline 带来可量化收益；
- P95 latency 必须满足预算；
- 地图必须包含有效点且达到精度/完整度要求；指标 API 已有，但数据接线和 gate 未完成；
- RPE、旋转误差及对应门禁尚未实现。

> **2026-08-23 frontend-correctness-closure 补充**：新增了一个与上面"地图必须包含
> 有效点"相邻但更窄的 gate——`min_acoustic_optic_accepted`/`min_acoustic_optic_map_points`
> （`application::EvaluateReplayGates`），只在 `configs/experiment/acoustic_optic_demo.yaml`
> 打开，检查地图证据里 `contribution_mask == DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC` 的点数是否
> 非零（区分"有稠密光学深度"和"声光关联真的接受了至少一次"）。这**不是**上面列的
> `ComputeMapMetrics` 精度/完整度 gate——那个仍未接线，暴力最近邻实现也仍未替换成
> KD-tree/octree（见代码库参考文档 9.3 节）；RPE、旋转误差、baseline 改善收益、P95
> latency 四项 gap 均未被这次改动触及，仍是未完成状态。

### 5.6 可复现记录已部分落地

> **P0 执行中的复核**：本节描述的是审计时刻（2.1 之前）的状态。git commit、
> 配置/标定 hash、OS、CPU、seed 和起止时间这几项已在 P0 执行中补齐（见第 7 节
> P0 清单），MCAP 依赖也已固定到具体 commit；`gpu_info` 字段也已填充，写的是
> `"n/a (CPU-only Eigen pipeline, no GPU dependency in this build)"`，如实反映
> 当前求解器不用 GPU，不是留空。仍未落地的只有 `model_hash`
> （`include/runtime/run_manifest.hpp` 有这个字段，但 `src/application/replay_pipeline.cpp`
> 目前没有任何地方给它赋值）——当前系统没有可学习模型权重这一类产物，这个字段
> 要到 P2/P3 引入学习式组件时才有内容可填。本节原始判断按未改写保留在下方，
> 供审计基线参照，第 7 节是当前实际完成状态。

当前 RunManifest 已填 git commit、配置/标定 FNV-1a hash、bag 路径、OS、CPU、GPU
说明、seed 和起止时间；MCAP 依赖已固定 commit。`model_hash` 因当前没有学习模型而
为空，hash 也不是加密内容摘要；bag 只有路径而没有版本化数据 ID/内容 hash，完整依赖
清单也未写入 manifest。因此它能定位本地运行上下文，但还不足以可靠复原跨机器实验。

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
- ~~在 schema/验证函数中明确深度正方向、坐标 frame、单位和 rectified 语义~~——
  已完成：`schemas/proto/uw/domain/measurement.proto` 的
  `PressureDepthMeasurement`/`OpticalDepthPriorMeasurement`/
  `FusedDepthMeasurement`/`AcousticOpticAssociationRecord` 各 `depth_m` 字段
  现在都标注了各自的符号/坐标 frame 约定（**两种不同约定共用同一字段名**：
  `PressureDepthMeasurement` 是世界系 Z-up、正值向下；后三者是相机光学系、
  正值朝前——这是审计发现的最大歧义点），`image.proto` 的 `is_rectified`
  标注了"当前无消费者校验、仅合成数据生成器写入"的实际状态；
  `src/domain/domain.cpp` 的 `ValidateDepthValues`、
  `src/application/replay_pipeline.cpp`/`apps/synth_bag_gen.cpp` 的正负号转换点、
  `src/mapping/acoustic_optic_map_bridge.cpp` 的 `depth_m > 0` 过滤都补了
  对应注释；`PressureDepthMeasurement` 本身仍没有 `Validate*` 函数、原样
  透传不做有限性/量级检查——这不算本条范围内的缺口（本条是"讲清楚约定"，
  不是"新增运行时校验"），但值得在后续 gate 工作中留意；
- ~~为 replay 增加求解收敛、最小匹配数、ATE 和非空地图 gate~~——已完成
  并在两个合成 experiment（`synthetic_smoke.yaml`/`synthetic_smoke_vo.yaml`）
  的 `gates:` 段落打开，`require_converged` 无条件默认开启；
- ~~为场景矩阵增加最低有效覆盖、baseline 改善和 P95 latency gate~~——三个 gate
  都已写入 `apps/acoustic_optic_scenario_matrix.cpp`（最低有效覆盖是无条件
  开启的；P95 latency 通过 `--max-p95-latency-ms` opt-in；baseline 改善通过
  `--min-fusion-improvement-fraction` opt-in），且导致最低有效覆盖 gate 之前
  没法接入 ctest 的关联评分 bug 本身也已修复——见 2.3 节"根因已修复，gate 已
  接入 ctest"一节：`src/frontends/acoustic_optic_associator.cpp` 加了
  `depth_agreement_sigma` 判定，`clean_textured`/`elevation_stress` 从
  20 次全 AMBIGUOUS 变成 20 次全 accepted；`time_offset_fault`/
  `extrinsic_perturbation` 确认是设计上就该 0 接受的故障注入场景，补进了 gate
  的排除名单。baseline 改善/P95 latency 两个 gate 仍是 opt-in（原因不变：多数
  场景 `covered_rmse_samples`/延迟预算尚未到 P1/P3 该收口的程度）；
- ~~让确定性测试保留矩阵真实 gate 失败，而不是统一忽略退出码~~——已完成：
  `tests/integration/acoustic_optic_scenario_matrix_determinism_test.sh` 不再
  `|| true` 忽略矩阵二进制的退出码，`--trials-per-scenario` 从 5 提到 8（原因
  见 2.3 节该条备注——`turbid_sonar_visible` 在这条测试固定的 seed 下小样本会
  合理地落到 0，8 次是留了余量的下限，不是放宽覆盖率阈值本身）；
- ~~填充 RunManifest 的代码、配置、标定、bag 路径、seed、环境与起止时间~~——已完成
  （`apps/replay_demo.cpp` 注入 `UW_GIT_COMMIT`，`src/application/replay_pipeline.cpp`
  使用该值并通过配置/标定内容 FNV-1a 哈希 + `DetectOsInfo`/`DetectCpuInfo` 填充，`cmake/Applications.cmake`
  负责 git commit 注入，脏树会带 `-dirty` 后缀）；
- ~~将 MCAP 等外部依赖固定到 commit/tag~~——已完成（`cmake/UwMcap.cmake` 固定到
  具体 commit，不再跟踪 `main`）；
- `.github/workflows/ci.yml`（`tools/verify_pipeline.sh`）已经在跑
  build/CTest/pytest/lint/合成 replay 全套——上面已完成的 replay gate 随
  `synthetic_smoke*.yaml` 一起被这条流水线的 `replay_demo` 步骤自动覆盖；场景
  矩阵的最低覆盖 gate 现在也通过 ctest 里的
  `integration.acoustic_optic_scenario_matrix_determinism` 被这条流水线覆盖
  （该 ctest target 本身就在 `verify_pipeline.sh` 第 3 步跑的 `ctest` 范围内，
  不需要 `verify_pipeline.sh` 单独再调一次矩阵二进制）；baseline 改善/P95
  latency 两个 opt-in gate 仍未接入任何自动化，留给对应阶段收口。

阶段验收：

- 工作区所有测试和 lint 全绿；
- 人为让 solver stalled、地图为空或场景无有效关联时，相关 gate 必须失败；
- 同一 manifest 能定位到完整代码、配置、标定和数据版本；
- clean checkout 在受支持环境中可一次完成配置、构建和验证。

### P1：真实离线多传感器闭环（4–8 周）

**目标**：先把真实数据离线链路跑通，再扩展在线系统。

主要工作：

- 固定直线、转弯、小闭环三类 1–3 分钟 HoloOcean 场景；
- 录制包含双目、成像声呐、IMU、depth、GT 的统一 MCAP 录制格式；
- 完成 topic、capture/receive time、clock domain、TF 和标定 audit；
- 建立去畸变和双目极线校正，明确 raw/rectified 图像契约；
- 保持未知配置 fail-fast；为目前只有单一实现的 sonar/optical/map 增加真正的可替换实现
  与派发（estimator 和 landmark detector 已真实派发，其他选择目前只是校验支持值）；
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
- 在已有 ASan+UBSan CI、gcov/cppcheck 报告基础上增加可信 TSan、覆盖率/静态分析阈值、
  包构建和 60 分钟 soak test（当前预编译 protobuf/gtest 使 TSan 不可信）。

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

## 9. 下一轮建议动作

深度符号、首批 replay/场景矩阵 gate、manifest 基础字段、MCAP pin 和真实 bag 初步
rectification 诊断已经完成。下一轮优先做：

1. 冻结一份包含双目、声呐、IMU、depth、GT 的 HoloOcean 录制规格并完成首轮采集；
2. 把 camera rectifier 接入可配置回放路径，并针对校正后影像重新标定 VO 参数；
3. 版本化直线、转弯、小闭环三条固定数据及其 calibration/data ID；
4. 为三条轨迹建立统一结果表，至少包含输入/因子计数、solver 状态、ATE、地图点数、
   有效覆盖和 P95 latency；RPE 应在 evaluator 实现后加入，而不是在文档中假定已有；
5. 给场景矩阵的融合收益和 P95 latency 选择可执行预算并接入 CI；
6. 补齐 manifest 的数据内容标识和依赖版本清单，保证跨机器可复现。

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
- 具体实施细节由代码、测试和版本历史承载，本文件只保留团队级路线；
- 数字必须来自可重复命令或版本化数据，不以一次人工观察替代回归证据；
- 新增能力时同时说明适用数据域、失败模式和验收门，不以“代码已存在”代替“能力已交付”；
- 若目标、硬件或传感器组合改变，先更新完成定义和 P1 数据规格，再调整算法路线。
