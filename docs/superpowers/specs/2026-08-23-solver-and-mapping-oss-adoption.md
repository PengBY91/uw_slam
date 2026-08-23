# 求解器与建图空间索引：成熟开源库采纳设计

> 日期：2026-08-23
> 状态：设计复核通过，§11 三项开放问题已拍板（见该节结论），待排期实施
> 范围：求解器后端（Ceres 适配器 + 基准决策门）、SurfelMap 空间索引加速（nanoflann 适配器）
> 关联：`docs/superpowers/specs/2026-08-23-frontend-correctness-closure-design.md`（OpenCV 双目
> 适配层，同一“核心保持无第三方类型、适配层隔离、应用层编排”方案在这里被复用到另外两个模块）；
> `docs/acoustic-optic-slam-platform-architecture-2026-08-17.md` 第 20 节（本文档处理的两项都是
> 该节明确列出的“延后到实施计划用实测选择”项：“第一版图优化库”“TSDF/occupancy 的具体实现库”，
> 后者本文档只解决其中一个更窄的子问题，见 §1.3）

---

## 第一部分：需求规格

### 1. 背景

#### 1.1 求解器

`include/estimation/gauss_newton_solver.hpp` 是手写 Levenberg-Marquardt，头文件注释已经明确标注
这是“a deliberate, documented v1 simplification”，用于验证 `FactorBuilder`/`StateStore`/
`PoseGraphProblem` 契约，不是最终生产求解器；`ResidualBlock`（`include/measurement_api/
residual_block.hpp`）的接口设计本身就是为了让求解器可替换：其 `Evaluate(parameters, residuals,
jacobians)` 签名与 `ceres::CostFunction::Evaluate` 几乎逐参数对应（`double const* const*` 输入、
行主序 Jacobian 输出），且已经是解析 Jacobian，不是数值/自动微分——这不是巧合，是当前架构就按照
“以后接 Ceres/GTSAM 时不用碰 factor_builders”这个目标设计的。

但当前只有一个求解器消费者：`PoseGraphProblem` 把 keyframe 参数块和 residual binding 通过
`friend class GaussNewtonSolver;` 私有暴露给它。引入第二个求解器前，这个耦合必须先换成公开、
后端无关的导出接口——这是两个求解器共同的前置依赖，不引入任何第三方库，纯粹是 `estimation`
内部重构。

#### 1.2 为什么现在评估，而不是继续等

架构文档第 20 节把这个决策放进“应通过 benchmark 和决策门选择”的清单，但截至目前仓库里没有
任何基准脚手架去实际做这个决策——一直没有实测数据，这个决策就会无限期停留在“延后”状态。
本文档不是要现在就把默认求解器换成 Ceres，而是先把能做出决策所需的适配器和基准工具建起来，
用真实数字而不是主观判断来关闭这个决策。

#### 1.2.1 真实数据现状（补充说明，避免本文档只用合成数据）

仓库里已经有一段真实 HoloOcean 录制（`/tmp/real_session_depth_fixed.mcap`，50 个关键帧）
和跑通的真实端到端结果：`configs/experiment/real_holoocean_vo.yaml` 驱动
`stereo_landmark_vo`+`harris_corner`，真实 ATE rmse=0.667m（`--align-ate`，详见
[[project_uw_slam_relative_pose_vio]] memory）——MONO8 转换、真实角点检测、双目标定都已经
解决，不是被卡住的工作。所以“求解器基准只用合成数据”不是因为真实数据不可用，而是这段真实
录制只有 50 个关键帧，量级不够回答“大图上 Ceres 是否胜出”这个规模问题；真实数据在这里应该
作为**额外的正确性交叉验证**补进去，而不是被合成数据取代——见 §6.1.1、§9、验收标准 3a。

#### 1.3 SurfelMap 空间索引

`include/mapping/surfel_map.hpp:82` 自己写明：“a spatial index (voxel hash, KD-tree) is
required before this scales”；`FindNearest`（供 `AddPoint`/`AddPointWithNormal`/
`FuseWorldPoint` 使用）和 `CarveFreeSpace` 的走廊查询都是 O(现有 surfel 数) 暴力线性扫描，
注释直接指出这“NOT fine for apps/replay_demo's real map evidence volume (millions of points
per run)”。

需要澄清范围边界：架构文档第 20 节延后决策里还有一项是“TSDF/occupancy 的具体实现库与体素
分辨率”——这是一个**更大**的决策（是否要引入一整套新的占据栅格/TSDF 地图后端，比如
OctoMap），本文档**不处理**这一项，理由见 §5.2。本文档只处理 SurfelMap 现有暴力搜索这个已经
自证的、独立于地图表示选型决策的规模天花板。

### 2. 目标与非目标

#### 2.1 目标

**求解器工作单元：**

1. 把 `PoseGraphProblem` 的求解器耦合从单一 friend 类换成公开、后端无关的导出接口。
2. 新增 Ceres 求解器适配器，产出与 `GaussNewtonSolver::Summary` 同形状的结果，可在
   `replay_pipeline`/新增基准工具里按配置切换。
3. 新增一个基准工具，在现有 `synthetic_smoke` 规模和一个更大的合成压力场景（`configs/experiment/synthetic_stress.yaml`，1000
   个 keyframe，见 §11.2 的具体理由）上比较两个后端的墙钟时间、迭代次数、最终 cost、ATE，
   并验证 Ceres 路径在锁定线程数/求解器类型后仍满足仓库现有确定性回放测试的要求。
3a. 同一个基准工具额外跑一遍已有的真实 HoloOcean 录制（`real_holoocean_vo.yaml` +
   `/tmp/real_session_depth_fixed.mcap`，50 个关键帧，见 §1.2.1），验证两个后端在真实图上
   收敛到一致的解——这是规模基准之外的正确性交叉验证，不是规模基准的替代。
4. 不改变默认后端，除非基准结果支持这么做——这个决定留给用户基于基准数据做，本文档只负责
   把决策所需的数据造出来。

**SurfelMap 空间索引工作单元：**

5. 新增基于 nanoflann 的空间索引适配器，替换 `FindNearest`/`CarveFreeSpace` 走廊查询的暴力
   扫描路径，索引返回结果必须与暴力扫描完全一致（精确最近邻，不是近似），不改变现有合并/
   离群点门限/free-space carving 的语义或测试期望的具体判定结果。
6. `SurfelMap` 的公开 API、`Surfel`/`SurfelMapParams` 语义、`AddPoint`/`AddKeyframeObservation`/
   `ReintegrateKeyframe`/`CarveFreeSpace` 的调用约定保持不变；索引是可选注入的实现细节，不注入
   时行为与当前完全一致（现有测试不用改）。

#### 2.2 非目标

- 不在本阶段把默认求解器切换为 Ceres（等基准数据）；不引入 GTSAM（见 §5.1 的排除理由）；
  不实现滑窗、边缘化、IMU 预积分——这些是求解器选型之后、且很可能需要不同图变量模型的
  独立后续决策。
- 不引入 robust kernel（`robust_policy_hint` 保持未消费，与
  `2026-08-23-frontend-correctness-closure-design.md` §8.2 的既有决定一致）。
- 不把 SurfelMap 换成占据栅格/TSDF/OctoMap，不改变地图表示本身，不解决架构文档第 20 节
  “TSDF/occupancy 具体实现库”那一项延后决策——那是范围更大、需要独立评估地图表示的决策
  （见 §5.2）。
- 不给 `AcousticOpticMapBridge`/`SubmapManager`/声呐路标路径接入空间索引——`SurfelMap` 自己
  文档里已经写明声呐稀疏路标路径今天不喂给 SurfelMap，本文档不扩大这个范围。
- 不修改 `factor_builders` 的任何残差数学。

### 3. 验收标准

本阶段完成必须同时满足：

**求解器：**

1. `PoseGraphProblem` 对外暴露的求解器接入点不再是单一 friend 类，`GaussNewtonSolver` 和新增
   Ceres 适配器都通过同一套公开接口读写图状态。
2. Ceres 适配器构建在独立 CMake 选项（默认 OFF）之后，不装 Ceres 也能正常构建/测试仓库其余
   部分。
3. 基准工具能在 `synthetic_smoke` 规模和一个更大的合成压力场景上产出两个后端的对比数据
   （墙钟、迭代数、最终 cost、ATE），以可复现（固定 seed）的方式运行。
3a. 基准工具在已有真实 50 关键帧 HoloOcean 录制上，两个后端收敛到一致的解（容差内），且
   `converged` 状态一致——真实数据规模不够做规模基准，但必须覆盖正确性交叉验证。
4. Ceres 路径在固定线程数/求解器类型的配置下，通过现有 `tests/integration/determinism_test.sh`
   同等强度的确定性检查（同 seed 两次运行输出一致）。
5. 现有 `synthetic_smoke`/`synthetic_smoke_vo` 端到端 demo 在默认（`GaussNewtonSolver`）路径下
   ATE 和收敛迭代数不回归。

**SurfelMap 空间索引：**

6. 注入索引前后，对同一组输入调用序列，`SurfelMap::Surfels()`、`NumOutliersRejected()`、
   `CarveFreeSpace` 返回的移除计数完全一致（逐 surfel 完全一致，不是“大致相近”）。
7. 现有 `tests/mapping/surfel_map_test.cpp`（或等价现有测试文件）全部通过，不需要修改测试期望
   值——只新增覆盖“索引路径与暴力路径结果一致”的测试。
8. 一个新增的规模测试证明：点数从现有测试规模（几十/几百）扩大到万级时，索引路径相对暴力路径
   有数量级级别的加速（不要求具体阈值，但要有可观测的、方向正确的对比数据）。
9. 全量 CTest、layer lint（`tools/lint/check_no_ros_in_core.sh`）、ASan/UBSan 通过。

---

## 第二部分：实施计划

### 4. 前置重构：`PoseGraphProblem` 求解器接入点去 friend 化

在引入 Ceres 之前，先把 `include/estimation/pose_graph_problem.hpp` 里的
`friend class GaussNewtonSolver;` 换成公开的、纯 `estimation` 层内部的导出接口，例如：

```cpp
// pose_graph_problem.hpp
struct MutableKeyframeParameters {
  std::string keyframe_id;
  std::array<double, 7>* params;  // tx,ty,tz,qx,qy,qz,qw — layout unchanged
  bool fixed;
};
struct ResidualBinding {
  uw::measurement_api::ResidualBlock* block;
  const std::vector<std::string>* involved_keyframes;
};

std::vector<MutableKeyframeParameters> MutableParameterBlocks();
std::vector<ResidualBinding> ResidualBindings() const;
```

`GaussNewtonSolver::Solve` 改为通过这套接口读写，而不是直接碰 `PoseGraphProblem` 的私有成员。
这一步不引入任何第三方依赖、不改变任何求解器行为，可以独立完成、独立跑现有 `estimation` 测试
验证零回归——是后面两条并行工作（Ceres 适配器、以及任何未来第三个求解器）共同的地基，因此排在
实施拆分的第一步（见 §9）。

### 5. 方案选择

#### 5.1 求解器：Ceres，不是 GTSAM

选 Ceres 而不是 GTSAM，理由是它和现有架构的契合度，不是“哪个更出名”：

- `ResidualBlock::Evaluate` 已经是解析 Jacobian、`double const* const*` 参数块布局，跟
  `ceres::CostFunction::Evaluate` 是同构的——包一层 `ceres::CostFunction` 子类，`Evaluate`
  直接转发，不需要重新推导任何残差数学，`factor_builders` 零改动。
- GTSAM 没有跟这里的 `ResidualBlock` 对等的通用契约。要么用 GTSAM 的 `CustomFactor`
  （数值微分，等于扔掉本仓库已经手推、已经过测试验证的解析 Jacobian），要么给每个残差类型
  单独实现一个 GTSAM `NoetActor`/`ExpressionFactor` 子类——后者需要按 GTSAM 自己的位姿/流形
  表达重新表达每个残差，会直接违反 `ResidualBlock` 头注释里“替换求解器不需要碰
  factor_builders”这个承诺。
- 四元数流形：`Pose3` 用 Eigen 的 xyzw 存储（CLAUDE.md 明确的约定），Ceres 自带
  `EigenQuaternionManifold` 就是专门给 xyzw 内存布局用的，平移块用 `EuclideanManifold<3>`
  ——不需要自己写流形/局部参数化。
- 如果后续真的需要滑窗/边缘化/IMU 预积分（GTSAM 的强项），那本来就是一个比“换求解器”更大、
  需要重新设计图变量模型的决策，届时再单独评估，不应该现在为了那个还没排上日程的能力，牺牲
  当前这次“验证解析 Jacobian 路径能否直接换后端”的窄范围目标。

#### 5.2 空间索引：nanoflann（加速现有 SurfelMap），不是 OctoMap（换地图表示）

这两个是不同量级的决策，不应该合并：

- nanoflann 只是给 `SurfelMap` 现有的“暴力找最近点”这个动作换一个更快的实现，`Surfel`/
  `SurfelMapParams`/合并-离群点门限-free space carving 的语义完全不变——范围窄、风险低、
  可以现在就做，不用等谁拍板“地图到底要不要走占据栅格”。
- OctoMap 意味着引入一整套新的地图表示（概率占据 + raycasting carving 是它的核心能力，
  跟当前 Surfel 面元表示是两种不同的地图模型），会牵动 `MapEvidence`、
  `acoustic_optic_map_bridge.cpp`、`SubmapManager` 的输出契约，是架构文档第 20 节那条
  “TSDF/occupancy 具体实现库”延后决策本身，需要独立的基准和评估（选哪个体素分辨率、跟现有
  Surfel 面元语义如何共存或替代），不应该被“顺便当作空间索引库来用”这种理由捎带决定。
- nanoflann 是 header-only（MIT/BSD 2-Clause），跟 `cmake/UwMcap.cmake` 对 MCAP SDK 的处理方式
  一样可以用 `FetchContent_Populate` 手动接入，没有额外链接面。

### 6. 模块与依赖

延续 `2026-08-23-frontend-correctness-closure-design.md` 已经采用的方案：**核心层不暴露第三方
类型，适配层隔离在独立 `adapters/<vendor>/` 目录，应用层负责编排**。

#### 6.1 求解器适配层

新增 `adapters/ceres/`（结构与既有 `adapters/opencv/` 计划一致），依赖：

- Ceres Solver；
- `uw::estimation`（消费前置重构后 `PoseGraphProblem` 的公开导出接口）、
  `uw::measurement_api`；
- 不依赖 `frontends`、`mapping`、`application`。

公开接口：

```cpp
// adapters/ceres/include/adapters/ceres/ceres_pose_graph_solver.hpp
struct CeresSolverOptions {
  int max_iterations = 30;
  int num_threads = 1;  // 确定性优先，见 §8
  double function_tolerance = 1e-12;
};

class CeresPoseGraphSolver {
 public:
  // Summary 复用 uw::estimation::GaussNewtonSummary 的字段形状
  // (iterations/initial_cost/final_cost/converged)，调用方不用区分后端。
  uw::estimation::GaussNewtonSummary Solve(uw::estimation::PoseGraphProblem& problem,
                                            const CeresSolverOptions& options = {}) const;
};
```

##### 6.1.1 真实数据交叉验证（见 §1.2.1）

基准工具除了 `synthetic_smoke`/`synthetic_stress` 两个合成场景外，还应支持直接指向一个
已录制的 bag + experiment 配置对——落地时用现成的 `real_holoocean_vo.yaml` +
`/tmp/real_session_depth_fixed.mcap`（50 关键帧）跑一遍，两个后端各自收敛后比较解和
`converged` 状态。这段真实数据只用于正确性交叉验证，不计入规模基准的对比图表（50 个关键帧
的墙钟/迭代数差异在统计上没有意义），两者在报告里分开呈现。

CMake：新增 `option(UW_BUILD_CERES_SOLVER "..." OFF)`，用 `find_package(Ceres CONFIG REQUIRED)`
（不是 `FetchContent`）——理由见 §11.1：Ceres 自带正确的 CMake config 导出，跟 `Eigen3`/
`yaml-cpp` 走的是同一条路径（conda-forge 提供预编译包），不是 MCAP SDK 那种“没有自己的
CMakeLists.txt 只能手动 Populate”的特殊情况，没必要为它单独走一条更重的从源码构建路径（Ceres
从源码构建还会拉 SuiteSparse/glog 等一整套依赖链）。`adapters_ceres` target 只在该选项打开时
构建——不装 Ceres 时仓库其余部分（包括默认求解器路径）不受影响，跟 `UW_BUILD_ROS2` 的可选性
模式一致。

#### 6.2 空间索引适配层

`mapping` 不能直接依赖 `adapters`（`tools/lint/check_layer_dependencies.py` 里
`ALLOWED["mapping"]` 不含 `"adapters"`），所以沿用 OpenCV 方案同样的“核心定义纯接口、适配层
实现、应用层注入”结构，而不是让 `SurfelMap` 直接 include nanoflann：

```cpp
// include/mapping/surfel_map.hpp 新增的纯抽象接口，无第三方类型
class SurfelSpatialIndex {
 public:
  virtual ~SurfelSpatialIndex() = default;
  virtual void Rebuild(const std::vector<Eigen::Vector3d>& points) = 0;
  virtual std::optional<std::size_t> FindNearestWithinRadius(
      const Eigen::Vector3d& query, double radius_m) const = 0;
  // CarveFreeSpace 走廊查询：[segment_start, segment_end] 半径 radius_m 内的所有点索引
  virtual std::vector<std::size_t> FindWithinSegment(
      const Eigen::Vector3d& segment_start, const Eigen::Vector3d& segment_end,
      double radius_m) const = 0;
};
```

`SurfelMap` 构造函数新增可选参数 `std::unique_ptr<SurfelSpatialIndex> index = nullptr`：不传时
行为、性能特征与今天完全一致（`FindNearest`/`CarveFreeSpace` 走现有暴力路径）；传入时两处热路径
改走索引。这保持了现有测试不用碰、现有调用方不用改。

新增 `adapters/spatial_index/`：

```cpp
// adapters/spatial_index/include/adapters/spatial_index/nanoflann_surfel_index.hpp
class NanoflannSurfelIndex : public uw::mapping::SurfelSpatialIndex {
  // 内部用 nanoflann::KDTreeSingleIndexDynamicAdaptor 支持增量插入/移除
  // （surfel 合并会挪动位置、free-space carving 会移除 surfel，都不是纯追加），
  // Rebuild() 对应 SurfelMap::ReintegrateKeyframe 触发的全量重建路径。
  ...
};
```

`application` 层（`replay_pipeline.cpp` 构造 `SurfelMap` 的地方）负责按配置决定是否注入
`NanoflannSurfelIndex`——`mapping` 本身永远不知道 nanoflann 的存在。

**为什么这条工作单元的测试保持用程序生成的点云，不是回避真实数据**：§9 的等价性/规模测试
要回答的问题是“索引返回的最近邻和暴力扫描是否逐点一致、以及规模扩大后是否更快”——这是
`NanoflannSurfelIndex` 这个通用空间数据结构本身的正确性/性能问题，跟点是从真实声呐/相机
管线来的还是程序生成的无关；换成真实点云不会多验证出任何这两个问题之外的东西，反而会让一个
纯数据结构组件的测试依赖“真实光学深度证据已经接入 SurfelMap”这个当前还没做的另一件事
（见 §2.2 非目标——真实数据接入 SurfelMap 的具体管线不在本文档范围）。

#### 6.3 lint 覆盖盲区（顺带修复项）

`tools/lint/check_layer_dependencies.py` 的 `source_files()`/`owner()` 目前只扫描
`adapters/ros2/`，其余 `adapters/<vendor>/` 子目录（既有计划里的 `adapters/opencv/`，本文档新增
的 `adapters/ceres/`、`adapters/spatial_index/`）完全不在扫描范围内——也就是说这几个适配层
目录里的依赖违规今天不会被这个 lint 抓到。建议把 `owner()` 泛化成“`adapters/` 下任意一级子目录
都记为该子目录名对应的角色”，并给 `ALLOWED` 补上对应条目（例如 `adapters/opencv`、
`adapters/ceres`、`adapters/spatial_index` 各自允许 include 自己的第三方库 + 需要用到的核心层
角色）。这不是本次两个工作单元的阻塞项，但既然本文档新增了两个会撞上这个盲区的目录，应该在
提交前一并修掉，否则“改完代码顺手跑一下 lint”这条约定对新目录形同虚设。

### 7. 配置

新增字段，走现有分层 YAML + `ValidateExperimentConfigSelections()` fail-fast 校验（与
`estimator_mode`/`landmark_detector` 同一模式，未知取值启动失败）：

- `solver_backend: gauss_newton | ceres`（默认 `gauss_newton`）；
- `solver.ceres.num_threads`、`solver.ceres.max_iterations`（仅 `UW_BUILD_CERES_SOLVER=ON` 时
  生效，否则选了 `ceres` 直接启动失败并给出“未编译 Ceres 支持”的明确错误，不静默回退）；
- `mapping.surfel_spatial_index: brute_force | nanoflann`（默认 `brute_force`，即当前行为）。

### 8. 错误处理与风险

| 项 | 行为 | 备注 |
|---|---|---|
| `solver_backend: ceres` 但未编译 Ceres 支持 | 启动失败，明确报错 | 配置错误，不回退到 gauss_newton |
| Ceres 求解失败/不收敛 | 返回 `converged=false` 的 Summary，同 GaussNewtonSolver 语义 | 调用方已有处理路径 |
| Ceres 多线程导致跨运行结果不确定 | `num_threads` 默认锁 1，基准/确定性测试下强制单线程 | 见验收标准 4；这是本工作单元最大的正确性风险点，必须在合入前用 `determinism_test.sh` 同等强度验证，不能只信 Ceres 文档说的“确定性求解器类型” |
| nanoflann 索引返回结果与暴力扫描不一致 | 视为该工作单元的阻塞 bug，不允许以“足够接近”为由放行 | 见验收标准 6——SurfelMap 现有测试的具体断言值依赖精确最近邻，近似索引会静默改变这些值 |
| 索引增量更新后与批量重建结果不一致 | `Rebuild()` 路径与增量插入路径必须产出相同索引状态 | 对应 `ReintegrateKeyframe` 的全量重建语义 |

### 9. 测试设计

**求解器：**

- `pose_graph_problem` 去 friend 化后的导出接口：现有 `estimation` 测试全部不改动通过；新增
  测试直接覆盖 `MutableParameterBlocks`/`ResidualBindings` 的正确性。
- Ceres 适配器：小规模合成图（几个 keyframe + 已知残差）验证收敛到与 `GaussNewtonSolver` 一致
  （在容差内）的解；四元数块经过优化后仍是单位四元数（流形约束生效的直接证据）。
- 基准工具：`synthetic_smoke` 规模 + `synthetic_stress`（1000 keyframe，见 §11.2），两个后端各跑
  固定 seed 两次，验证 Ceres 路径确定性；报告墙钟/迭代数/最终 cost/ATE 对比。
- 真实数据交叉验证（§6.1.1）：`real_holoocean_vo.yaml` + 真实 50 关键帧 bag，两个后端收敛解
  一致性检查——这是本文档唯一一处真实数据用例，用途是正确性而非规模基准。
- 确定性回归：`tests/integration/determinism_test.sh` 同等检查套用到 `solver_backend: ceres`
  路径。

**空间索引：**

- 索引路径与暴力路径在相同随机点序列（含合并、离群点拒绝、free-space carving 触发）下逐
  surfel 结果一致的等价性测试。
- 增量插入 vs 全量 `Rebuild()` 一致性测试。
- 规模对比测试：点数从现有测试规模到万级，记录两条路径耗时，证明方向正确的加速（非阈值门禁，
  诊断性质，避免把具体硬件相关的绝对耗时数字做成会在别的机器上 flaky 的门禁）。
- `SurfelMap` 现有测试文件不修改断言值，只新增上述测试用例。

### 10. 实施拆分

两个工作单元彼此独立，可以并行推进；单元内部按 TDD red-green-refactor 顺序执行：

1. **前置重构**（求解器工作单元的地基，见 §4）：`PoseGraphProblem` 去 friend 化，`estimation`
   现有测试零回归。**必须先于 Ceres 适配器完成。**
2. **[已完成 2026-08-23] Ceres 适配器**：`adapters/ceres/`（`CeresPoseGraphSolver`，
   `ResidualBlockCostFunction` 把 `ResidualBlock::Evaluate` 原样转发给
   `ceres::CostFunction::Evaluate`——两者签名本来就同构，零残差数学改动；四元数流形用
   `ceres::ProductManifold<EuclideanManifold<3>, EigenQuaternionManifold>`，顺序和
   `Pose3::ToParameterBlock()` 的 `[tx,ty,tz,qx,qy,qz,qw]` 布局精确对应）、
   `UW_BUILD_CERES_SOLVER` CMake 选项（默认 OFF，`find_package(Ceres CONFIG REQUIRED)`）。
   **配置接入比原计划更简单**：实现时发现 `PlatformDefaultsConfig` 里本来就有一个
   `estimation.solver` 字段（值 `"gauss_newton_v1"`），已经在 YAML 里解析,但既没有被
   `ValidateExperimentConfigSelections` 校验、也没有被 `replay_pipeline.cpp` 实际
   dispatch——是 CLAUDE.md/configs/README.md 反复强调的"配置存在但不驱动实现"那类缺口
   本身。没有新增 `solver_backend` 字段,而是补全这个已有字段的校验+dispatch,并顺带
   加了一个实验文件级别的 `estimation.solver` override（与 `landmark_detector` 同一
   模式,不用整个复制一份 `defaults/*.yaml` 才能切后端）。
   **环境细节记一笔**：Ceres 的 CMake config 透传依赖 SuiteSparse_config,后者会
   `find_dependency(OpenMP COMPONENTS C)`——本仓库 `project()` 只声明了 `LANGUAGES CXX`,
   没有 C,所以在 `UW_BUILD_CERES_SOLVER` 分支里加了一个作用域限定的
   `enable_language(C)`，不改动顶层语言声明。conda-forge 的 `ceres-solver` 默认构建
   会带 CUDA（拉约 943MB,含 libcublas/libcusolver）,显式选 `ceres-solver=2.2.0=cpu*`
   构建变体只要 8.6MB，装的时候两者都验证过（dry-run 对比）。
   **验证结果全部一次性通过,没有出现任何数学/接线 bug**：4 个新增单测（含单线程
   bit-determinism 对拍、四元数流形单位范数验证、fixed keyframe 不被移动）全部一次
   通过；真实端到端 demo 对拍（`synthetic_smoke.yaml` vs `synthetic_smoke_ceres.yaml`,
   同一个 bag）——两个后端在 12 关键帧的真实图上收敛到**逐位相同**的迭代次数（6）、
   cost（19.5387→4.49144）和 ATE（rmse=0.0665821m）；`ceres_v1` 端到端跑两遍轨迹
   `diff` 逐字节相同（确定性）；不编译 Ceres 支持时选 `ceres_v1` 在启动时直接报错
   退出（exit code 1),不静默回退；全量 CTest 在 `UW_BUILD_CERES_SOLVER=ON`
   （190/190）和默认 `OFF`（186/186,新增的 3 个 config 测试仍在,`adapters_ceres_tests`
   按预期整体跳过）下都全绿,`layer_dependencies` lint 两种配置都通过。
3. **[已完成 2026-08-23] 基准工具（1000 keyframe 压力场景 + 真实数据交叉验证）**：
   `tools/bench/solver_benchmark.sh`（跑三对场景：`synthetic_smoke`/`synthetic_stress`/
   `real_holoocean_vo`，各自对拍 gauss_newton_v1 vs ceres_v1，报告墙钟/迭代数/最终
   cost/ATE）、`configs/scenario/synthetic_stress.yaml`（1000 keyframe）。

   **规模场景的 rig 中途改过一次，原因值得记录**：第一版 `synthetic_stress.yaml` 直接
   复用 `synthetic_smoke.yaml` 那份带相机的 rig，实测发现完全测不出求解器本身的耗时——
   1000 个关键帧会触发每帧一次的稠密双目 + acoustic-optic 融合（逐像素处理，12 帧的
   demo 场景就已经产出 340 万 map evidence points），这条路径跟"求解器求解位姿图快
   不快"毫无关系，但在 1000 帧规模下墙钟时间完全被它主导（gauss_newton_v1 单次 LM
   迭代很快跑完，但后续的稠密融合跑了 100 秒以上仍未结束）。修法是新增一份不含相机的
   `configs/rig/example_auv_sonar_only.yaml`，只留 IMU/声呐/深度——干净地把压力场景
   限定到只测试求解器本身。

   **即使去掉稠密视觉路径，1000 关键帧对两个后端来说都是真实的规模墙**：校准测试量出
   `gauss_newton_v1` 单次 LM 迭代（999 个自由 keyframe × 7 维稠密 LDLT 分解）约 40 秒，
   完整 30 次迭代量级会跑到几十分钟——不是脚本能在交互式会话里等出来的时间，所以基准
   工具对压力场景用 `--max-iterations 3`（而不是各自的默认 30），比较"固定小迭代预算
   内两个后端各自的墙钟和 cost 下降"，而不是等谁先收敛。

   **实测结果**（本机单次运行,数字见下,不是跨机器/跨次运行稳定值）：

   | 场景 | 后端 | 墙钟 | 迭代数 | 最终 cost | ATE rmse | 收敛 |
   |---|---|---|---|---|---|---|
   | synthetic_smoke (12 kf) | gauss_newton_v1 | 3.76s | 6 | 4.49144 | 0.0666m | 是 |
   | synthetic_smoke (12 kf) | ceres_v1 | 3.73s | 6 | 4.49144 | 0.0666m | 是 |
   | synthetic_stress (1000 kf, 3 迭代预算) | gauss_newton_v1 | 117.4s | 3 | 827.594 | 0.2215m | 否（预算内） |
   | synthetic_stress (1000 kf, 3 迭代预算) | ceres_v1 | 173.8s | 4 | 827.427 | 0.2215m | 否（预算内） |
   | real_holoocean_vo (50 kf, 真实数据) | gauss_newton_v1 | 1.23s | 14 | 281.023 | 0.6667m | 否 |
   | real_holoocean_vo (50 kf, 真实数据) | ceres_v1 | 1.31s | 31 | 268.518 | 0.4700m | 否 |

   三个不同结论,分场景看：

   1. **12 关键帧合成场景：正确性证据非常干净**——两个后端收敛到逐位相同的迭代数、
      cost、ATE，跟之前 §6.1.1 的真实端到端 demo 对拍结论一致。
   2. **1000 关键帧规模场景：Ceres 在这个对比下没有赢，反而更慢**——两者 cost 下降量
      级相当,但 ceres_v1 每次迭代墙钟比 gauss_newton_v1 更长（174s/4≈43.5s/iter vs
      117s/3≈39s/iter）。原因不是 Ceres 本身慢，是这次对比刻意让两边都用稠密线性代数
      （`ceres::DENSE_QR` vs `GaussNewtonSolver` 手写的 Eigen `LDLT`）——QR 分解对同一个
      对称正定法方程比 Cholesky/LDLT 系族贵了大约一倍浮点操作量，观测到的墙钟差距和
      这个理论倍数量级吻合，不是接线问题。**这个结果本身就是答案的一部分**：稠密线性
      代数不管哪个后端来做，在 1000 关键帧规模都不实用（几十秒一次迭代）；Ceres
      真正可能带来规模优势的地方是稀疏求解器（`ceres::SPARSE_NORMAL_CHOLESKY`，
      本机的 SuiteSparse 已经链接好，能直接试）——这次基准没有跑，是下一步要做的对比，
      不是本次结论。
   3. **50 帧真实 HoloOcean 数据：两个后端都没有在 30 次迭代内收敛（真实噪声比合成
      数据难得多，与既有 memory 记录一致），但 Ceres 收敛过程明显更有效**——同样
      "stalled"，ceres_v1 达到的 cost 更低（268.5 vs 281.0）、ATE 更好（0.4700m vs
      0.6667m，约 30% 提升）。两个后端的默认 `require_converged: true` gate 都因此
      判定失败（exit code 2）——`real_holoocean_vo.yaml` 没有像压力场景那样关掉这条
      gate，这是预期行为不是脚本 bug，两次运行的 cost/ATE 数据仍然正确提取、正确打印。

   **对"该不该换默认后端"这个决策的建议**：现在还不该换。已有数据不支持"Ceres 在
   墙钟上更快"（稠密对比下反而更慢），但确实支持"Ceres 在真实噪声数据上收敛质量更好"
   ——这两个信号方向不一致，说明决策还没到能拍板的地步，下一步最有信息量的实验是把
   `synthetic_stress_ceres.yaml`/真实数据场景换成稀疏求解器（`SPARSE_NORMAL_CHOLESKY`）
   重新跑一遍，看规模墙钟结论会不会反转——如果反转了，"是否切换默认后端"这个问题就
   变成了"稀疏 Ceres 的收敛质量优势是否也保留、值不值得为此接受手写求解器之外的一个
   新依赖"，这才是真正需要用户拍板的那个决策点。
3. **[已完成 2026-08-23] SurfelMap 空间索引抽象**（独立于 1/2）：`include/mapping/surfel_map.hpp`
   新增 `SurfelSpatialIndex` 纯接口 + 构造函数注入点，默认行为不变，现有测试零改动
   （29/29 原测试通过，全量 178/178 CTest 通过，真实 demo ATE/迭代数/map evidence 数量
   与改动前逐位一致）。实现时把设计文档原先未展开的一个真实正确性细节想清楚并落了地：
   `CarveFreeSpace` 的 swap-and-pop 删除会让"索引"随每次删除漂移，如果索引给出的候选
   列表按原始顺序处理，后面的候选可能已经被更早的删除挪到别的槽位——解法是把候选按
   **降序**处理（去重后从大到小），可证明每次 swap-and-pop 只会动"当前尾部"和"正在处理
   的槽位"，两者在降序处理下总是 ≥ 任何还未处理的候选，所以更小的候选永远不会被更早的
   删除污染。新增 4 个等价性测试（含一个 200 点 + 20 条射线的随机化对拍，`std::mt19937_64`
   固定 seed 20260823），用一个测试专用的 `BruteForceReferenceIndex`（把
   `SurfelSpatialIndex` 接口实现成普通线性扫描）证明 SurfelMap 这一侧的 Notify*/query
   接线本身是对的，独立于以后接的具体第三方数据结构。
4. **[已完成 2026-08-23] nanoflann 适配器**：`adapters/spatial_index/`（`NanoflannSurfelIndex`,
   基于 `nanoflann::KDTreeSingleIndexIncrementalAdaptor`——单棵 weight-balanced 树、
   `alpha_deleted` 有界回收 tombstone，而不是另一个 dynamic adaptor（"logarithmic forest"）
   ——后者对同一 index 号重新 `addPoints` 会原地复用旧节点、不重新读取坐标，会在
   `NotifyMoved` 场景下悄悄搜索到过期坐标；`SurfelMap` 的合并语义恰恰是移动密集型，
   所以特意避开了这个坑，`NotifyMoved` 永远给移动后的点分配一个全新 nanoflann 内部 id，
   旧 id 打 tombstone，不复用）。CMake 走 `FetchContent`（`cmake/UwNanoflann.cmake`,
   固定 v1.10.0 tag,header-only INTERFACE target，不需要像 Ceres 那样开关式可选构建）。
   新增 5 个测试（`tests/adapters/spatial_index/nanoflann_surfel_index_test.cpp`），
   复用 mapping 侧已验证的等价性测试结构，另外专门加了一个边界回归测试——**实现过程中
   真的抓到一个边界 bug**：nanoflann 的 radius/distance 比较是**严格小于**（`<`），
   而 `SurfelMap::FindNearest` 自己是 `<=`（闭区间）；恰好落在 gate 边界上的点会被
   nanoflann 静默排除。修法不是简单放大查询半径再筛（那样在"最近点恰好在放大区间外、
   次近点在真实半径内"时会把次近点也漏掉），而是用 `radiusSearch` 拿到按距离升序排列
   的候选列表（半径故意放大一个极小 epsilon 越过边界)，再对每个候选用**未放大的精确
   半径**重新核验，取第一个通过的——这样保真最近邻语义，放大只解决"恰好卡在严格小于
   边界"的问题，不引入假阳性。3000 点随机化测试显示 6ms→1ms 的量级加速（诊断性质,
   非硬门禁,避免机器相关的 flaky 阈值）。
   **配置接入（`mapping.surfel_spatial_index` 配置项）未做，且暂时没有地方可接**：
   实现时发现 `SurfelMap`/`FuseDepthIntoSurfels` 目前在 `application`/`apps/replay_demo`
   里没有任何真实调用点——`replay_pipeline.cpp` 走的是 `map_backend:
   submap_point_cloud_v1`（`SubmapManager`），`SurfelMap` 只在测试里用到。给一个没人
   调用的代码路径接配置项没有意义，这一步等 `SurfelMap` 本身被接入真实回放管线（那是
   独立于本工作单元的另一个里程碑）之后再一起做。
5. **lint 覆盖盲区修复**（§6.3）：与步骤 2/4 中任一个新增 `adapters/` 子目录一起提交即可，不必
   单独排期，但不能被两者都遗漏。

### 11. 已拍板的决策（原“待用户复核”，用户已确认按建议执行）

#### 11.1 Ceres 引入方式：`find_package(Ceres CONFIG REQUIRED)`，走 conda-forge

不用 `FetchContent`。理由：

- Ceres 自带正确的 CMake config 导出（`CeresConfig.cmake`），`find_package` 是它的标准接入
  方式，跟仓库现有 `Eigen3`/`yaml-cpp` 走的是同一条路（`cmake/Dependencies.cmake` 里已经是
  `find_package(... REQUIRED NO_MODULE)` 模式），不是 MCAP SDK 那种“没有自己的
  CMakeLists.txt，只能 `FetchContent_Populate` 手动接”的特例（`cmake/UwMcap.cmake`）。
- 从源码 `FetchContent` 构建 Ceres 会连带拉 SuiteSparse/glog/gflags 一整条依赖链，构建时间
  和失败面都显著更大，跟本工作单元“先拿到基准数据、不急着定型”的目标不成比例。
- 落地时按 README「构建」一节的 conda-forge 回退路径，在 `uw_slam_build` 环境里装
  `ceres-solver`（conda-forge 有预编译包），跟 Eigen3/yaml-cpp 的既有获取方式一致，不需要
  额外走 CLAUDE.md 里那条本机 apt 代理坑（那是 ROS2 专属的 apt-only 路径，Ceres 不需要）。

#### 11.2 压力场景规模：`configs/experiment/synthetic_stress.yaml`，1000 keyframe

- 参考 memory 里记录的 production-readiness 目标（内部生产测试、成为 R&D 部门核心工具链），
  压力场景应该贴近“一次真实规划任务的时长”而不是任意选一个更大的数字。按现有
  `synthetic_smoke` 的关键帧节奏推算，1000 keyframe 大致对应一次十几到二十分钟量级的
  典型作业时长，是比当前 demo（几十个 keyframe）跨越到接近真实任务规模的合理台阶，同时
  仍在合成数据生成器（`synth_bag_gen`）单次运行可接受的时间内。
- 图连接稀疏度沿用现有 `synthetic_smoke` 的生成逻辑（相邻 keyframe 相对位姿因子 +
  周期性声呐/深度证据），不引入新的连接模式——压力测试只放大规模，不改变图的拓扑特征，
  这样两个后端的对比数据才能干净地归因到“规模变化”而不是“图结构变化”。
- 后续如果实测发现 1000 这个数字不够或过头（比如 Ceres 在这个规模下就已经明显胜出/明显不如
  预期），基准工具本身应该做成 keyframe 数量可配置，不是把 1000 写死在测试代码里——具体见
  §6/§9 的基准工具设计。

#### 11.3 新配置项默认值：保持现状（`gauss_newton` / `brute_force`）

`solver_backend` 默认 `gauss_newton`，`mapping.surfel_spatial_index` 默认 `brute_force`。两个
新路径落地后默认不生效，需要显式在 experiment 配置里选择才会启用——直到 §9 的基准数据支持
切换默认值为止。这跟本文档从一开始的偏保守立场一致（§2.1 目标 4：“不改变默认后端，除非基准
结果支持这么做”），不因为“建好了就默认打开”而绕过这条已经写进目标里的决策门。
