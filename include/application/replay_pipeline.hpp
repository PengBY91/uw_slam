// 离线回放管线的对外接口：RunReplayPipeline 是整条离线 SLAM 主线的编排入口
// （apps/replay_demo.cpp 只负责解析参数然后调它）。
//
// 这个头文件里除了入口函数，其余声明大多是**为了可单测而从 RunReplayPipeline 里
// 抽出来的纯函数**。抽出来的判断标准是"这段逻辑有值得单独验证的决策，但混在
// 1500 行的编排里没法测"：DecideTrackingStatus 的状态优先级、BuildStateSnapshot
// 的确定性排序、CountDepthContributions 的口径、CheckGraphObservability 的结构性
// 检查、EvaluateReplayGates 的门禁判定。它们都不做任何 IO。
#pragma once

#include <string>
#include <vector>

#include "domain/domain.hpp"
#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "evaluation/trajectory_metrics.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/config.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::application {

struct ReplayOptions {
  std::string bag_path;
  std::string experiment_path;
  std::string out_prefix = "/tmp/replay_demo";
  int max_iterations = -1;
  bool align_ate = false;
};

int RunReplayPipeline(const ReplayOptions& options,
                      const std::string& git_commit);

// 放在 application 层做这个转换，是为了让 runtime 和 frontends 保持平级（谁也不
// 依赖谁），同时又保证配置里每一项声呐默认值都真的落到了前端参数上——如果让
// frontends 直接读 runtime 的配置结构，这两层就绑死了。
uw::frontends::SonarCfarFrontendParams BuildSonarCfarFrontendParams(
    const uw::runtime::SonarFrontendConfig& config);

// 抽出来单测的纯决策函数（不做 IO）：给定批量求解器是否收敛，以及（在
// estimator_mode == "stereo_landmark_vo" 时）该 keyframe **当时**的 VO 前端健康度，
// 决定这个 keyframe 的快照该报什么 tracking_status。
//
// 强调"当时"：绝不能拿前端跑完整轮之后的**最终**健康度去追溯性地覆盖所有历史
// keyframe（见 RunReplayPipeline 里的 vo_health_by_keyframe）——那样第 3 帧明明跟踪
// 良好，会因为第 40 帧丢了而被改写成 LOST，历史就假了。
//
// 优先级：LOST > DEGRADED > TRACKING。含义是两条都不能被"抵消"：求解器 stalled 时
// 即便 VO 自身健康也不许报 TRACKING；VO 报 STATUS_UNAVAILABLE 时也不许因为求解器
// 恰好收敛就降格成只是 DEGRADED。
struct ReplayTrackingInputs {
  bool solver_converged = false;
  bool vo_enabled = false;
  uw::domain::HealthReport::Status vo_health = uw::domain::HealthReport::STATUS_UNSPECIFIED;
};

uw::domain::StateSnapshot::TrackingStatus DecideTrackingStatus(const ReplayTrackingInputs& inputs);

// 抽出来单测的纯构造函数（不依赖 IO / StateStore）。
//
// contributing_evidence 在写入前会按 evidence_id 排序并去重：这样只要逻辑输入相同，
// 输出就逐字节相同，与证据被收集的先后顺序无关——这正是
// tests/integration/determinism_test.sh 真正在把关的契约。
struct StateSnapshotInputs {
  std::string state_id;
  uint64_t state_version = 0;
  uw::sensor_models::Pose3 pose;
  uw::domain::Stamp capture_timestamp;
  std::string calibration_version;
  std::vector<uw::domain::EvidenceId> contributing_evidence;
  uw::domain::StateSnapshot::TrackingStatus tracking_status =
      uw::domain::StateSnapshot::TRACKING_STATUS_UNSPECIFIED;
};

uw::domain::StateSnapshot BuildStateSnapshot(const StateSnapshotInputs& inputs);

// 按 contribution_mask 统计一份融合深度里的有效像素分别来自哪种来源。
//
// **必须在 mapping::BuildMapEvidenceFromFusedDepth 之前调用**：那个函数只输出点，
// 不保留每个点的来源信息，转换之后就统计不出来了。
//
// 无效像素（contribution_mask == DEPTH_CONTRIBUTION_INVALID，或 valid_mask 位为 0）
// 两类都不计入。
struct MapContributionCounts {
  uint64_t optical_only_points = 0;
  uint64_t acoustic_optic_points = 0;
};

MapContributionCounts CountDepthContributions(const uw::domain::FusedDepthMeasurement& fused);

// 一轮运行的结果，在 RunReplayPipeline 末尾以 `summary.<key>=<value>` 的形式打印一次。
//
// 运行过程中别处那些散文式的诊断输出是给人看日志用的；**这里这份才是脚本解析的契约**
// （tests/integration/imu_preintegration_smoke_test.sh 就在解析它）。所以它被做成
// "结构体 + 纯格式化函数"，而不是散落各处的 std::cout——后者要靠正则去拼，改一行输出
// 就可能悄悄弄坏下游脚本。
struct ReplayRunSummary {
  std::string estimator_mode;
  std::string solver;
  bool solver_converged = false;
  int solver_iterations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;

  int keyframe_count = 0;
  int keyframe_boundary_count = 0;
  int imu_factor_count = 0;
  int imu_interval_rejected_count = 0;
  int relative_pose_factor_count = 0;
  int loop_closure_factor_count = 0;
  int sonar_range_factor_count = 0;
  int depth_factor_count = 0;
  int landmark_count = 0;

  // estimator_mode 为 imu_preintegration 时取 "stationary" / "wide_velocity_prior"；
  // 其他模式一律 "none"——它们没有需要初始化的惯性状态。
  std::string initialization = "none";
  // 结构可观测性：非固定参数块的**最小**维度（每个位姿算 6，而不是四元数那 7 个
  // 过参数化的量）对上真正约束它们的残差行数。用 7 会高估自由度，把本来欠定的图
  // 误判成定的。
  int free_parameter_dim = 0;
  int residual_dim = 0;

  double ate_rmse_m = 0.0;
  int ate_matched_poses = 0;
};

std::string FormatReplayRunSummary(const ReplayRunSummary& summary);

// 在求解**之前**做的结构性检查。
//
// 动机很实在：一个没有被任何残差引用到的非固定参数块，会让正规方程奇异；而 LM 的
// 阻尼项会心安理得地给它返回一个看着像模像样、实则完全任意的值，**不会报错**。
//
// 返回发现的问题列表（结构健全时为空）。非空时必须 fail-closed 让整轮失败，不能
// 硬着头皮解下去——解出来的那个数没有任何意义。
struct GraphObservability {
  int free_parameter_dim = 0;
  int residual_dim = 0;
  std::vector<std::string> problems;
};

GraphObservability CheckGraphObservability(uw::estimation::PoseGraphProblem& problem);

// 抽出来单测的纯门禁判定（不做 IO、不与退出码耦合）：给定一轮运行的结果，返回所有
// 未通过的门禁说明（人可读），全部通过则返回空 vector。返回非空时调用方应以非零码
// 退出（见 RunReplayPipeline 里的实际用法）。
//
// require_converged 默认开启（见 PlatformDefaultsConfig）；其余门禁都是 opt-in——
// 填 0 或负数即表示关闭该项。
std::vector<std::string> EvaluateReplayGates(const uw::runtime::PlatformDefaultsConfig& defaults,
                                             const uw::estimation::GaussNewtonSummary& solver,
                                             const uw::evaluation::AteResult& ate, int num_landmarks,
                                             const MapContributionCounts& contributions,
                                             int num_acoustic_optic_accepted);

}  // namespace uw::application
