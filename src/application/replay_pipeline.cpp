// 端到端离线回放：MCAP bag -> SonarCfarFrontend -> FactorBuilders ->
// PoseGraphProblem -> GaussNewtonSolver -> StateStore -> SubmapManager ->
// 轨迹 + ATE + RunManifest。
//
// 它是"架构设计里划分的那些层真的能拼起来跑通"的具体证明，输入通常是
// apps/synth_bag_gen.cpp 造的合成数据（合成数据的作用是提供精确已知的真值和可复现
// 的输入，不是因为跑不了仿真器——真实 HoloOcean 录制也能经
// configs/experiment/real_holoocean_vo.yaml 走同一条路）。
//
// ============================ 本文件主要逻辑 ============================
//
// RunReplayPipeline 是一条长直线，没有回头路，大致九段：
//
//   1. 读配置：分层加载 experiment（defaults/rig/scenario/experiment），并
//      fail-fast 校验里面选择的 frontend/estimator/solver 标识符。CLI 参数最后生效。
//   2. 读 bag：**只做一趟有序遍历**，经 PumpEvents 灌进 ReplayInputAccumulator，
//      得到一份扁平且身份已校验的 ReplayInputData。之后所有消费点都是在这份数据上
//      过滤，不再重复读 bag。
//   3. 立体校正：如果 rig 带相机，先一次性建好校正上下文——下游所有吃相机帧的东西
//      都硬性要求 is_rectified()==true。
//   4. 定 keyframe 集合与时间戳：keyframe 身份来自线上字段，不从时间推导。
//   5. 建因子图：相对位姿（黑箱 VIO 证据 / 实时 VO / IMU 预积分三选一）、声呐 range、
//      深度、可选回环闭合。kf0 固定为 anchor。
//   6. 解之前先查结构可观测性（CheckGraphObservability），欠定就 fail-closed。
//   7. 求解：默认手写 GN/LM，可切 Ceres。
//   8. 出结果：轨迹 TUM 文件、StateStore 快照、SubmapManager 地图、声光融合与地图桥、
//      ATE 评测、RunManifest。
//   9. 判门禁（EvaluateReplayGates），有失败就非零退出。
//
// 【v1 的限制，明写出来而不是藏着】
//  - 没有真正的可靠性调度器 / 学习模型信息量上限（架构文档 8.4 节）：下面那些
//    sqrt-information 都是固定常数，不是经过标定的可靠性策略。
//  - 声呐 range 因子的路标关联现在走的是真实的 SubmapManager::QueryNearestPoint()
//    （见下面声呐那段）：检测落在某个已知路标的 kLandmarkGateM 半径内就复用它存的
//    位置，落空则新插入一个。
//    仍属 v1 的部分（对应 sonar_range_factor 里写明的限制）：单靠声呐的 range+bearing
//    观测不到 elevation，所以新插入路标的 z 是个占位值（与传感器同高），后续没有任何
//    环节去精化它——既没有联合路标估计，也没有考虑协方差的马氏门控，只是一个固定的
//    欧氏半径。
//  - 声呐：每帧 SonarFrame 只消费排名第一的那个 DBSCAN 聚类（这是
//    schemas/proto/uw/domain/hypothesis.proto 里写明的 v1 规则——线上契约给多假设
//    消费者留了位置，但本 app 不是那种消费者）。
//    synth_bag_gen 是"量程内每个目标各发一次 ping"，而不是"一次 ping 覆盖所有目标"，
//    所以在**本 demo 的场景下**这么做并不丢信息；但它不是通用的"单 ping 多目标"模型，
//    这条注意事项见 synth_bag_gen 自己的头注释。
//  - 分层配置（defaults/rig/scenario/experiment，架构文档 14.2 节）通过 --experiment
//    覆盖了求解器选项和各因子的 sqrt-information 常数（见
//    uw::runtime::LoadExperimentConfig）。
//    其中 estimator_mode 和 landmark_detector 是**真的会分派**的（见下面
//    stereo_landmark_vo 那段）；而 sonar_frontend/optical_frontend/map_backend 目前
//    各自只有一个硬编码实现（sonar_range_v1 + depth_v1 + submap_point_cloud_v1），
//    因为第二个实现还不存在。不过自从有了 ValidateExperimentConfigSelections()，
//    在 experiment YAML 里写别的名字会在启动时就失败，而不是静默地照跑那唯一一条。
//  - defaults.warmup_seconds（默认 0）：预热窗口内的 keyframe 仍然留在图里、靠相对
//    位姿因子做航位推算，但**暂不**给它们加声呐 range / 深度这类"绝对参考"因子，直到
//    窗口结束。无论如何 kf0 都保持为固定 anchor。理由见下面预热那段。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sys/utsname.h>

#ifdef UW_HAVE_CERES_SOLVER
#include "adapters/ceres/ceres_pose_graph_solver.hpp"
#endif
#include "application/event_pump.hpp"
#include "application/replay_input_accumulator.hpp"
#include "application/replay_pipeline.hpp"
#include "domain/domain.hpp"
#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "estimation/state_store.hpp"
#include "evaluation/trajectory_metrics.hpp"
#include "factor_builders/depth_factor_builder.hpp"
#include "factor_builders/imu_preintegration_factor_builder.hpp"
#include "factor_builders/inertial_prior_residual.hpp"
#include "factor_builders/relative_pose_factor_builder.hpp"
#include "factor_builders/sonar_range_factor_builder.hpp"
#include "frontends/acoustic_optic_depth_fusion_frontend.hpp"
#include "frontends/imu_preintegration_frontend.hpp"
#include "frontends/imu_stationary_initializer.hpp"
#include "frontends/loop_closure_frontend.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "frontends/stereo_landmark_vo_frontend.hpp"
#include "frontends/stereo_optical_depth_frontend.hpp"
#include "mapping/acoustic_optic_map_bridge.hpp"
#include "mapping/submap_manager.hpp"
#include "opencv_adapters/stereo_rectifier.hpp"
#include "runtime/acoustic_optic_synchronizer.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_event_source.hpp"
#include "runtime/run_manifest.hpp"
#include "sensor_models/geometry.hpp"
#include "sensor_models/imu_preintegration.hpp"
#include "sensor_models/sonar_beam_model.hpp"

using uw::sensor_models::Pose3;

namespace {

std::string ToTumLine(double timestamp_s, const Pose3& pose) {
  std::ostringstream out;
  out.precision(9);
  out << timestamp_s << " " << pose.translation.x() << " " << pose.translation.y() << " "
      << pose.translation.z() << " " << pose.rotation.x() << " " << pose.rotation.y() << " "
      << pose.rotation.z() << " " << pose.rotation.w();
  return out.str();
}

// 以下是填 RunManifest 的几个小工具（见 docs/archive/uw-slam-production-readiness-
// and-roadmap-2026-08-21.md 5.6 节：这些字段以前一直是静默空着的）。
// 刻意保持简单——不为了填一点溯源元数据就引入新依赖。

std::string NowIso8601Utc() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm utc_tm{};
  gmtime_r(&now_c, &utc_tm);
  std::ostringstream out;
  out << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

// 不是密码学哈希。它只需要够用来发现"这份配置/标定文件的字节相比上一轮变了"，
// 用于 manifest 溯源，**不用于抵抗篡改**。别把它当完整性校验用。
std::string Fnv1aHex(const std::string& data) {
  uint64_t hash = 1469598103934665603ull;  // FNV 偏移基准值
  for (unsigned char c : data) {
    hash ^= c;
    hash *= 1099511628211ull;  // FNV 质数
  }
  std::ostringstream out;
  out << std::hex << hash;
  return out.str();
}

std::string ReadFileBytesOrEmpty(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return "";
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

std::string DetectOsInfo() {
  struct utsname info{};
  if (uname(&info) != 0) return "unknown";
  return std::string(info.sysname) + " " + info.release + " " + info.machine;
}

// 只对 Linux 有效（按 CLAUDE.md，本仓库的开发/CI 环境就是 Linux）；非 Linux 上
// 拿到 "unknown" 就好，不会编译失败。
std::string DetectCpuInfo() {
  std::ifstream cpuinfo("/proc/cpuinfo");
  std::string line;
  while (std::getline(cpuinfo, line)) {
    const auto colon = line.find(':');
    if (colon != std::string::npos && line.compare(0, 10, "model name") == 0) {
      std::string value = line.substr(colon + 1);
      const auto first = value.find_first_not_of(' ');
      return first == std::string::npos ? "unknown" : value.substr(first);
    }
  }
  return "unknown";
}

}  // namespace

uw::domain::StateSnapshot::TrackingStatus uw::application::DecideTrackingStatus(
    const ReplayTrackingInputs& inputs) {
  if (inputs.vo_enabled && inputs.vo_health == uw::domain::HealthReport::STATUS_UNAVAILABLE) {
    return uw::domain::StateSnapshot::TRACKING_STATUS_LOST;
  }
  if (!inputs.solver_converged ||
      (inputs.vo_enabled && inputs.vo_health == uw::domain::HealthReport::STATUS_SUSPECT)) {
    return uw::domain::StateSnapshot::TRACKING_STATUS_DEGRADED;
  }
  return uw::domain::StateSnapshot::TRACKING_STATUS_TRACKING;
}

uw::domain::StateSnapshot uw::application::BuildStateSnapshot(const StateSnapshotInputs& inputs) {
  uw::domain::StateSnapshot snapshot;
  snapshot.mutable_state_id()->set_value(inputs.state_id);
  snapshot.mutable_state_version()->set_value(inputs.state_version);
  *snapshot.mutable_capture_timestamp() = inputs.capture_timestamp;
  *snapshot.mutable_pose_wb() = inputs.pose.ToProto();
  snapshot.set_tracking_status(inputs.tracking_status);
  snapshot.mutable_calibration_version()->set_value(inputs.calibration_version);

  std::vector<std::string> evidence_values;
  evidence_values.reserve(inputs.contributing_evidence.size());
  for (const auto& id : inputs.contributing_evidence) evidence_values.push_back(id.value());
  std::sort(evidence_values.begin(), evidence_values.end());
  evidence_values.erase(std::unique(evidence_values.begin(), evidence_values.end()),
                        evidence_values.end());
  for (const auto& value : evidence_values) snapshot.add_contributing_measurements()->set_value(value);
  return snapshot;
}

uw::application::MapContributionCounts uw::application::CountDepthContributions(
    const uw::domain::FusedDepthMeasurement& fused) {
  MapContributionCounts counts;
  for (unsigned char byte : fused.contribution_mask()) {
    if (byte == static_cast<unsigned char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY)) {
      ++counts.optical_only_points;
    } else if (byte == static_cast<unsigned char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC)) {
      ++counts.acoustic_optic_points;
    }
  }
  return counts;
}

std::string uw::application::FormatReplayRunSummary(
    const uw::application::ReplayRunSummary& summary) {
  std::ostringstream out;
  auto emit = [&out](const char* key, const std::string& value) {
    out << "summary." << key << "=" << value << "\n";
  };
  auto emit_int = [&](const char* key, long long value) { emit(key, std::to_string(value)); };
  auto emit_double = [&](const char* key, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9f", value);
    emit(key, buffer);
  };

  emit("estimator_mode", summary.estimator_mode);
  emit("solver", summary.solver);
  emit("solver_converged", summary.solver_converged ? "true" : "false");
  emit_int("solver_iterations", summary.solver_iterations);
  emit_double("initial_cost", summary.initial_cost);
  emit_double("final_cost", summary.final_cost);
  emit_int("keyframe_count", summary.keyframe_count);
  emit_int("keyframe_boundary_count", summary.keyframe_boundary_count);
  emit_int("imu_factor_count", summary.imu_factor_count);
  emit_int("imu_interval_rejected_count", summary.imu_interval_rejected_count);
  emit_int("relative_pose_factor_count", summary.relative_pose_factor_count);
  emit_int("loop_closure_factor_count", summary.loop_closure_factor_count);
  emit_int("sonar_range_factor_count", summary.sonar_range_factor_count);
  emit_int("depth_factor_count", summary.depth_factor_count);
  emit_int("landmark_count", summary.landmark_count);
  emit("initialization", summary.initialization);
  emit_int("free_parameter_dim", summary.free_parameter_dim);
  emit_int("residual_dim", summary.residual_dim);
  emit_double("ate_rmse_m", summary.ate_rmse_m);
  emit_int("ate_matched_poses", summary.ate_matched_poses);
  return out.str();
}

uw::application::GraphObservability uw::application::CheckGraphObservability(
    uw::estimation::PoseGraphProblem& problem) {
  uw::application::GraphObservability result;

  // 用 (kind, id) 做键：同一个 keyframe 的位姿块和惯性块是两个各自独立、各自被约束
  // 的参数。把它们混为一谈，恰好会掩盖这个函数存在的意义所要抓的那种故障——一个
  // 既没有 IMU 边也没有先验的惯性状态，紧挨着一个同名且约束良好的位姿。
  std::set<std::pair<int, std::string>> free_blocks;
  for (const auto& block : problem.MutableAllParameterBlocks()) {
    if (block.fixed) continue;
    free_blocks.insert({static_cast<int>(block.ref.kind), block.ref.keyframe_id});
  }

  std::set<std::pair<int, std::string>> referenced;
  for (const auto& binding : problem.ResidualBindings()) {
    if (binding.involved_parameters == nullptr) continue;
    bool touches_a_free_block = false;
    for (const auto& ref : *binding.involved_parameters) {
      const std::pair<int, std::string> key{static_cast<int>(ref.kind), ref.keyframe_id};
      referenced.insert(key);
      if (free_blocks.count(key) > 0) touches_a_free_block = true;
    }
    // 只连到固定块的残差（比如 anchor 自己的深度因子）本质上是个常数：它对求解器
    // 会动的任何量都不提供信息。把它的行数算进来会虚增总数，反而掩盖别处真实的
    // 秩亏。
    if (touches_a_free_block) result.residual_dim += binding.block->ResidualDim();
  }

  for (const auto& block : problem.MutableAllParameterBlocks()) {
    if (block.fixed) continue;  // 固定块不参与求解
    // 取**最小维度**而不是块的存储大小：位姿存了 7 个数，但只有 6 个自由度
    // （四元数是归一化的，残差沿那个方向不变——那一列为什么真的是零，见
    // imu_preintegration_residual.hpp）。
    //
    // 如果按 7 算：一条本来完全定的相对位姿链（每条边 6 行残差，对每个 keyframe
    // 7 "列"）会被判成结构奇异，于是所有没有深度或声呐因子来补差额的 experiment 都会
    // 硬失败。这是个会让检查本身变成误报源的错误。
    result.free_parameter_dim +=
        block.ref.kind == uw::estimation::PoseGraphProblem::ParameterKind::kPose ? 6 : block.size;
    if (referenced.count({static_cast<int>(block.ref.kind), block.ref.keyframe_id}) == 0) {
      result.problems.push_back(
          std::string(block.ref.kind == uw::estimation::PoseGraphProblem::ParameterKind::kInertial
                          ? "inertial state '"
                          : "keyframe pose '") +
          block.ref.keyframe_id + "' is free but referenced by no residual — the normal equations "
          "are singular in those " + std::to_string(block.size) + " columns");
    }
  }

  if (result.residual_dim < result.free_parameter_dim) {
    result.problems.push_back("the graph has " + std::to_string(result.residual_dim) +
                              " residual rows for " + std::to_string(result.free_parameter_dim) +
                              " free parameters — underdetermined, and LM damping would return an "
                              "arbitrary answer rather than fail");
  }
  return result;
}

std::vector<std::string> uw::application::EvaluateReplayGates(
    const uw::runtime::PlatformDefaultsConfig& defaults, const uw::estimation::GaussNewtonSummary& solver,
    const uw::evaluation::AteResult& ate, int num_landmarks, const MapContributionCounts& contributions,
    int num_acoustic_optic_accepted) {
  std::vector<std::string> failures;
  if (defaults.require_converged && !solver.converged) {
    failures.push_back("solver did not converge (" + std::to_string(solver.iterations) +
                       " iterations, stalled)");
  }
  if (defaults.min_matched_ate_poses > 0 &&
      static_cast<int>(ate.num_matched_poses) < defaults.min_matched_ate_poses) {
    failures.push_back("matched ATE poses " + std::to_string(ate.num_matched_poses) + " < required " +
                       std::to_string(defaults.min_matched_ate_poses));
  }
  if (defaults.max_ate_rmse_m >= 0.0 && ate.rmse_m > defaults.max_ate_rmse_m) {
    failures.push_back("ATE rmse " + std::to_string(ate.rmse_m) + "m > max " +
                       std::to_string(defaults.max_ate_rmse_m) + "m");
  }
  const uint64_t total_map_points = contributions.optical_only_points + contributions.acoustic_optic_points;
  if (defaults.require_nonempty_map && (static_cast<uint64_t>(num_landmarks) + total_map_points) == 0) {
    failures.push_back("map is empty (0 landmarks discovered, 0 map evidence points)");
  }
  if (defaults.min_acoustic_optic_accepted > 0 &&
      num_acoustic_optic_accepted < defaults.min_acoustic_optic_accepted) {
    failures.push_back("acoustic-optic accepted associations " +
                       std::to_string(num_acoustic_optic_accepted) + " < required " +
                       std::to_string(defaults.min_acoustic_optic_accepted));
  }
  if (defaults.min_acoustic_optic_map_points > 0 &&
      contributions.acoustic_optic_points < static_cast<uint64_t>(defaults.min_acoustic_optic_map_points)) {
    failures.push_back("acoustic-optic map points " + std::to_string(contributions.acoustic_optic_points) +
                       " < required " + std::to_string(defaults.min_acoustic_optic_map_points));
  }
  return failures;
}

uw::frontends::SonarCfarFrontendParams uw::application::BuildSonarCfarFrontendParams(
    const uw::runtime::SonarFrontendConfig& config) {
  uw::frontends::SonarCfarFrontendParams params;
  params.cfar.num_training_cells = config.training_cells;
  params.cfar.num_guard_cells = config.guard_cells;
  params.cfar.probability_false_alarm = config.probability_false_alarm;
  params.detector_threshold = static_cast<uint8_t>(config.detector_threshold);
  params.dbscan_eps_m = config.dbscan_eps_m;
  params.dbscan_min_samples = config.dbscan_min_samples;
  params.default_range_sigma_m = config.default_range_sigma_m;
  params.default_bearing_sigma_rad = config.default_bearing_sigma_rad;
  return params;
}

int uw::application::RunReplayPipeline(const ReplayOptions& opt,
                                       const std::string& git_commit) {
  if (opt.bag_path.empty()) {
    std::cerr << "replay pipeline requires a bag path\n";
    return 1;
  }
  const std::string run_start_time_iso8601 = NowIso8601Utc();
  // 第 1 层：平台默认值。来源要么是 --experiment 的 defaults: 层，要么是这个结构体
  // 内建的值——就是以前直接硬编码在下面的 kRelativePoseSqrtInfo 之类，现在被提到
  // uw::runtime::PlatformDefaultsConfig 里，使它们**只有一处定义**
  // （configs/defaults/platform.yaml）。
  uw::runtime::PlatformDefaultsConfig defaults;
  bool write_run_manifest = true;
  // 只有当 --experiment 加载的 rig **带相机**时才会赋值。它是下面那趟声光处理的开关，
  // 这样"不给 experiment"的路径（tests/l2_replay/determinism_test.sh 在用）可以被证明
  // 完全没变。
  std::optional<uw::domain::RigCalibrationSnapshot> rig;
  // 在 stereo_landmark_vo_frontend 出现之前，这个字段是"读了但没用"的（见 README
  // 的"已知边界"一节——experiment 里那些 frontend/estimator/map-backend 选择项当时
  // 并不真的分派到任何分支）。
  //
  // "stereo_landmark_vo" 是第一个真的起作用的值：它把下面相对位姿那段，从"读
  // synth_bag_gen 烘焙进 bag 的 真值+噪声 黑箱 VIO 证据"，换成"从 /raw/camera/left,
  // right 实时算出来的真实证据"。为什么光有这个字符串不够、还需要 `rig`，见那一段。
  std::string estimator_mode = "black_box_vio";
  // estimator_mode 为 "imu_preintegration" 时改用这个。之所以和上面的 `rig` 分开：
  // 那个刻意只在 rig **带相机**时才填（它是声光那趟的开关），而惯性这条路需要的是
  // imu_noise 配置块和 base_link->imu_link 这条外参边，而这些在诸如
  // configs/rig/example_auv_sonar_only.yaml 这类**没有相机**的 rig 上也必须能拿到。
  std::optional<uw::domain::RigCalibrationSnapshot> imu_rig;
  std::string landmark_detector = "bright_blob";
  // RunManifest 的溯源信息（5.6 节）：尽力而为，不是保证。这里的 seed 反映的是本
  // experiment 声明的 scenario seed——也就是"用 synth_bag_gen 该生成出配套 bag 的那个
  // 种子"。但如果 bag 是别处来的（比如一段真实 HoloOcean 录制），它未必出自这个种子。
  uint64_t scenario_seed = 0;
  std::string config_hash;
  std::string calibration_hash;
  if (!opt.experiment_path.empty()) {
    const auto config = uw::runtime::LoadExperimentConfig(opt.experiment_path);
    if (const auto error = uw::runtime::ValidateExperimentConfigSelections(config)) {
      std::cerr << "invalid experiment config " << opt.experiment_path << ": " << *error << "\n";
      return 1;
    }
    defaults = config.defaults;
    write_run_manifest = config.write_run_manifest;
    if (config.rig.cameras_size() > 0) rig = config.rig;
    estimator_mode = config.estimator_mode;
    if (config.estimator_mode == "imu_preintegration") imu_rig = config.rig;
    landmark_detector = config.landmark_detector;
    scenario_seed = config.scenario.seed;
    config_hash = Fnv1aHex(ReadFileBytesOrEmpty(opt.experiment_path));
    calibration_hash = Fnv1aHex(config.rig.SerializeAsString());
    std::cout << "loaded experiment config: " << opt.experiment_path << " (sonar_frontend="
              << config.sonar_frontend << ", estimator_mode=" << config.estimator_mode << ")\n";
  }
  if (opt.max_iterations > 0) defaults.max_iterations = opt.max_iterations;  // CLI 优先级最高

  // 对 bag 只做**一趟有序遍历**（见 docs/archive/superpowers/plans/2026-08-24-
  // live-replay-unified-ingress.md Task 4）：这个函数以前按 topic、按 payload 类型
  // 各调一次 ReadMcapMessages<T>，现在统一变成在这一份 ReplayInputData 上做过滤。
  // 每个消费点下面都写了它原来的过滤条件是怎么在扁平 vector 上复现的。
  uw::runtime::McapEventSource event_source(opt.bag_path);
  uw::application::ReplayInputAccumulator accumulator;
  const auto pump_report = uw::application::PumpEvents(event_source, accumulator);
  if (pump_report.status == uw::runtime::EventSourceStatus::kOpenFailed) {
    std::cerr << "failed to open bag: " << opt.bag_path << "\n";
    return 1;
  }
  if (pump_report.unknown_topic_count > 0 || pump_report.parse_failure_count > 0) {
    std::cout << "warning: bag scan saw " << pump_report.unknown_topic_count
              << " unknown-topic message(s) and " << pump_report.parse_failure_count
              << " topic/schema-mismatched message(s) (see EventSourceReport)\n";
  }
  if (accumulator.Diagnostics().HasErrors()) {
    std::cerr << "replay input identity error(s):\n";
    for (const auto& message : accumulator.Diagnostics().messages) std::cerr << "  - " << message << "\n";
    return 1;
  }
  const uw::application::ReplayInputData& input = accumulator.Data();

  // 校正只做一次，且放在最前面。下游所有吃相机帧的东西
  // （stereo_landmark_vo_frontend、stereo_optical_depth_frontend，以及建在它之上的
  // 声光融合/地图桥链路）现在都**硬性要求** is_rectified()==true
  // （即 StereoGeometry::Resolve 的"已校正像对"契约，见 sensor_models/camera_model.hpp）。
  //
  // 这里 Create() 失败意味着这个 rig 根本无法校正，那么下面每一趟依赖相机的处理都会
  // 静默地产出零证据。与其那样，不如立刻让整轮失败——这更诚实。
  std::optional<uw::opencv_adapters::StereoRectificationContext> rectification_context;
  if (rig.has_value()) {
    uw::opencv_adapters::StereoRectificationParams rectification_params;
    rectification_params.alpha = defaults.stereo_rectification.alpha;
    rectification_params.rectified_frame_suffix = defaults.stereo_rectification.frame_suffix;
    rectification_params.crop_policy =
        defaults.stereo_rectification.crop_policy == "common_valid_roi"
            ? uw::opencv_adapters::RectificationCropPolicy::kCommonValidRoi
            : uw::opencv_adapters::RectificationCropPolicy::kFullCanvas;
    std::string rectification_error;
    rectification_context = uw::opencv_adapters::StereoRectificationContext::Create(
        *rig, rectification_params, &rectification_error);
    if (!rectification_context.has_value()) {
      std::cerr << "stereo rectification failed: " << rectification_error << "\n";
      return 1;
    }
  }

  // 预热窗口（defaults.warmup_seconds，0 表示关闭）：这里用 synth_bag_gen 固定的
  // 5 Hz keyframe 间隔（"kf0"、"kf1"…… 每 kKeyframeIntervalS 一个）把它换算成一个
  // keyframe 计数截止点。
  //
  // **这是本文件里仅剩的、还假设那套命名/间距约定的地方**——下面其他所有 keyframe
  // 身份都直接来自 ReplayInputData（线上的 observation id），绝不从时间推。
  //
  // 窗口内的 keyframe 仍然进图、仍然获得相对位姿（航位推算）因子——预热期靠航位推算
  // 往前走，正是一个真实估计器在还不信任绝对修正时会做的事——但它们**不会**拿到
  // 声呐 range 或深度因子（下面那两类"绝对参考"因子会跳过它们）。
  //
  // 这就是"在 VIO 零偏收敛之前不要融合绝对定位"这条经验在批量位姿图里的对应物
  // （这个想法的出处见 runtime/include/uw/runtime/config.hpp 里 defaults.warmup_seconds
  // 的注释）。
  //
  // 无论 warmup_seconds 取多少，kf0 始终是那个固定 anchor：改锚到后面某个 keyframe，
  // 就得知道那个 keyframe 真实的 x/y/yaw，而这三个量在本图里根本没有被观测
  // （x/y/yaw 是真正的 gauge 自由度，见下）。
  //
  // kf0 之所以可以对这三个量直接取 Identity()，是因为 synth_bag_gen 在构造上就把 kf0
  // 放在世界坐标原点——**不是因为"第一个 keyframe"在一般意义上有什么特殊**。
  constexpr double kKeyframeIntervalS = 0.2;  // 对应 synth_bag_gen 的 5 Hz 间隔
  const int warmup_keyframes = defaults.warmup_seconds > 0.0
                                    ? static_cast<int>(std::ceil(defaults.warmup_seconds / kKeyframeIntervalS))
                                    : 0;
  std::unordered_set<std::string> warmup_keyframe_ids;
  for (int i = 0; i < warmup_keyframes; ++i) warmup_keyframe_ids.insert("kf" + std::to_string(i));
  if (warmup_keyframes > 0) {
    std::cout << "warmup_seconds=" << defaults.warmup_seconds << " -> " << warmup_keyframes
              << " keyframes dead-reckoned only (no sonar/depth fusion)\n";
  }

  // 固定 anchor keyframe "kf0" 的 gauge 选择。
  //
  // 对"相对位姿 + 纯 range"这样的图，x/y/yaw 确实是 gauge 自由度（没有任何东西观测
  // 绝对水平位置或朝向），所以这三个量取 identity 是个无所谓的任意选择。
  //
  // **但 z 不是 gauge 自由度**——只要图里有深度因子，深度传感器就给了每个 keyframe
  // （包括 kf0）一个绝对参考。把 kf0 的 z 钉在 0，同时深度因子又把其余每个 keyframe
  // 往 -measured_depth 拉，这是一个货真价实的约束冲突。
  //
  // 这个问题是**实跑 demo** 才发现的：求解器 stalled、ATE 高达几米，而不是这种噪声
  // 很小的干净合成场景本该有的分米以下误差。修法就是拿 kf0 自己的深度观测来定它的 z，
  // 让这个固定顶点与图的其余部分自洽。
  //
  // 这件事与 warmup_seconds 无关：它是直接给固定顶点设初值，而不是加一个深度因子，
  // 所以不属于预热要拦的那些"融合"。
  //
  // 在 estimator_mode: imu_preintegration 下，anchor 是**第一条 keyframe boundary**
  // 对应的那个 keyframe，而不是字面量 "kf0"——该模式下 keyframe 身份只来自
  // /keyframe/boundary，别无他源。
  //
  // 它的 x/y/yaw 出于同样的 gauge 理由仍取规范零值；roll/pitch 来自静止初始化器测出的
  // 重力方向（重力唯一能观测的就是这两个量）；z 则和下面一样来自它自己的深度证据。
  const bool imu_mode = estimator_mode == "imu_preintegration";
  std::vector<const uw::domain::KeyframeBoundary*> ordered_boundaries;
  std::optional<uw::frontends::ImuStationaryInitialization> inertial_initialization;
  std::string anchor_keyframe_id = "kf0";
  if (imu_mode) {
    if (!imu_rig.has_value()) {
      std::cerr << "imu_preintegration: no rig loaded; this mode needs --experiment with a rig "
                   "carrying imu_noise and a base_link->imu_link edge\n";
      return 1;
    }
    // ReplayInputAccumulator 已经拒掉了重复 id 和非严格递增的 capture_time，所以在
    // 这里 bag 的顺序**就是** boundary 的顺序；这个循环只是把它记下来，不需要再排序。
    for (const auto& boundary : input.keyframe_boundaries) ordered_boundaries.push_back(&boundary);
    if (ordered_boundaries.size() < 2) {
      std::cerr << "imu_preintegration: fewer than two keyframe boundaries on "
                << uw::runtime::kTopicKeyframeBoundary << " (" << ordered_boundaries.size()
                << " found) — this mode has no other source of keyframe identity, and ground "
                   "truth is not a substitute\n";
      return 1;
    }
    anchor_keyframe_id = ordered_boundaries.front()->keyframe_id().value();

    const double window_end_s =
        uw::domain::ToSeconds(ordered_boundaries.front()->header().capture_time());
    inertial_initialization = uw::frontends::InitializeFromStationaryWindow(
        input.imu_samples, window_end_s, *imu_rig);
    if (!inertial_initialization.has_value()) {
      std::cerr << "imu_preintegration: cannot define an initial inertial prior from this bag "
                   "(rig imu_noise unusable, or a malformed IMU reading inside the pre-roll "
                   "window)\n";
      return 1;
    }
    std::cout << "imu stationary initializer: "
              << (inertial_initialization->mode ==
                          uw::frontends::ImuStationaryInitialization::Mode::kStationary
                      ? "stationary"
                      : "wide_velocity_prior")
              << " over " << inertial_initialization->sample_count << " sample(s) / "
              << inertial_initialization->window_duration_s << " s before "
              << anchor_keyframe_id;
    if (!inertial_initialization->detail.empty()) {
      std::cout << " (" << inertial_initialization->detail << ")";
    }
    std::cout << "\n";
  }

  double anchor_z = 0.0;
  for (const auto& evidence : input.evidence) {
    if (anchor_z != 0.0) break;  // 已经找到了
    if (!uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(evidence)) continue;
    if (evidence.source_observations_size() == 0) continue;
    if (evidence.source_observations(0).value() != anchor_keyframe_id) continue;
    // depth_m 是**正向下**的水深量，而世界系是 Z-up，所以取负号才是位姿的 z
    // ——见 schemas/proto/uw/domain/measurement.proto 里 PressureDepthMeasurement
    // 的字段注释。
    anchor_z = -uw::domain::GetPayload<uw::domain::PressureDepthMeasurement>(evidence).depth_m();
  }

  uw::estimation::PoseGraphProblem problem;
  Pose3 anchor_pose = Pose3::Identity();
  anchor_pose.translation.z() = anchor_z;
  if (imu_mode) anchor_pose.rotation = inertial_initialization->rotation_WB;
  problem.AddKeyframe(anchor_keyframe_id, anchor_pose, /*fixed=*/true);

  // sqrt-information 常数，来自 configs/defaults/platform.yaml（经 --experiment），
  // 没给 --experiment 时则用 PlatformDefaultsConfig 内建的回退值。
  // 见文件头的限制说明：它们目前仍是固定常数，不是经过标定的可靠性调度器。
  const double relative_pose_translation_cap = defaults.default_sqrt_information.relative_pose.translation;
  const double relative_pose_rotation_cap = defaults.default_sqrt_information.relative_pose.rotation;
  // FactorCandidate.proposed_noise 在线上只是一个标量（见
  // RelativePoseFactorBuilder::Build() 自己的注释）。这里取两个分类上限中的**较大值**，
  // 保证它不会把其中任何一个收紧到比 builder 构造时已有的值还严。
  const double relative_pose_sqrt_info = std::max(relative_pose_translation_cap, relative_pose_rotation_cap);
  const double sonar_range_sqrt_info = defaults.default_sqrt_information.sonar_range;
  const double depth_sqrt_info = defaults.default_sqrt_information.depth;

  uw::factor_builders::RelativePoseFactorBuilder relative_pose_builder(relative_pose_translation_cap,
                                                                       relative_pose_rotation_cap);
  uw::factor_builders::SonarRangeFactorBuilder sonar_range_builder;
  uw::factor_builders::DepthFactorBuilder depth_builder;

  // 声呐 range 因子做数据关联用的在线路标库（见文件头）。
  //
  // 所有发现的路标都放在同一个名为 "landmarks"、位姿固定为 Identity 的桶里，因为
  // QueryNearestPoint / WorldPointsForKeyframe 存取的本来就是世界系下的点——见
  // include/mapping/submap_manager.hpp。
  //
  // 声明在这里（而不是跟下面的 StateStore 放一起），是因为它在**声呐证据那一趟**就
  // 必须已经活着，不是解完之后才用。
  uw::mapping::SubmapManager submap_manager;
  submap_manager.UpdateKeyframePose("landmarks", Pose3::Identity());
  int next_landmark_id = 0;
  int num_landmarks_discovered = 0;

  // 相机帧只分拣一次，之后由两处共享：下面的相对位姿那段
  // （estimator_mode == "stereo_landmark_vo"）和更下面的声光那一趟。在这次改造之前
  // 它们各读各的。
  //
  // 这里存的是**原始帧**：同步器和声呐前端仍然要消费原始声呐/图像的 header
  // （capture_time、sensor_frame）；只有"已校正 + MONO8 的像素内容"才在下面通过
  // get_rectified() 按需派生。
  //
  // 身份直接取自 header.observation_id（ImageFrame.header，由 synth_bag_gen 的
  // BuildStereoPair / record_session.py 的 _write_keyframe 写入），不做任何基于
  // capture_time 的重建。左右目靠 header.sensor_id（"camera_left"/"camera_right"）
  // 区分，两个生产者用的都是这个约定。
  std::unordered_map<std::string, uw::domain::ImageFrame> left_by_kf_raw, right_by_kf_raw;
  // 相机 keyframe id 按 bag 的 log_time_ns 顺序（McapEventSource）首次出现的次序。
  // 它取代了以前那个 `"kf" + std::to_string(i)`、i 从 0 遍历到 max_kf_index 的循环
  // ——那种写法假设 id 是连续的数字，不成立就会漏掉。
  std::vector<std::string> ordered_camera_keyframe_ids;
  std::unordered_set<std::string> seen_camera_keyframe_ids;

  // 给 StateSnapshot / TUM 输出用的、逐 keyframe 的真实元数据（P1：不再用
  // `index * kKeyframeIntervalS` 编造时间戳）。
  //
  // capture_time_by_keyframe 的优先级，由高到低三档：
  //   1. 原始左相机的 capture_time（在这里填）；
  //   2. /gt/state 自带的 capture_timestamp，按 state_id 匹配（在下面、快照循环之前填）；
  //   3. 最早引用到该 keyframe 的那条相对位姿/深度证据的 MCAP log_time_ns（在对应的
  //      读取趟里填）。
  // 每一档只填补更高档没能填上的 keyframe。
  std::unordered_map<std::string, uw::domain::Stamp> capture_time_by_keyframe;
  std::unordered_map<std::string, std::vector<uw::domain::EvidenceId>> evidence_by_keyframe;

  if (rig.has_value()) {
    for (const auto& frame : input.images) {
      const std::string& kf_id = frame.header().observation_id().value();
      const std::string& sensor_id = frame.header().sensor_id().value();
      if (sensor_id == "camera_left") {
        left_by_kf_raw[kf_id] = frame;
        capture_time_by_keyframe[kf_id] = frame.header().capture_time();
      } else if (sensor_id == "camera_right") {
        right_by_kf_raw[kf_id] = frame;
      } else {
        continue;
      }
      if (seen_camera_keyframe_ids.insert(kf_id).second) ordered_camera_keyframe_ids.push_back(kf_id);
    }
  }

  // 对某个 keyframe 的原始立体像对最多做一次校正（ConvertToMono8 +
  // StereoRectificationContext::Process()）并缓存结果，这样下面的 VO 那一趟和声光
  // 那一趟不会把同一批像素重映射两次。
  //
  // 原始像对缺失，或 ConvertToMono8/Process() 失败（编码不对、尺寸不匹配等）时返回
  // nullptr。注意这是**逐 keyframe 的局部状况**，不像上面 Create() 失败那样是致命错误
  // ——个别帧坏掉不该让整轮停摆。
  std::unordered_map<std::string, uw::opencv_adapters::RectifiedStereoBundle> rectified_by_kf;
  auto get_rectified = [&](const std::string& kf_id) -> uw::opencv_adapters::RectifiedStereoBundle* {
    const auto cached = rectified_by_kf.find(kf_id);
    if (cached != rectified_by_kf.end()) return &cached->second;
    const auto left_it = left_by_kf_raw.find(kf_id);
    const auto right_it = right_by_kf_raw.find(kf_id);
    if (left_it == left_by_kf_raw.end() || right_it == right_by_kf_raw.end()) return nullptr;
    const auto left_mono = uw::domain::ConvertToMono8(left_it->second);
    const auto right_mono = uw::domain::ConvertToMono8(right_it->second);
    if (!left_mono.has_value() || !right_mono.has_value()) return nullptr;
    std::string process_error;
    auto rectified = rectification_context->Process(*left_mono, *right_mono, &process_error);
    if (!rectified.has_value()) return nullptr;
    return &rectified_by_kf.emplace(kf_id, std::move(*rectified)).first->second;
  };

  // 逐 keyframe 记录**它被处理当时**的 VO 健康度。黑箱模式下这个表永远是空的
  // （根本没有本地 VO 前端可问），所以那些快照在调 DecideTrackingStatus 时
  // vo_enabled 必须传 false。
  std::unordered_map<std::string, uw::domain::HealthReport::Status> vo_health_by_keyframe;

  // 补齐 capture_time_by_keyframe 中低于"原始相机 capture_time"（上面已无条件填过）
  // 的那几档：
  //   第 2 档：真值自带的 capture_timestamp，按 state_id 匹配。这是本平台合成/真实
  //            bag 本来就带的真实时间戳字段，这里**只用作时间**、绝不用于位姿——
  //            真值依然永远不会进入估计状态。
  //   第 3 档（兜底）：最早引用到该 keyframe 的深度/相对位姿证据的 MCAP log_time_ns。
  // .emplace() 只在更高档没占过该 keyframe 时才插入，所以"按这个固定顺序跑三趟"本身
  // 就是优先级。
  //
  // 在 estimator_mode: imu_preintegration 下以上全部不适用：/keyframe/boundary 流是
  // keyframe 身份**和**时间的唯一来源，在这里先于其他所有档填好；下面那个真值循环对
  // 算法路径**没有任何贡献**——既不给位姿，也不给时间戳。
  //
  // 强调一下：在这个模式里从真值取时间戳**不是无害的**。时间戳定义了预积分区间，
  // 从 /gt/state 读它就等于把真值直接放进了估计里。
  if (imu_mode) {
    for (const auto* boundary : ordered_boundaries) {
      capture_time_by_keyframe[boundary->keyframe_id().value()] = boundary->header().capture_time();
    }
  }
  std::vector<uw::evaluation::TrajectoryPose> ground_truth_trajectory;
  for (const auto& gt : input.reference_states) {
    if (!imu_mode && gt.has_state_id() && !gt.state_id().value().empty()) {
      capture_time_by_keyframe.emplace(gt.state_id().value(), gt.capture_timestamp());
    }
    ground_truth_trajectory.push_back(
        {uw::domain::ToSeconds(gt.capture_timestamp()), Pose3::FromProto(gt.pose_wb())});
  }
  int num_timestamp_fallback_evidence = 0;
  for (std::size_t i = 0; i < input.evidence.size() && !imu_mode; ++i) {
    const auto& evidence = input.evidence[i];
    if (!uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(evidence)) continue;
    if (evidence.source_observations_size() == 0) continue;
    const auto log_time_ns = accumulator.EvidenceLogTimeNs()[i];
    const auto [it, inserted] = capture_time_by_keyframe.emplace(
        evidence.source_observations(0).value(),
        uw::domain::FromSeconds(static_cast<double>(log_time_ns) * 1e-9));
    if (inserted) ++num_timestamp_fallback_evidence;
  }
  for (std::size_t i = 0; i < input.evidence.size() && !imu_mode; ++i) {
    const auto& evidence = input.evidence[i];
    if (!uw::domain::HasPayload<uw::domain::RelativePoseMeasurement>(evidence)) continue;
    const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(evidence);
    const auto log_time_ns = accumulator.EvidenceLogTimeNs()[i];
    const auto [it, inserted] = capture_time_by_keyframe.emplace(
        measurement.to_keyframe().value(), uw::domain::FromSeconds(static_cast<double>(log_time_ns) * 1e-9));
    if (inserted) ++num_timestamp_fallback_evidence;
  }
  if (num_timestamp_fallback_evidence > 0) {
    std::cout << "warning: " << num_timestamp_fallback_evidence
              << " keyframe(s) had no camera/GT timestamp; used MCAP log_time_ns as a fallback\n";
  }

  int num_relative_pose_factors = 0;
  int num_imu_factors = 0;
  int num_imu_intervals_rejected = 0;
  if (imu_mode) {
    // 惯性航位推算。这个分支**不读** /gt/state，也不读 RelativePoseMeasurement：
    // anchor 之后的每个 keyframe，其位姿**和**速度都纯粹由"上一个**估计**状态"经本
    // 区间的预积分增量传播而来——这是第一阶段这套传感器配置能合法给出的唯一初值。
    //
    // 增量的定义见 sensor_models/imu_preintegration.hpp，下面是把它反解成前向传播的
    // 形式（取 g_W = (0, 0, -gravity)）：
    //   R_j = R_i dR        v_j = v_i + g dt + R_i dv
    //   p_j = p_i + v_i dt + 0.5 g dt^2 + R_i dp
    uw::frontends::ImuPreintegrationFrontendParams imu_params;
    uw::frontends::ImuPreintegrationFrontend imu_frontend(imu_params);
    uw::factor_builders::ImuPreintegrationFactorBuilder imu_builder;
    const double gravity_mps2 = imu_rig->imu_noise().gravity_mps2() > 0.0
                                    ? imu_rig->imu_noise().gravity_mps2()
                                    : 9.80665;
    const Eigen::Vector3d gravity_W(0.0, 0.0, -gravity_mps2);

    // anchor 的**位姿**是固定的（gauge），但它的**惯性块不是**：速度和两个零偏都是
    // 求解器应当去精化的量。
    //
    // 约束住它们的，是下面加的那一条先验残差。没有这条先验，这个块在结构上就是无约束
    // 的，正规方程会奇异。
    uw::estimation::PoseGraphProblem::InertialState anchor_inertial;
    anchor_inertial.velocity_W = inertial_initialization->velocity_W;
    anchor_inertial.bias_gyro = inertial_initialization->bias_gyro;
    anchor_inertial.bias_accel = inertial_initialization->bias_accel;
    problem.AddInertialState(anchor_keyframe_id, anchor_inertial);

    Eigen::Matrix<double, 9, 1> prior_target;
    prior_target.segment<3>(0) = anchor_inertial.velocity_W;
    prior_target.segment<3>(3) = anchor_inertial.bias_gyro;
    prior_target.segment<3>(6) = anchor_inertial.bias_accel;
    auto prior_block = uw::factor_builders::InertialPriorResidual::Create(
        prior_target, inertial_initialization->sigma);
    if (!prior_block) {
      std::cerr << "imu_preintegration: the stationary initializer produced a prior sigma that "
                   "cannot define a residual\n";
      return 1;
    }
    problem.AddResidualBlockOnParameters(
        std::move(prior_block),
        {uw::estimation::PoseGraphProblem::InertialRef(anchor_keyframe_id)});

    // 排序只做一次，之后按区间切片。
    //
    // 每次都把整包样本丢给前端是**平方复杂度**：它每次调用都要重新过滤、重新转换、
    // 重新排序全部样本。一段 5 分钟、200 Hz 的录制配 5 Hz keyframe，就是对 6 万个样本
    // 做约 1500 次完整排序。合成用例（541 个样本、11 个区间）完全暴露不出这个问题。
    //
    // 切片的起点取"最后一个 <= from_time 的样本"——也就是前端在区间起始时刻手上持有的
    // 那个读数；终点取 to_time。这正好是它会看的全部内容。
    std::vector<uw::domain::ImuSample> sorted_imu_samples = input.imu_samples;
    std::stable_sort(sorted_imu_samples.begin(), sorted_imu_samples.end(),
                     [](const uw::domain::ImuSample& a, const uw::domain::ImuSample& b) {
                       return uw::domain::ToSeconds(a.header().capture_time()) <
                              uw::domain::ToSeconds(b.header().capture_time());
                     });
    std::vector<double> sorted_imu_times;
    sorted_imu_times.reserve(sorted_imu_samples.size());
    for (const auto& sample : sorted_imu_samples) {
      sorted_imu_times.push_back(uw::domain::ToSeconds(sample.header().capture_time()));
    }
    auto imu_window_for = [&](double from_time_s,
                              double to_time_s) -> std::vector<uw::domain::ImuSample> {
      const auto first_after_from =
          std::upper_bound(sorted_imu_times.begin(), sorted_imu_times.end(), from_time_s);
      const std::size_t begin =
          first_after_from == sorted_imu_times.begin()
              ? 0
              : static_cast<std::size_t>(first_after_from - sorted_imu_times.begin()) - 1;
      const auto first_after_to =
          std::upper_bound(sorted_imu_times.begin(), sorted_imu_times.end(), to_time_s);
      const std::size_t end = static_cast<std::size_t>(first_after_to - sorted_imu_times.begin());
      if (end <= begin) return {};
      return std::vector<uw::domain::ImuSample>(sorted_imu_samples.begin() + begin,
                                                 sorted_imu_samples.begin() + end);
    };

    for (std::size_t i = 1; i < ordered_boundaries.size(); ++i) {
      const std::string from = ordered_boundaries[i - 1]->keyframe_id().value();
      const std::string to = ordered_boundaries[i]->keyframe_id().value();
      if (!problem.HasKeyframe(from)) {
        ++num_imu_intervals_rejected;
        continue;  // 上一个区间被拒了，不要跨过它硬接
      }

      uw::measurement_api::ImuPreintegrationRequest request;
      request.from_keyframe_id = from;
      request.to_keyframe_id = to;
      request.from_time = ordered_boundaries[i - 1]->header().capture_time();
      request.to_time = ordered_boundaries[i]->header().capture_time();
      // 在图当前为 `from` 保存的那个状态处做线性化：对 anchor 来说就是初始化器给的
      // 估计，之后各帧则是传播过来的那个。
      const auto from_inertial = problem.GetInertialState(from);
      request.bias_gyro = from_inertial.bias_gyro;
      request.bias_accel = from_inertial.bias_accel;

      const auto evidence = imu_frontend.Process(
          imu_window_for(uw::domain::ToSeconds(request.from_time),
                         uw::domain::ToSeconds(request.to_time)),
          request, *imu_rig);
      if (!evidence.has_value()) {
        ++num_imu_intervals_rejected;
        std::cout << "imu interval " << from << " -> " << to
                  << " rejected: " << imu_frontend.last_rejection_reason() << "\n";
        continue;
      }
      std::string delta_error;
      const auto delta = uw::sensor_models::PreintegratedImuDelta::FromProto(
          evidence->imu_preintegration(), &delta_error);
      if (!delta.has_value()) {
        ++num_imu_intervals_rejected;
        std::cout << "imu interval " << from << " -> " << to
                  << " rejected: unreadable delta (" << delta_error << ")\n";
        continue;
      }

      const Pose3 from_pose = problem.GetKeyframePose(from);
      const Eigen::Matrix3d R_i = from_pose.rotation.toRotationMatrix();
      const double dt = delta->delta_time_s;
      Pose3 to_pose;
      to_pose.rotation = Eigen::Quaterniond(R_i * delta->delta_rotation).normalized();
      to_pose.translation = from_pose.translation + from_inertial.velocity_W * dt +
                            0.5 * gravity_W * dt * dt + R_i * delta->delta_position;
      uw::estimation::PoseGraphProblem::InertialState to_inertial;
      to_inertial.velocity_W =
          from_inertial.velocity_W + gravity_W * dt + R_i * delta->delta_velocity;
      // 零偏按随机游走建模，所以下一状态的传播初值就直接取当前值；真正让求解器把
      // 两者拉开的，是 IMU 残差里的零偏那几行。
      to_inertial.bias_gyro = from_inertial.bias_gyro;
      to_inertial.bias_accel = from_inertial.bias_accel;

      uw::domain::FactorCandidate candidate;
      candidate.set_residual_model(
          uw::factor_builders::ImuPreintegrationFactorBuilder::kResidualModel);
      candidate.set_proposed_noise(1.0);
      auto block = imu_builder.Build(candidate, *evidence, {});
      if (!block) {
        ++num_imu_intervals_rejected;
        std::cout << "imu interval " << from << " -> " << to
                  << " rejected: factor builder refused the evidence\n";
        continue;
      }

      // 到这里边一定建得成了才注册顶点。
      //
      // 如果反过来先加 keyframe、后面再中途 bail，`to` 就会以"一个自由的 9 维惯性块 +
      // 没有任何 IMU 边"的形态留在图里。这有两重危害：一是让上面那道"上一个区间活下来
      // 了吗"的防线失效（下一轮迭代会发现 `to` 已存在，于是直接跨过缺口硬接）；二是
      // 如果 `to` 恰好也没有深度/声呐因子，一个本来可恢复的区间拒绝就变成了硬性的
      // 结构失败。
      problem.AddKeyframe(to, to_pose);
      problem.AddInertialState(to, to_inertial);
      problem.AddResidualBlockOnParameters(
          std::move(block), {uw::estimation::PoseGraphProblem::PoseRef(from),
                             uw::estimation::PoseGraphProblem::InertialRef(from),
                             uw::estimation::PoseGraphProblem::PoseRef(to),
                             uw::estimation::PoseGraphProblem::InertialRef(to)});
      evidence_by_keyframe[to].push_back(evidence->evidence_id());
      ++num_imu_factors;
    }
    std::cout << "added " << num_imu_factors << " IMU preintegration factors over "
              << ordered_boundaries.size() << " keyframe boundaries ("
              << num_imu_intervals_rejected << " interval(s) rejected)\n";
    // 一个区间被拒，损失的不只是一条边：这套传感器配置里没有别的相对运动来源，
    // 所以 `to` 以及它之后的一切都进不了图。
    //
    // 那样这轮就会写出一条**更短**的轨迹，并只在这个前缀上算 ATE——而前缀的 ATE 往往
    // 比整轮**更好看**，不是更差。所以这里 fail-closed，而不是发布一个悄悄描述了另一
    // 条轨迹的数字。
    if (problem.NumKeyframes() != ordered_boundaries.size()) {
      std::cerr << "imu_preintegration: " << problem.NumKeyframes() << " of "
                << ordered_boundaries.size()
                << " keyframe boundaries made it into the graph — " << num_imu_intervals_rejected
                << " interval(s) were rejected, so the trajectory is truncated and any ATE over "
                   "it would describe a different run\n";
      return 1;
    }
  } else if (rig.has_value() && estimator_mode == "stereo_landmark_vo") {
    // 从双目相机帧实时算出来的**真实**相对位姿证据
    // （include/frontends/stereo_landmark_vo_frontend.hpp），取代下面那条从
    // /evidence/relative_pose 读 synth_bag_gen "真值+噪声"黑箱 VIO 替身的路径。
    //
    // keyframe 必须**按顺序**访问：这个前端跨调用是有状态的，它拿本次的路标和上一次的
    // 比。所以这里按 kf0..kfN 的顺序迭代，而不是依赖 bag 的流顺序。
    uw::frontends::StereoLandmarkVoFrontendParams vo_params;
    vo_params.left_frame = rectification_context->LeftRectifiedFrame();
    vo_params.right_frame = rectification_context->RightRectifiedFrame();
    vo_params.max_consecutive_failures = defaults.visual_odometry.max_consecutive_failures;
    vo_params.covariance_estimation.max_condition_number = defaults.visual_odometry.max_condition_number;
    vo_params.covariance_estimation.residual_variance_floor_m2 =
        defaults.visual_odometry.residual_variance_floor_m2;
    vo_params.covariance_estimation.max_inlier_rmse_m = defaults.visual_odometry.max_inlier_rmse_m;
    vo_params.detector_kind = landmark_detector == "harris_corner"
                                   ? uw::frontends::LandmarkDetectorKind::kHarrisCorner
                                   : uw::frontends::LandmarkDetectorKind::kBrightBlob;
    if (landmark_detector == "harris_corner") {
      // 这几项只对**真实相机**路径生效。synth_bag_gen 的路标是刻意做成逐 id 唯一的
      // 哈希图案（见它的头注释），所以 PatchMatcher 那套朴素贪心 NCC 在合成数据上从来
      // 没有歧义问题，跟踪中的合成 ATE 数字也不该因此变化。
      //
      // 真实水下影像（珊瑚/岩石/沙地）在局部是高度重复的——这是拿 apps/replay_demo
      // 跑真实 HoloOcean 录制时吃过亏才确认的：纯外观匹配、不加任何空间/独特性约束，
      // 结果 ATE=587m（求解器 stalled）。根因是一旦错误对应和正确对应一样多，RANSAC
      // 赖以成立的"多数共识"假设就崩了。
      //
      // 两个约束各管一头：
      //   - max_row_diff_px 对应 BlockMatcher 自己的同行视差搜索（对这个平行双目 rig
      //     而言就是极线约束）；
      //   - min_score_margin 要求一个匹配必须明显优于任一侧的次优候选才被接受
      //     （Lowe ratio test 那种独特性检查）。两者都见 patch_matcher.hpp。
      //
      // 这些阈值是**针对手上仅有的那一份真实 bag 的首次经验取值**，不是标定常数。
      vo_params.stereo_matcher.max_row_diff_px = 4.0;
      vo_params.stereo_matcher.min_score_margin = 0.02;
      vo_params.temporal_matcher.min_score_margin = 0.05;
    }
    uw::frontends::StereoLandmarkVoFrontend vo_frontend(vo_params);

    for (const auto& kf_id : ordered_camera_keyframe_ids) {
      auto* rectified = get_rectified(kf_id);
      if (rectified == nullptr) continue;

      uw::measurement_api::CameraFrameBundle bundle = rectified->images;
      bundle.primary.mutable_header()->mutable_observation_id()->set_value(kf_id);

      const auto vo_evidence = vo_frontend.Process(bundle, rectification_context->DerivedRig());
      // **每一帧**处理完都记录，无论成功失败，键是刚处理完的那一帧。
      // 这正是 DecideTrackingStatus 里"逐快照 VO 健康度不可追溯改写"的实现基础：
      // 在 VO 还健康时处理的那个 keyframe，即使 VO 后来退化了，也必须保持 TRACKING。
      vo_health_by_keyframe[kf_id] = vo_frontend.Health().status();
      if (!vo_evidence.has_value()) continue;  // 这是见到的第一帧，或者这次转移拟合不出来

      const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(*vo_evidence);
      const std::string from = measurement.from_keyframe().value();
      const std::string to = measurement.to_keyframe().value();
      if (!problem.HasKeyframe(from)) continue;  // 乱序或意料之外的输入：跳过，不去猜

      const auto measured_relative = Pose3::FromProto(measurement.relative_pose());
      const auto dead_reckoned_initial_guess = problem.GetKeyframePose(from) * measured_relative;
      problem.AddKeyframe(to, dead_reckoned_initial_guess);

      uw::domain::FactorCandidate candidate;
      candidate.set_residual_model(uw::factor_builders::RelativePoseFactorBuilder::kResidualModel);
      candidate.set_proposed_noise(relative_pose_sqrt_info);
      auto block = relative_pose_builder.Build(candidate, *vo_evidence, {});
      if (block) {
        problem.AddResidualBlock(std::move(block), {from, to});
        evidence_by_keyframe[to].push_back(vo_evidence->evidence_id());
        ++num_relative_pose_factors;
      }
    }
    std::cout << "stereo_landmark_vo_frontend: computed relative-pose evidence from camera frames "
                 "(estimator_mode=stereo_landmark_vo)\n";
  } else {
    for (const auto& evidence : input.evidence) {
      if (!uw::domain::HasPayload<uw::domain::RelativePoseMeasurement>(evidence)) continue;
      const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(evidence);
      const std::string from = measurement.from_keyframe().value();
      const std::string to = measurement.to_keyframe().value();
      if (!problem.HasKeyframe(from)) continue;  // 乱序或意料之外的输入：跳过，不去猜

      const auto measured_relative = Pose3::FromProto(measurement.relative_pose());
      const auto dead_reckoned_initial_guess = problem.GetKeyframePose(from) * measured_relative;
      problem.AddKeyframe(to, dead_reckoned_initial_guess);

      uw::domain::FactorCandidate candidate;
      candidate.set_residual_model(uw::factor_builders::RelativePoseFactorBuilder::kResidualModel);
      candidate.set_proposed_noise(relative_pose_sqrt_info);
      auto block = relative_pose_builder.Build(candidate, evidence, {});
      if (block) {
        problem.AddResidualBlock(std::move(block), {from, to});
        evidence_by_keyframe[to].push_back(evidence.evidence_id());
        ++num_relative_pose_factors;
      }
    }
  }
  std::cout << "added " << num_relative_pose_factors << " relative-pose factors, "
            << problem.NumKeyframes() << " keyframes\n";

  // 回环闭合的位姿图边（架构上受 SVIn 的 pose_graph 模块启发——具体移植了什么、
  // 没移植什么，见 frontends/loop_closure_frontend.hpp 自己的头注释）。
  //
  // 这是**独立的第二趟**，跑在上面 stereo_landmark_vo 那个循环把每个 keyframe 都
  // 航位推算进 `problem` **之后**，而不是与之交织。这样"一次推进一条边的航位推算"和
  // "在迄今为止的整个档案里检索"就是两件互不干扰的事，而且档案里永远不会出现一个
  // 尚未被航位推算过的 keyframe。
  //
  // 开启条件：与 VO **相同**的 rig/estimator_mode 前提（回环闭合需要真实相机帧），
  // 外加 defaults.loop_closure.enabled（默认关——除非配置显式打开，否则对任何既有
  // experiment 的行为都是零改变）。
  int num_loop_closure_factors = 0;
  if (rig.has_value() && estimator_mode == "stereo_landmark_vo" && defaults.loop_closure.enabled) {
    uw::frontends::LoopClosureFrontendParams lc_params;
    lc_params.left_frame = rectification_context->LeftRectifiedFrame();
    lc_params.right_frame = rectification_context->RightRectifiedFrame();
    lc_params.candidate_search_radius_m = defaults.loop_closure.candidate_search_radius_m;
    lc_params.min_keyframe_index_gap = defaults.loop_closure.min_keyframe_index_gap;
    lc_params.max_accepted_translation_m = defaults.loop_closure.max_accepted_translation_m;
    lc_params.max_accepted_rotation_rad = defaults.loop_closure.max_accepted_rotation_rad;
    lc_params.min_landmarks_for_pose = defaults.loop_closure.min_landmarks_for_pose;
    lc_params.max_loop_edges_per_keyframe = defaults.loop_closure.max_loop_edges_per_keyframe;
    uw::frontends::LoopClosureFrontend loop_frontend(lc_params);

    for (const auto& kf_id : ordered_camera_keyframe_ids) {
      if (!problem.HasKeyframe(kf_id)) continue;  // VO 没把它推算进来过（比如第一帧）
      auto* rectified = get_rectified(kf_id);
      if (rectified == nullptr) continue;

      uw::measurement_api::CameraFrameBundle bundle = rectified->images;
      bundle.primary.mutable_header()->mutable_observation_id()->set_value(kf_id);

      // 此刻 problem.GetKeyframePose(kf_id) 拿到的仍是**求解之前**的航位推算估计
      // （solver.Solve() 还没跑）——这恰好就是位姿邻近检索所需要的"插入时刻的位置"。
      const auto loop_evidences =
          loop_frontend.Process(bundle, rectification_context->DerivedRig(), kf_id, problem.GetKeyframePose(kf_id));
      for (const auto& loop_evidence : loop_evidences) {
        const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(loop_evidence);
        const std::string from = measurement.from_keyframe().value();
        const std::string to = measurement.to_keyframe().value();
        if (!problem.HasKeyframe(from) || !problem.HasKeyframe(to)) continue;

        uw::domain::FactorCandidate candidate;
        candidate.set_residual_model(uw::factor_builders::RelativePoseFactorBuilder::kResidualModel);
        candidate.set_proposed_noise(relative_pose_sqrt_info);
        candidate.set_robust_policy_hint(uw::domain::ROBUST_POLICY_HUBER);
        auto block = relative_pose_builder.Build(candidate, loop_evidence, {});
        if (block) {
          problem.AddResidualBlock(std::move(block), {from, to},
                                   uw::estimation::PoseGraphProblem::RobustPolicy::kHuber);
          evidence_by_keyframe[to].push_back(loop_evidence.evidence_id());
          ++num_loop_closure_factors;
        }
      }
    }
    std::cout << "loop_closure_frontend: added " << num_loop_closure_factors << " loop-closure factors\n";
  }

  const auto cfar_params =
      uw::application::BuildSonarCfarFrontendParams(defaults.sonar_frontend);
  uw::frontends::SonarCfarFrontend sonar_frontend(cfar_params);

  int num_sonar_frames = 0;
  int num_sonar_factors = 0;
  // 逐声呐帧的处理延迟统计，做法照搬 acoustic_optic_scenario_matrix.cpp 的逐次试验
  // 计时模式。
  //
  // 为什么选这个循环：上面 stereo_landmark_vo 那个循环只在 VO 模式下跑，而这份多趟
  // 批处理文件里别的地方也没有单点可选；只有这一趟是"无论 estimator_mode 取什么、
  // 每个 experiment 配置都会跑"的、真正逐 keyframe 的 前端+数据关联+建因子 过程，
  // 所以它是放第一个延迟代理指标最自然的位置。
  //
  // **这只是打底，不是真实的在线延迟**：replay_demo 是分几趟批处理整包数据，所以这里
  // 量到的是批处理内部的逐帧 CPU 开销，而不是实时系统里"从采集到出位姿"的延迟——后者
  // 需要一个在线调度器才谈得上。
  std::vector<double> sonar_frame_latencies_ms;
  for (const auto& frame : input.sonar_frames) {
    // 用 RAII，是为了让下面**每一条**退出路径都被计时（包括预热跳过、未知 keyframe、
    // 无检测这几个提前 continue），而不是只计完整处理那条路——在在线系统里，做出一个
    // "拒绝"的决定同样是要花真实时间的。
    struct LatencyRecorder {
      std::vector<double>& out;
      std::chrono::steady_clock::time_point start;
      ~LatencyRecorder() {
        out.push_back(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
      }
    } latency_recorder{sonar_frame_latencies_ms, std::chrono::steady_clock::now()};

    ++num_sonar_frames;
    const std::string keyframe_id = frame.header().observation_id().value();
    if (!problem.HasKeyframe(keyframe_id)) continue;
    if (warmup_keyframe_ids.count(keyframe_id)) continue;  // 预热期只做航位推算

    // v1 规则（schemas/proto/uw/domain/hypothesis.proto，另见文件头）：只消费排名
    // 第一（聚类最大）的那个候选。
    //
    // candidates(0) 本身就是一条完整的 MeasurementEvidence，不只是 payload——所以这里
    // 原样复用，而不是用 domain::TopCandidate<T>() 再包一层：那样会白白丢掉
    // evidence_id / algorithm_version，在这里没有任何好处。
    const auto hypothesis_set = sonar_frontend.ProcessSonarFrame(frame);
    if (hypothesis_set.candidates_size() == 0) continue;  // 没有超过阈值的检测
    const auto& evidence = hypothesis_set.candidates(0);
    if (!uw::domain::HasPayload<uw::domain::SonarRangeBearing>(evidence)) continue;
    const auto& measurement = uw::domain::GetPayload<uw::domain::SonarRangeBearing>(evidence);

    // 真实的 submap_manager 查询（见文件头）：先用该 keyframe **当前**的位姿估计
    // （航位推算值，尚未优化）把检测投到世界系，再问 submap 在 kLandmarkGateM 半径内
    // 有没有已知路标。
    //
    // 命中：复用那个路标存着的位置——它在多次观测间是稳定的，不会被这一次噪声更大的
    // 单次检测带偏。
    // 未命中：说明这是个从没见过的路标，插入它，好让之后的观测能匹配上。
    const auto current_pose = problem.GetKeyframePose(keyframe_id);
    const Eigen::Vector3d local_detection(measurement.range_m() * std::cos(measurement.bearing_rad()),
                                           measurement.range_m() * std::sin(measurement.bearing_rad()),
                                           0.0);  // 声呐观测不到 elevation——见文件头
    const Eigen::Vector3d predicted_point_W = current_pose.Apply(local_detection);

    constexpr double kLandmarkGateM = 1.5;  // 留出的余量，大于这个量程下预期的航位推算漂移
    const auto existing = submap_manager.QueryNearestPoint(predicted_point_W, kLandmarkGateM);
    Eigen::Vector3d landmark_W;
    if (existing.has_value()) {
      landmark_W = *existing;
    } else {
      landmark_W = predicted_point_W;
      uw::domain::MapEvidence landmark_evidence;
      landmark_evidence.mutable_evidence_id()->set_value("landmark_" + std::to_string(next_landmark_id++));
      landmark_evidence.mutable_keyframe_id()->set_value("landmarks");
      landmark_evidence.mutable_local_frame()->set_value("world");
      landmark_evidence.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
      landmark_evidence.set_reintegration_policy(
          uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);
      std::string bytes(3 * sizeof(float), '\0');
      auto* raw = reinterpret_cast<float*>(bytes.data());
      raw[0] = static_cast<float>(landmark_W.x());
      raw[1] = static_cast<float>(landmark_W.y());
      raw[2] = static_cast<float>(landmark_W.z());
      landmark_evidence.set_geometry_or_occupancy(bytes);
      submap_manager.AddMapEvidence(landmark_evidence);
      ++num_landmarks_discovered;
    }

    uw::domain::FactorCandidate candidate;
    candidate.set_residual_model(uw::factor_builders::SonarRangeFactorBuilder::kResidualModel);
    candidate.set_proposed_noise(sonar_range_sqrt_info);
    uw::measurement_api::FactorBuildContext context;
    context.nearby_points_W = {landmark_W};
    auto block = sonar_range_builder.Build(candidate, evidence, context);
    if (block) {
      problem.AddResidualBlock(std::move(block), {keyframe_id});
      evidence_by_keyframe[keyframe_id].push_back(evidence.evidence_id());
      ++num_sonar_factors;
    }
  }
  std::cout << "discovered " << num_landmarks_discovered << " landmarks via submap query\n";
  std::cout << "processed " << num_sonar_frames << " sonar frames (sonar_cfar_frontend) -> "
            << num_sonar_factors << " sonar range factors\n";
  // P95 的算法与 acoustic_optic_scenario_matrix.cpp 保持一致（最近秩，不做插值）
  // ——这个指标的适用范围见上面那段注释。
  std::sort(sonar_frame_latencies_ms.begin(), sonar_frame_latencies_ms.end());
  const double p95_sonar_frame_latency_ms =
      sonar_frame_latencies_ms.empty()
          ? 0.0
          : sonar_frame_latencies_ms[static_cast<std::size_t>(0.95 * (sonar_frame_latencies_ms.size() - 1))];
  std::cout << "sonar frame processing latency: p95_ms=" << p95_sonar_frame_latency_ms << "\n";

  int num_depth_factors = 0;
  for (const auto& evidence : input.evidence) {
    if (!uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(evidence)) continue;
    if (evidence.source_observations_size() == 0) continue;
    const std::string keyframe_id = evidence.source_observations(0).value();
    if (!problem.HasKeyframe(keyframe_id)) continue;
    if (warmup_keyframe_ids.count(keyframe_id)) continue;  // 预热期只做航位推算

    uw::domain::FactorCandidate candidate;
    candidate.set_residual_model(uw::factor_builders::DepthFactorBuilder::kResidualModel);
    candidate.set_proposed_noise(depth_sqrt_info);
    auto block = depth_builder.Build(candidate, evidence, {});
    if (block) {
      problem.AddResidualBlock(std::move(block), {keyframe_id});
      evidence_by_keyframe[keyframe_id].push_back(evidence.evidence_id());
      ++num_depth_factors;
    }
  }
  std::cout << "added " << num_depth_factors << " depth factors\n";

  // defaults.solver 取 "gauss_newton_v1"（默认）或 "ceres_v1"——见
  // docs/archive/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md §7。
  // 上面的 ValidateExperimentConfigSelections 已经校验过只能是这两者之一；如果在
  // 没开 UW_BUILD_CERES_SOLVER 编译的二进制里选了 "ceres_v1"，这里会是致命的启动错误，
  // **不会**静默回退到 gauss_newton_v1（见那份文档 §8 的错误表）。
  //
  // 结构可观测性检查放在**求解之前**（PREP-B-01）：一个没有任何残差碰到的自由参数块
  // 就是一列奇异，而 LM 的阻尼会给它返回一个看着合理、实则任意的值而不报错。
  //
  // 在 estimator_mode: imu_preintegration 下，这正是设计说明要求的那道防线——每一个
  // 惯性状态都必须能被某条 IMU 边或 anchor 先验触及。但这条性质与模式无关，所以每轮
  // 都查。
  const auto observability = uw::application::CheckGraphObservability(problem);
  std::cout << "graph: " << problem.NumKeyframes() << " keyframes, "
            << problem.NumInertialStates() << " inertial states, "
            << problem.NumResidualBlocks() << " residual blocks, "
            << observability.free_parameter_dim << " free parameter dims, "
            << observability.residual_dim << " residual dims\n";
  if (!observability.problems.empty()) {
    std::cerr << "STRUCTURALLY SINGULAR GRAPH:\n";
    for (const auto& problem_text : observability.problems) {
      std::cerr << "  - " << problem_text << "\n";
    }
    return 1;
  }

  uw::estimation::GaussNewtonSummary summary;
  if (defaults.solver == "ceres_v1") {
#ifdef UW_HAVE_CERES_SOLVER
    uw::adapters::ceres_solver::CeresPoseGraphSolver ceres_solver;
    uw::adapters::ceres_solver::CeresSolverOptions ceres_options;
    ceres_options.max_iterations = defaults.max_iterations;
    summary = ceres_solver.Solve(problem, ceres_options);
#else
    std::cerr << "solver: ceres_v1 requested but this binary was not built with "
                 "UW_BUILD_CERES_SOLVER=ON\n";
    return 1;
#endif
  } else {
    uw::estimation::GaussNewtonSolver solver;
    uw::estimation::GaussNewtonSolver::Options solver_options;
    solver_options.max_iterations = defaults.max_iterations;
    solver_options.initial_lambda = defaults.initial_lambda;
    // 只影响被标记为 RobustPolicy::kHuber 的绑定（即上面的回环闭合边）；一条都没有
    // 时它就是个无害的空设置。
    solver_options.huber_delta = defaults.loop_closure.huber_delta;
    summary = solver.Solve(problem, solver_options);
  }
  std::cout << "solver(" << defaults.solver << "): " << summary.iterations << " iterations, cost "
            << summary.initial_cost << " -> " << summary.final_cost
            << (summary.converged ? " (converged)" : " (stalled)") << "\n";

  // 接上 StateStore（submap_manager 在前面就声明了——从上面声呐那一趟开始，它就一直
  // 在累积路标证据了）。
  uw::estimation::StateStore state_store;
  std::vector<uw::evaluation::TrajectoryPose> estimated_trajectory;

  const std::string calibration_version_for_snapshot =
      rectification_context.has_value() ? rectification_context->DerivedRig().calibration_version().value()
                                        : "";
  const bool vo_enabled_for_tracking = rig.has_value() && estimator_mode == "stereo_landmark_vo";

  int num_snapshot_timestamp_missing = 0;
  for (int i = 0; i < static_cast<int>(problem.KeyframeOrder().size()); ++i) {
    const std::string& kf_id = problem.KeyframeOrder()[i];
    const auto pose = problem.GetKeyframePose(kf_id);

    uw::application::ReplayTrackingInputs tracking_inputs;
    tracking_inputs.solver_converged = summary.converged;
    tracking_inputs.vo_enabled = vo_enabled_for_tracking;
    const auto vo_health_it = vo_health_by_keyframe.find(kf_id);
    if (vo_health_it != vo_health_by_keyframe.end()) tracking_inputs.vo_health = vo_health_it->second;

    uw::application::StateSnapshotInputs snapshot_inputs;
    snapshot_inputs.state_id = kf_id;
    snapshot_inputs.state_version = static_cast<uint64_t>(i) + 1;
    snapshot_inputs.pose = pose;
    const auto capture_time_it = capture_time_by_keyframe.find(kf_id);
    if (capture_time_it != capture_time_by_keyframe.end()) {
      snapshot_inputs.capture_timestamp = capture_time_it->second;
    } else {
      ++num_snapshot_timestamp_missing;  // 相机/真值/证据三档时间都没有：取 Stamp{} 默认值（epoch）
    }
    snapshot_inputs.calibration_version = calibration_version_for_snapshot;
    const auto evidence_it = evidence_by_keyframe.find(kf_id);
    if (evidence_it != evidence_by_keyframe.end()) snapshot_inputs.contributing_evidence = evidence_it->second;
    snapshot_inputs.tracking_status = uw::application::DecideTrackingStatus(tracking_inputs);

    const auto snapshot = uw::application::BuildStateSnapshot(snapshot_inputs);
    state_store.Commit(snapshot);

    submap_manager.UpdateKeyframePose(kf_id, pose);
    estimated_trajectory.push_back({uw::domain::ToSeconds(snapshot.capture_timestamp()), pose});
  }
  if (num_snapshot_timestamp_missing > 0) {
    std::cout << "warning: " << num_snapshot_timestamp_missing
              << " keyframe(s) had no camera/GT/evidence timestamp at all; capture_timestamp defaults to "
                 "epoch\n";
  }

  // 声光那一趟（只在 --experiment 加载了带相机的 rig 时才跑）：逐 keyframe 依次走
  // StereoOpticalDepthFrontend -> SonarCfarFrontend（复用上面声呐那趟的**同一个实例**）
  // -> AcousticOpticDepthFusionFrontend::Fuse -> BuildMapEvidenceFromFusedDepth，
  // 把**第三个** MapEvidence 桶（按 keyframe 分键，与已有的 "landmarks" 桶并列）存进
  // submap_manager，用的是上面**已经提交**的那个位姿。
  //
  // 这一趟**完全不碰** PoseGraphProblem / 求解器 / ATE——稠密深度不会变成一种新的因子
  // 类型。它只影响地图，不影响轨迹估计。
  int num_keyframes_with_camera = 0;
  int num_acoustic_optic_accepted = 0;
  int num_acoustic_optic_ambiguous = 0;
  int num_acoustic_optic_conflict = 0;
  int num_acoustic_optic_rejected = 0;
  int num_sync_invalid_timestamp = 0;
  int num_map_evidence_points = 0;
  MapContributionCounts total_contributions;
  if (rig.has_value()) {
    // left_by_kf_raw / right_by_kf_raw / get_rectified 在上面就定义好了，与相对位姿
    // 那段的 stereo_landmark_vo_frontend 路径共用。
    std::unordered_map<std::string, uw::domain::SonarFrame> sonar_by_kf;
    for (const auto& f : input.sonar_frames) {
      // v1 的 top-1 规则（见文件头）：若量程内有多个目标，每个 keyframe 只保留见到的
      // 第一帧声呐。
      const std::string kf_id = f.header().observation_id().value();
      if (sonar_by_kf.find(kf_id) == sonar_by_kf.end()) sonar_by_kf[kf_id] = f;
    }

    uw::frontends::StereoOpticalDepthFrontendParams stereo_params;
    stereo_params.left_frame = rectification_context->LeftRectifiedFrame();
    stereo_params.right_frame = rectification_context->RightRectifiedFrame();
    stereo_params.matcher.min_texture_variance = defaults.stereo_matching.min_texture_variance;
    stereo_params.matcher.min_uniqueness_margin = defaults.stereo_matching.min_uniqueness_margin;
    stereo_params.matcher.left_right_max_diff_px = defaults.stereo_matching.left_right_max_diff_px;
    uw::frontends::StereoOpticalDepthFrontend stereo_frontend(stereo_params);
    uw::frontends::AcousticOpticDepthFusionParams fusion_params;
    uw::frontends::AcousticOpticDepthFusionFrontend fusion_frontend(fusion_params);
    uw::mapping::AcousticOpticMapBridgeParams bridge_params;

    for (std::size_t kf_index = 0; kf_index < problem.KeyframeOrder().size(); ++kf_index) {
      const std::string& kf_id = problem.KeyframeOrder()[kf_index];
      auto left_raw_it = left_by_kf_raw.find(kf_id);
      auto right_raw_it = right_by_kf_raw.find(kf_id);
      if (left_raw_it == left_by_kf_raw.end() || right_raw_it == right_by_kf_raw.end()) continue;
      ++num_keyframes_with_camera;

      const auto sonar_it = sonar_by_kf.find(kf_id);
      uw::runtime::SynchronizerParams sync_params;
      // 同步器以及下游的 time_delta 消费的仍然是**原始**图像 header
      // （capture_time、sensor_frame）：校正只改变像素内容和 frame 命名，不改变采集
      // 时刻。
      const auto sync_decision = uw::runtime::SynchronizeAcousticOptic(
          left_raw_it->second, std::optional<uw::domain::ImageFrame>(right_raw_it->second),
          sonar_it != sonar_by_kf.end() ? std::optional<uw::domain::SonarFrame>(sonar_it->second)
                                        : std::nullopt,
          *rig, sync_params);

      auto* rectified = get_rectified(kf_id);
      if (rectified == nullptr) continue;
      const auto optical_evidence =
          stereo_frontend.Process(rectified->images, rectification_context->DerivedRig());
      if (!optical_evidence.has_value()) continue;

      // kSynchronized 和 kTimeDeltaExceeded 都会把**真实的**声呐假设和**真实的**时间
      // 差交给 Fuse()。把过大的时间差变成一条 REJECTED/TIME_DELTA 记录（且发生在任何
      // 几何投影之前）是关联器自己第一道时间闸的职责
      // （frontends/acoustic_optic_associator.cpp）——所以本文件不再需要、也**不允许**
      // 伪造一个零时间差去绕过那个决定。
      //
      // kNoSonar 和 kInvalidTimestamp 则带着一个空假设集落下去，于是 Fuse() 返回一个
      // 纯光学的结果、完全没有关联记录——**光学链路不会因为跨模态配对失败就停下来**。
      uw::domain::HypothesisSet sonar_hypotheses;
      switch (sync_decision.status) {
        case uw::runtime::SynchronizationStatus::kSynchronized:
        case uw::runtime::SynchronizationStatus::kTimeDeltaExceeded:
          sonar_hypotheses = sonar_frontend.ProcessSonarFrame(sonar_it->second);
          break;
        case uw::runtime::SynchronizationStatus::kNoSonar:
          break;
        case uw::runtime::SynchronizationStatus::kInvalidTimestamp:
          ++num_sync_invalid_timestamp;
          break;
      }

      const auto fused_result =
          fusion_frontend.Fuse(sonar_hypotheses, *optical_evidence, rectification_context->DerivedRig(),
                                sync_decision.max_pairwise_time_delta_s);
      if (!fused_result.has_value()) continue;

      const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_result->fused_evidence);
      const auto contribution_counts = CountDepthContributions(fused);
      total_contributions.optical_only_points += contribution_counts.optical_only_points;
      total_contributions.acoustic_optic_points += contribution_counts.acoustic_optic_points;
      if (fused.associations_size() > 0) {
        switch (fused.associations(0).status()) {
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED:
            ++num_acoustic_optic_accepted;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS:
            ++num_acoustic_optic_ambiguous;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT:
            ++num_acoustic_optic_conflict;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED:
            ++num_acoustic_optic_rejected;
            break;
          default:
            break;
        }
      }

      const uint64_t state_version = kf_index + 1;
      const auto map_evidence = uw::mapping::BuildMapEvidenceFromFusedDepth(
          fused_result->fused_evidence, rectification_context->DerivedRig(), bridge_params, kf_id,
          state_version);
      if (map_evidence.has_value()) {
        num_map_evidence_points +=
            static_cast<int>(map_evidence->geometry_or_occupancy().size() / (3 * sizeof(float)));
        submap_manager.AddMapEvidence(*map_evidence);
      }
    }
    std::cout << "acoustic-optic: " << num_keyframes_with_camera << " keyframes with camera data, "
              << num_acoustic_optic_accepted << " accepted, " << num_acoustic_optic_ambiguous
              << " ambiguous, " << num_acoustic_optic_conflict << " conflict, " << num_acoustic_optic_rejected
              << " rejected, " << num_sync_invalid_timestamp
              << " sync invalid-timestamp (optical-only), " << num_map_evidence_points
              << " map evidence points added (" << total_contributions.optical_only_points
              << " optical-only, " << total_contributions.acoustic_optic_points << " acoustic-optic)\n";
  }

  const auto ate =
      uw::evaluation::ComputeAte(estimated_trajectory, ground_truth_trajectory, 0.05, opt.align_ate);
  std::cout << "ATE: rmse=" << ate.rmse_m << "m mean=" << ate.mean_m << "m max=" << ate.max_m
            << "m (" << ate.num_matched_poses << " matched poses)\n";

  uw::application::ReplayRunSummary run_summary;
  run_summary.estimator_mode = estimator_mode;
  run_summary.solver = defaults.solver;
  run_summary.solver_converged = summary.converged;
  run_summary.solver_iterations = summary.iterations;
  run_summary.initial_cost = summary.initial_cost;
  run_summary.final_cost = summary.final_cost;
  run_summary.keyframe_count = static_cast<int>(problem.NumKeyframes());
  run_summary.keyframe_boundary_count = static_cast<int>(input.keyframe_boundaries.size());
  run_summary.imu_factor_count = num_imu_factors;
  run_summary.imu_interval_rejected_count = num_imu_intervals_rejected;
  run_summary.relative_pose_factor_count = num_relative_pose_factors;
  run_summary.loop_closure_factor_count = num_loop_closure_factors;
  run_summary.sonar_range_factor_count = num_sonar_factors;
  run_summary.depth_factor_count = num_depth_factors;
  run_summary.landmark_count = num_landmarks_discovered;
  if (imu_mode) {
    run_summary.initialization =
        inertial_initialization->mode ==
                uw::frontends::ImuStationaryInitialization::Mode::kStationary
            ? "stationary"
            : "wide_velocity_prior";
  }
  run_summary.free_parameter_dim = observability.free_parameter_dim;
  run_summary.residual_dim = observability.residual_dim;
  run_summary.ate_rmse_m = ate.rmse_m;
  run_summary.ate_matched_poses = ate.num_matched_poses;
  std::cout << uw::application::FormatReplayRunSummary(run_summary);

  {
    std::ofstream traj_out(opt.out_prefix + "_trajectory.tum");
    for (const auto& pose_entry : estimated_trajectory) {
      traj_out << ToTumLine(pose_entry.timestamp_s, pose_entry.pose_WB) << "\n";
    }
  }

  if (write_run_manifest) {
    uw::runtime::RunManifest manifest;
    manifest.run_id = "replay_demo_" + std::to_string(
                                            std::chrono::duration_cast<std::chrono::seconds>(
                                                std::chrono::system_clock::now().time_since_epoch())
                                                .count());
    manifest.git_commit = git_commit;
    manifest.config_hash = config_hash;
    manifest.calibration_hash = calibration_hash;
    manifest.derived_calibration_hash =
        rectification_context.has_value()
            ? Fnv1aHex(rectification_context->DerivedRig().SerializeAsString())
            : "";
    manifest.dataset_or_scenario = opt.bag_path;
    manifest.simulator = "synthetic (apps/synth_bag_gen.cpp)";
    manifest.os_info = DetectOsInfo();
    manifest.cpu_info = DetectCpuInfo();
    manifest.gpu_info = "n/a (CPU-only Eigen pipeline, no GPU dependency in this build)";
    manifest.seed = scenario_seed;
    manifest.start_time_iso8601 = run_start_time_iso8601;
    manifest.end_time_iso8601 = NowIso8601Utc();
    std::ofstream manifest_out(opt.out_prefix + "_run_manifest.json");
    manifest_out << manifest.ToJson();
    std::cout << "wrote " << opt.out_prefix << "_trajectory.tum and " << opt.out_prefix
              << "_run_manifest.json\n";
  } else {
    std::cout << "wrote " << opt.out_prefix << "_trajectory.tum (run_manifest disabled by config)\n";
  }

  // P0 的"非空洞"门禁（见 docs/archive/uw-slam-production-readiness-and-roadmap-
  // 2026-08-21.md 5.5 节）。
  //
  // 关键约定：上面那些输出**即使失败也照写**，只有退出码变——所以 CI 门禁失败之后，
  // 现场仍然留着轨迹和 manifest 可供排查，而不是什么都没有。
  //
  // 每一项门禁都通过 defaults.* / experiment 的 `gates:` 单独 opt-in
  // （见 include/runtime/config.hpp），唯一例外是 require_converged：它默认开启，
  // 因为一个 stalled 的求解器产出在任何情况下都不是可接受的结果。
  const std::vector<std::string> gate_failures =
      EvaluateReplayGates(defaults, summary, ate, num_landmarks_discovered, total_contributions,
                          num_acoustic_optic_accepted);
  if (!gate_failures.empty()) {
    std::cerr << "GATE FAILURE:\n";
    for (const auto& failure : gate_failures) std::cerr << "  - " << failure << "\n";
    return 2;
  }
  return 0;
}
