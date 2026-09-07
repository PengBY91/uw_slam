// 生成一份带已知真值轨迹、并把观测量人工造出来的合成 MCAP bag（包括相对位姿
// "VIO" 证据、对若干固定目标的声呐 range-bearing 观测，以及深度）。它让
// apps/replay_demo 能把 FactorBuilder -> PoseGraphProblem -> GaussNewtonSolver
// -> StateStore -> SubmapManager 这一整条链路端到端跑通。
//
// 注意它存在的**理由已经变了**：最早写这个文件时，本机既没有 HoloOcean 也没有
// ROS2，那时它是唯一的数据来源。现在不是了——WSL2 里装了 ROS2 Jazzy（apt，
// colcon workspace 在 ~/ros2_ws），宿主 Windows 上跑着 UE5.3 + HoloOcean 引擎工程
// 和自制世界包，Windows↔WSL2 的 ROS2 话题桥接也是通的（见
// adapters/holoocean/docs/ue5-world-packaging.md）。所以"没有仿真器"不再是理由，
// 真正让这个合成 fixture 继续不可替代的是下面四条：
//
//   1. 真值是**构造出来的**，不是标出来的：轨迹、目标位置、深度都由解析式给出，
//      评测因此没有真值本身的误差。
//   2. 逐字节可复现：同一个 seed 产出同一份 bag，这是 tests/integration/
//      determinism_test.sh 和声光场景矩阵 gate 的立足点；真实仿真器做不到。
//   3. 不需要仿真器、GPU、也不需要那台 Windows 主机——任何一台能编译本仓库的机器
//      都能跑，CI 因此可以无条件跑完整回放链路。
//   4. 可以**故意造假**：下面那几个泄漏检测 flag 会丢掉或污染 /gt/state 而不动其他
//      任何字节，真实传感器数据没法这样被精确扰动。
//
// ================= 这份文件的主要逻辑 =================
//
// 整体是一个"先算真值、再按传感器模型造观测、最后写进 MCAP"的单进程离线生成器，
// 执行顺序如下：
//
//   1. 参数装配（main() 里的两趟解析）。第一趟只找 --experiment，把
//      configs/experiment/*.yaml 经分层配置（defaults -> rig -> scenario ->
//      experiment）解析出来的 scenario 段盖到 ScenarioOptions 的内置默认值上
//      （ApplyScenarioConfig）；同时记下两个可选的 rig：带相机的 rig 决定要不要出
//      双目图像，estimator_mode: imu_preintegration 的 rig 决定要不要出 IMU 链路。
//      第二趟才解析显式 CLI flag，所以 CLI 的优先级高于 --experiment。
//   2. 真值轨迹（BuildGroundTruthTrajectory + ArcPose）。轨迹是固定深度上的一段
//      圆弧，yaw 始终跟着切向转；keyframe 在归一化运动时间 s ∈ [0, 1] 上等分。
//      IMU 模式下 s -> 弧度的映射换成五次 smoothstep（ArcFraction），其他模式保持
//      匀角速度。
//   3. 随机数分流（MakeStreamRng）。pose / sonar / landmark / imu 四种噪声用途各开
//      一条独立的 mt19937_64，seed 相同、salt 不同，彼此的抽样次数互不影响——这条
//      纪律的由来见 MakeStreamRng 处的注释。
//   4. 场景级目标点云只写一次（/scenario/sonar_targets）。
//   5. IMU 流（仅 IMU 模式）在 keyframe 循环之前一次性写完，覆盖 [0, 最后一个
//      keyframe] 的整个区间：真值由 BodyImuTruth 解析求得，经 ToImuFrame 换算到
//      imu_link，再叠加初始零偏 + 零偏随机游走 + 白噪声。
//   6. keyframe 循环。每个 keyframe 依次产出：keyframe 边界（仅 IMU 模式）、真值
//      /gt/state（只供评测，可被泄漏检测 flag 关掉或污染）、与上一帧之间的相对位姿
//      证据、对每个在量程内的目标渲染的声呐帧、可选的双目图像对（BuildStereoPair），
//      以及深度证据。
//   7. 关闭 writer，打印一共写了多少个 keyframe。
//
// 有两条贯穿全文的设计约束，改这份文件时要一直记着。其一，真值只允许出现在
// /gt/state 上，任何算法输入话题都不许夹带真值通道（IMU 样本里的 bias 字段刻意留空
// 就是这个道理）；其二，除 IMU 模式之外，每一条既有产出路径都要与改动前逐字节一致，
// 所以所有 IMU 相关行为都统一挂在同一个开关后面。
//
// 写出的话题（全部 protobuf 编码，见 uw::runtime::McapProtobufWriter）：
//   /gt/state                    uw.domain.StateSnapshot   （每个 keyframe 的真值）
//   /evidence/relative_pose      uw.domain.MeasurementEvidence（RelativePoseMeasurement）
//   /raw/sonar_frame             uw.domain.SonarFrame      （合成成像声呐 ping；见下面的
//                                 RenderSyntheticSonarFrame——replay_pipeline 会把它送进
//                                 真正的 sonar_cfar_frontend（include/frontends、
//                                 src/frontends），它**不是**预先算好的 range-bearing 证据）
//   /evidence/depth              uw.domain.MeasurementEvidence（PressureDepthMeasurement）
//   /scenario/sonar_targets      uw.domain.MapEvidence     （已知目标位置，world frame，
//                                 打包成 float32 xyz——借用点云载荷来装；v1 还没有真正的
//                                 路标/子图查询，这条话题是它的替身，见 apps/replay_demo）
//   /raw/camera/left             uw.domain.ImageFrame      （仅当 --experiment 加载的 rig
//   /raw/camera/right            uw.domain.ImageFrame       带相机时才写；见 BuildStereoPair
//                                 ——每个 keyframe 一对合成双目图，几何是逐帧真实算出来的，
//                                 供 apps/replay_demo 的声光融合环节使用）
//   /raw/imu                     uw.domain.ImuSample       （仅当 --experiment 选择
//   /keyframe/boundary           uw.domain.KeyframeBoundary estimator_mode: imu_preintegration
//                                 时才写；见下面的「IMU fixture」）
//
// 下面这些 flag 只为测试真值泄漏而存在，不描述任何真实传感器：
// --omit-relative-pose 丢掉 /evidence/relative_pose；--omit-ground-truth、
// --ground-truth-time-offset-s 和 --ground-truth-pose-offset-m 则是丢掉或污染
// /gt/state。这四个 flag 都不会改动其他任何话题的一个字节，所以只要某条管线的输出
// 在用了其中之一后发生变化，就说明它读了不该读的东西。
//
// IMU fixture（PREP-B-01，docs/imu-preintegration-design-2026-09-03.md 第 7-8 节）。
// 只有 `estimator_mode: imu_preintegration` 会打开它，因此其他实验产出的 bag 与之前
// 逐字节相同。有三件事是一起变的，也只有放在一起才讲得通：
//
//   1. 前面接一段**静止预滚**（kImuPreRollS，0.75 s）：/raw/imu 从 t = 0 开始，
//      而载体在预滚结束之前不动。静止初始化器就是靠这段数据估计陀螺/加计的初始零偏
//      和重力方向——没有它，这两个量就没有合法的（非真值）来源。
//   2. 所有挂在 keyframe 上的话题整体平移同样的时长，并且每个 keyframe 都显式发一条
//      /keyframe/boundary 事件。这条边界流——不是 /gt/state，也不是相对位姿证据——才
//      是"一段预积分区间从哪开始、到哪结束"的唯一契约，它刻意不携带任何位姿。
//   3. 圆弧改用**五次 smoothstep 角度剖面**来遍历，而不是匀角速度（见 ArcFraction）。
//      几何上还是同一个圆；变的是 dθ/dt 和 d²θ/dt² 在起点处为零，于是载体在第一个
//      keyframe 处是真正静止的。匀角速度的弧会让机体在预滚结束的瞬间从 0 跳到
//      R·ω ≈ 5 m/s，任何 IMU 流都表达不了这种跳变——静止初始化器给出的 v₀ = 0 就会
//      差这么多，之后再怎么估计也救不回这条轨迹。其他 estimator 模式仍保持它们一直
//      在用的匀角速度剖面。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_io.hpp"
#include "runtime/synthetic_sonar.hpp"
#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"
#include "sensor_models/sonar_beam_model.hpp"

using uw::sensor_models::Pose3;

namespace {

struct ScenarioOptions {
  std::string out_path = "/tmp/synthetic.mcap";
  int num_keyframes = 12;
  double radius_m = 8.0;
  double arc_radians = 1.4;
  double depth_m = 12.0;
  double relative_pose_noise_m = 0.02;
  double sonar_range_noise_m = 0.03;
  double sonar_bearing_noise_rad = 0.01;
  uint64_t seed = 42;
  // 留空 => BuildSonarTargets() 回退到它内置的默认目标（保留这条路径是为了不带
  // --experiment 时也能单独跑，也为了 README.md 里现有的确定性/demo 命令不用改。）
  std::vector<Eigen::Vector3d> sonar_targets_world;
};

// 把 configs/scenario/*.yaml（经由 --experiment configs/experiment/*.yaml，即架构
// 文档 14.2 节的 defaults->rig->scenario->experiment 分层）盖到 ScenarioOptions 的
// 内置默认值上。在这次调用**之后**解析的显式 CLI flag 仍然优先（见 main()）——这里
// 是"scenario"层，CLI flag 是叠在它上面、更具体的临时覆盖。
void ApplyScenarioConfig(const uw::runtime::ScenarioConfig& scenario, ScenarioOptions& opt) {
  opt.num_keyframes = scenario.num_keyframes;
  opt.radius_m = scenario.radius_m;
  opt.arc_radians = scenario.arc_radians;
  opt.depth_m = scenario.depth_m;
  opt.relative_pose_noise_m = scenario.noise.relative_pose_noise_m;
  opt.sonar_range_noise_m = scenario.noise.sonar_range_noise_m;
  opt.sonar_bearing_noise_rad = scenario.noise.sonar_bearing_noise_rad;
  opt.seed = scenario.seed;
  if (!scenario.sonar_targets_world.empty()) {
    opt.sonar_targets_world = scenario.sonar_targets_world;
  }
}

// 在归一化运动时间 s ∈ [0, 1] 处，载体沿弧走了多远，以 arc_radians 的比例表示。
//
// `smooth_start` = false 是原来的匀角速度遍历，所有非 IMU 实验都在用、也必须继续用。
// `smooth_start` = true 则是五次 smoothstep 10s³ − 15s⁴ + 6s⁵，它的一阶**和**二阶
// 导数在两端都为零：于是机体的角速度和比力在"预滚/运动"接缝处都是连续的，所以
// "t = 0.5 s 之前保持静止"说的是轨迹本身，而不只是那之前的采样点。为什么静止预滚
// 后面不能接匀角速度的弧，见本文件头部注释。
double ArcFraction(double s, bool smooth_start) {
  if (!smooth_start) return s;
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// ArcFraction 对 s 的一阶与二阶导数，用来解析地推出 IMU 真值，而不是靠差分位姿。
double ArcFractionRate(double s, bool smooth_start) {
  if (!smooth_start) return 1.0;
  return s * s * (30.0 - 60.0 * s + 30.0 * s * s);
}

double ArcFractionAcceleration(double s, bool smooth_start) {
  if (!smooth_start) return 0.0;
  return s * (60.0 - 180.0 * s + 120.0 * s * s);
}

Pose3 ArcPose(const ScenarioOptions& opt, double theta) {
  Pose3 pose;
  pose.translation =
      Eigen::Vector3d(opt.radius_m * std::sin(theta), opt.radius_m * (1.0 - std::cos(theta)),
                      -opt.depth_m);
  pose.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  return pose;
}

std::vector<Pose3> BuildGroundTruthTrajectory(const ScenarioOptions& opt, bool smooth_start) {
  std::vector<Pose3> trajectory;
  trajectory.reserve(opt.num_keyframes);
  for (int i = 0; i < opt.num_keyframes; ++i) {
    const double s = opt.num_keyframes > 1
                          ? static_cast<double>(i) / (opt.num_keyframes - 1)
                          : 0.0;
    trajectory.push_back(ArcPose(opt, opt.arc_radians * ArcFraction(s, smooth_start)));
  }
  return trajectory;
}

std::vector<Eigen::Vector3d> BuildSonarTargets(const ScenarioOptions& opt) {
  if (!opt.sonar_targets_world.empty()) return opt.sonar_targets_world;
  // 不带 --experiment 单独跑时的回退：在航迹附近放几个类似海底特征的固定点，距离
  // 足够近，使得大多数 keyframe 至少能在合理声呐量程内看到一个。
  return {
      Eigen::Vector3d(2.0, 3.0, -opt.depth_m - 1.0),
      Eigen::Vector3d(6.0, 6.0, -opt.depth_m + 0.5),
      Eigen::Vector3d(-1.0, 8.0, -opt.depth_m - 0.5),
  };
}

std::string KeyframeId(int i) { return "kf" + std::to_string(i); }

// 5 Hz 的 keyframe，与最初的 fixture 保持一致。
constexpr double kKeyframePeriodS = 0.2;
// docs/imu-preintegration-design-2026-09-03.md 第 7 节要求第一个 keyframe 之前
// **至少**有 0.5 s 的静止 IMU 数据，静止初始化器才有东西可以用来估初始零偏和重力
// 方向。这里刻意留了余量而不是卡在下限上：正好 0.5 s 时，初始化器的
// "window_duration_s >= min_stationary_duration_s" 判定只能靠浮点数恰好相等才通过，
// 于是任何不能整除 0.5 s 的采样率（150 Hz 的最后一个预滚样本落在 0.4933 s）都会
// 悄悄把这次运行降级到宽速度先验，唯一的症状就是 ATE 变差。
constexpr double kImuPreRollS = 0.75;

uint64_t SecondsToNanos(double seconds) {
  return static_cast<uint64_t>(std::llround(seconds * 1e9));
}

// 本仓库所有合成数据生产者都遵循的 header 约定（见 runtime/synthetic_sonar.cpp）：
// receive_time 等于 capture_time，因为合成生成不建模任何传输延迟；而如果留着全零的
// 默认值，apps/bag_audit 会把它读成"从未填写过"。
uw::domain::ObservationHeader MakeSyntheticHeader(const std::string& observation_id,
                                                   const std::string& sensor_id,
                                                   const std::string& sensor_frame,
                                                   uint64_t t_ns) {
  uw::domain::ObservationHeader header;
  header.mutable_observation_id()->set_value(observation_id);
  header.mutable_sensor_id()->set_value(sensor_id);
  header.mutable_sensor_frame()->set_value(sensor_frame);
  header.mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
  header.mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
  *header.mutable_receive_time() = header.capture_time();
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header.set_provenance("synth_bag_gen_v1");
  return header;
}

// 归一化运动时间 `s` 处、BODY frame 下的解析 IMU 真值。
//
// 机体沿 p(theta) = (R sin, R(1-cos), -depth) 运动、yaw 为 theta，所以把世界系
// 加速度旋回机体系之后正好塌缩成 (R*theta_ddot, R*theta_dot^2, 0)：切向沿 +x、
// 向心沿 +y、z 方向没有分量。加速度计测的是比力 a_body - R_wb^T * g_w，其中
// g_w = (0, 0, -gravity)，而这里机体没有 roll/pitch，因此在机体 z 上多出一个常量
// +gravity。用这种解析方式（而不是差分位姿）推导，才让预滚接缝处严丝合缝：s = 0 时
// smoothstep 的一阶、二阶导数都为零，于是这里退化成静止读数 (0, 0, gravity)，没有跳变。
struct ImuTruth {
  Eigen::Vector3d specific_force_mps2;
  Eigen::Vector3d angular_velocity_radps;
  Eigen::Vector3d angular_acceleration_radps2;
};

ImuTruth StationaryImuTruth(double gravity_mps2) {
  return ImuTruth{Eigen::Vector3d(0.0, 0.0, gravity_mps2), Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero()};
}

ImuTruth BodyImuTruth(const ScenarioOptions& opt, double motion_duration_s, double s,
                      double gravity_mps2) {
  if (motion_duration_s <= 0.0) return StationaryImuTruth(gravity_mps2);
  const double theta_rate =
      opt.arc_radians * ArcFractionRate(s, /*smooth_start=*/true) / motion_duration_s;
  const double theta_acceleration = opt.arc_radians *
                                    ArcFractionAcceleration(s, /*smooth_start=*/true) /
                                    (motion_duration_s * motion_duration_s);
  ImuTruth truth;
  truth.specific_force_mps2 = Eigen::Vector3d(opt.radius_m * theta_acceleration,
                                              opt.radius_m * theta_rate * theta_rate,
                                              gravity_mps2);
  truth.angular_velocity_radps = Eigen::Vector3d(0.0, 0.0, theta_rate);
  truth.angular_acceleration_radps2 = Eigen::Vector3d(0.0, 0.0, theta_acceleration);
  return truth;
}

// base_link 读数 -> imu_link 读数：正好是 frontends/imu_preintegration_frontend 把
// 原始样本映射回机体系那一步的逆运算。安装在距机体原点 r 处的 IMU 还会感受到杆臂项
// omega x (omega x r) 与 alpha x r。前端刻意丢掉了 alpha x r（一个有记录的近似）；
// 而在 configs/rig/example_auv_sonar_only.yaml 里 imu_link 那条边是单位变换，两项都
// 精确为零，所以在这个 fixture 上两边逐比特一致——这里保留这一项，只是为了将来某个
// 带真实安装偏置的 rig 不会被悄悄生成错。
ImuTruth ToImuFrame(const ImuTruth& body, const Pose3& base_link_T_imu_link) {
  const Eigen::Matrix3d rotation = base_link_T_imu_link.rotation.toRotationMatrix();
  const Eigen::Vector3d& r = base_link_T_imu_link.translation;
  const Eigen::Vector3d& omega = body.angular_velocity_radps;
  const Eigen::Vector3d lever_arm =
      omega.cross(omega.cross(r)) + body.angular_acceleration_radps2.cross(r);
  ImuTruth sensor;
  sensor.specific_force_mps2 = rotation.transpose() * (body.specific_force_mps2 + lever_arm);
  sensor.angular_velocity_radps = rotation.transpose() * omega;
  sensor.angular_acceleration_radps2 = rotation.transpose() * body.angular_acceleration_radps2;
  return sensor;
}

// 注意这里**没有**写什么：has_bias / bias_* 一律留空。这些字段的含义是"传感器上报了
// 它自己内部的零偏估计"，把仿真器的精确零偏真值填进去，就等于在算法输入话题上开了
// 一条真值通道——而这正是 PREP-B-01 要堵死的东西。
uw::domain::ImuSample MakeImuSample(uint64_t t_ns, int index, const Eigen::Vector3d& specific_force,
                                    const Eigen::Vector3d& angular_velocity) {
  uw::domain::ImuSample sample;
  *sample.mutable_header() =
      MakeSyntheticHeader("imu_" + std::to_string(index), "imu0", "imu_link", t_ns);
  for (int i = 0; i < 3; ++i) {
    sample.add_linear_acceleration_mps2(specific_force(i));
    sample.add_angular_velocity_radps(angular_velocity(i));
  }
  return sample;
}

// 只携带这个 keyframe 的身份和它的时刻，别的什么都没有。位姿字段一律不从真值填充
// ——这正是这条话题存在的全部意义（docs/imu-preintegration-design-2026-09-03.md
// 第 8 节）。
uw::domain::KeyframeBoundary MakeKeyframeBoundary(uint64_t t_ns, const std::string& kf_id) {
  uw::domain::KeyframeBoundary boundary;
  *boundary.mutable_header() =
      MakeSyntheticHeader("boundary_" + kf_id, "keyframe_scheduler", "base_link", t_ns);
  boundary.mutable_keyframe_id()->set_value(kf_id);
  boundary.set_source("synthetic_fixed_interval_v1");
  return boundary;
}

// 用一个起区分作用的 salt（几个任意的 splitmix64 常数，选它们只是为了互不相同），
// 从 scenario 的顶层 seed 派生出某一种特定噪声用途专用的独立 RNG 流。正是这一点，
// 才让逐目标的声呐噪声抽样（抽几次取决于每个 keyframe 有多少 sonar_targets_world
// 落在量程内）不会把位姿噪声流带跑偏：每种用途各有一个由 {seed, salt} 一次性播种的
// std::mt19937_64，某条流消耗了多少次抽样，永远不会改变另一条流产生什么。在这之前
// 三种用途共用一个 rng，于是 scenario/acoustic_optic_demo.yaml 只有一个声呐目标
// （而 synthetic_smoke.yaml 有三个）就会悄悄改变实际烘焙进 bag 的、所谓"seed 42"的
// 相对位姿噪声。
std::mt19937_64 MakeStreamRng(uint64_t seed, uint64_t salt) {
  std::seed_seq seq{static_cast<uint32_t>(seed), static_cast<uint32_t>(seed >> 32),
                     static_cast<uint32_t>(salt), static_cast<uint32_t>(salt >> 32)};
  return std::mt19937_64(seq);
}

// --- 可选的逐 keyframe 双目图像，只在 --experiment 加载了带相机的 rig 时才产出
// （见 main() 里那个 `rig` 变量）。做法沿用 apps/acoustic_optic_scenarios.cpp 里
// 验证过的"先画背景、再贴目标"手法（为什么朴素的逐像素做法是错的，见那份文件
// MakeStereoPair 的头注释），但做了简化：不带噪声/退化变体——这个 app 不像那个
// 场景矩阵那样有深度精度评分要保护，只需要一个能跑、且诚实的场景，供
// apps/replay_demo 的声光融合环节使用。
constexpr uint32_t kCameraWidth = 640;
constexpr uint32_t kCameraHeight = 480;

// 比原来的整个 [0,255) range 调暗了：让背景严格低于 kLandmarkPatchMinIntensity
// （见 LandmarkPatchIntensity），这样基于阈值的路标检测器就不会把背景纹理当成
// 假路标捡起来。
uint8_t StereoTexture(int u, int v) { return static_cast<uint8_t>(15 + ((u * 131 + v * 67 + 19) % 110)); }

// 一个 3D 路标在某个 keyframe 上投影出来的像素足迹，外加用于查它 patch 图案的
// （场景内固定的）id——见 LandmarkPatchIntensity。从消费者的角度看，这个 id
// **不是**任何跟踪意义上的路标身份：真正的前端必须像面对真实相机那样，只从 patch
// 的外观/几何恢复对应关系，而不能靠这个 id。
struct VisibleLandmark {
  int id = 0;
  Eigen::Vector3d camera_optical;
};

constexpr int kLandmarksPerKeyframe = 10;
constexpr int kLandmarkPatchHalfSize = 6;  // 13x13 像素，小到一个 keyframe 的路标簇很少互相重叠
constexpr int kLandmarkPatchMinIntensity = 160;  // 高于 StereoTexture 的最大值（124），阈值分割得干净

// 沿航迹走廊撒下的路标点云：每个 keyframe **各自**的弧上位置附近聚一簇
// `kLandmarksPerKeyframe` 个点（半径/深度仍然加抖动，这样一簇点不共面——共面点集会让
// 刚体 Kabsch/Procrustes 拟合退化）。它替换掉了更早那版"在整段弧上均匀撒点"的做法：
// 总路标数固定时，一旦 arc_radians/radius_m 变大，均匀撒点的单位航程密度会稀得厉害，
// 于是相机（视场比声呐的 ~6 rad 窄得多）可能走不了几步就跟上一个 keyframe 失去重叠
// ——这是把 stereo_landmark_vo_frontend 端到端跑起来（而不只是跑它那些用手搭 fixture
// 的单测）才确认的：可见路标数到第 7 个 keyframe 就从 ~18 塌到 1。把密度直接锚在每个
// keyframe 上，就保证了不论轨迹多长，相邻 keyframe 之间都有健康的路标重叠。抖动取自
// 调用方传入的、带 seed 的 `rng`，一次性抽完——绝不中途重新 seed、绝不用全局 RNG
// （CLAUDE.md 的 RNG 纪律 / L2 确定性测试）。
std::vector<Eigen::Vector3d> BuildVisualLandmarks(const ScenarioOptions& opt, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> radius_jitter(0.6, 1.2);
  std::uniform_real_distribution<double> depth_jitter(-2.5, 2.5);
  const double keyframe_step_rad =
      opt.num_keyframes > 1 ? opt.arc_radians / (opt.num_keyframes - 1) : opt.arc_radians;
  std::uniform_real_distribution<double> theta_offset_jitter(-1.5 * keyframe_step_rad, 1.5 * keyframe_step_rad);

  std::vector<Eigen::Vector3d> landmarks;
  landmarks.reserve(static_cast<std::size_t>(kLandmarksPerKeyframe) * opt.num_keyframes);
  for (int kf = 0; kf < opt.num_keyframes; ++kf) {
    const double kf_t =
        opt.num_keyframes > 1 ? static_cast<double>(kf) / (opt.num_keyframes - 1) : 0.0;
    const double base_theta = kf_t * opt.arc_radians;
    for (int j = 0; j < kLandmarksPerKeyframe; ++j) {
      const double theta = base_theta + theta_offset_jitter(rng);
      const double radius = opt.radius_m * radius_jitter(rng);
      const double depth = -opt.depth_m + depth_jitter(rng);
      landmarks.emplace_back(radius * std::sin(theta), radius * (1.0 - std::cos(theta)), depth);
    }
  }
  return landmarks;
}

// 逐路标的确定性图案（一个与位置相关的小哈希，而不是均匀的一坨亮斑）：给每个路标 id
// 一份可复现、但外观上有区分度的足迹，好让将来的前端真能靠 patch 外观（归一化互相关
// 之类）把路标区分开，而不是所有"特征"长得一模一样——那样任何真实匹配器都没法消歧。
// 不是学出来的，也不是从哪里移植的——先例同 sonar_cfar_frontend 的 dbscan.hpp 和
// block_matcher.hpp（见 NOTICE）。
uint8_t LandmarkPatchIntensity(int landmark_id, int du, int dv) {
  uint32_t h = static_cast<uint32_t>(landmark_id) * 2654435761u;
  h ^= static_cast<uint32_t>((du + kLandmarkPatchHalfSize) * (2 * kLandmarkPatchHalfSize + 1) +
                              (dv + kLandmarkPatchHalfSize)) *
       2246822519u;
  h ^= h >> 13;
  h *= 3266489917u;
  h ^= h >> 16;
  return static_cast<uint8_t>(kLandmarkPatchMinIntensity + (h % (256 - kLandmarkPatchMinIntensity)));
}

Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig, const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return Pose3::FromProto(edge.transform());
  }
  return Pose3::Identity();
}

const uw::domain::CameraIntrinsics* FindRigCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                   const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> BuildStereoPair(
    const uw::sensor_models::StereoGeometry& stereo_geometry,
    const std::vector<VisibleLandmark>& visible_landmarks, uint64_t t_ns, const std::string& kf_id) {
  constexpr double kBackgroundDepthM = 15.0;
  const int background_disparity_px = std::max(
      1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m / kBackgroundDepthM)));

  std::string left_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  std::string right_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  for (uint32_t v = 0; v < kCameraHeight; ++v) {
    for (uint32_t u = 0; u < kCameraWidth; ++u) {
      left_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u), static_cast<int>(v)));
      right_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u) + background_disparity_px, static_cast<int>(v)));
    }
  }

  // 每个路标的 patch 会按它自己由深度算出的视差，同时画进**两幅**图，两边内容完全
  // 相同——这一点不同于它替换掉的那个单目标小技巧（那个只对 `right` 做扭曲，依赖
  // `left` 按构造本来就是同一个背景纹理值）。那个技巧能给稠密块匹配提供局部一致的
  // 视差，却造不出任何在单帧里显著、能被路标前端检测到并在 keyframe 之间跟踪的斑点；
  // 两边都画，才让每个路标成为真正可检测、可匹配的特征。
  for (const auto& landmark : visible_landmarks) {
    if (landmark.camera_optical.z() <= 0.5) continue;
    const Eigen::Vector2d pixel = stereo_geometry.left.Project(landmark.camera_optical);
    const int center_u = static_cast<int>(std::lround(pixel.x()));
    const int center_v = static_cast<int>(std::lround(pixel.y()));
    const int disparity_px = std::max(
        1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m /
                                        landmark.camera_optical.z())));
    for (int dv = -kLandmarkPatchHalfSize; dv <= kLandmarkPatchHalfSize; ++dv) {
      const int v = center_v + dv;
      if (v < 0 || v >= static_cast<int>(kCameraHeight)) continue;
      for (int du = -kLandmarkPatchHalfSize; du <= kLandmarkPatchHalfSize; ++du) {
        const int u_left = center_u + du;
        const int u_right = u_left - disparity_px;
        if (u_left < 0 || u_left >= static_cast<int>(kCameraWidth) || u_right < 0 ||
            u_right >= static_cast<int>(kCameraWidth)) {
          continue;
        }
        const char intensity = static_cast<char>(LandmarkPatchIntensity(landmark.id, du, dv));
        left_pixels[static_cast<std::size_t>(v) * kCameraWidth + static_cast<std::size_t>(u_left)] = intensity;
        right_pixels[static_cast<std::size_t>(v) * kCameraWidth + static_cast<std::size_t>(u_right)] = intensity;
      }
    }
  }

  auto make_frame = [&](const std::string& frame_name, std::string pixels) {
    uw::domain::ImageFrame image;
    image.mutable_header()->mutable_observation_id()->set_value(kf_id);
    image.mutable_header()->mutable_sensor_frame()->set_value(frame_name);
    image.mutable_header()->mutable_sensor_id()->set_value(
        frame_name == "camera_left_link" ? "camera_left" : "camera_right");
    image.mutable_header()->mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
    image.mutable_header()->mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
    // 合成生成没有真实的传输延迟要建模——receive_time 等于 capture_time（而不是留在
    // 全零 Stamp 默认值上，tools/bag_audit 会把那读成"从未填写"而不是"零延迟"）。
    *image.mutable_header()->mutable_receive_time() = image.header().capture_time();
    image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
    image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
    image.mutable_header()->set_provenance("synth_bag_gen_v1");
    image.set_width(kCameraWidth);
    image.set_height(kCameraHeight);
    image.set_row_stride_bytes(kCameraWidth);
    image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
    image.set_pixel_data(std::move(pixels));
    image.set_is_rectified(true);
    return image;
  };
  return {make_frame("camera_left_link", std::move(left_pixels)),
          make_frame("camera_right_link", std::move(right_pixels))};
}

}  // namespace

int main(int argc, char** argv) {
  ScenarioOptions opt;
  // 只有当 --experiment 加载到带相机的 rig 时才会赋值——它是下面
  // /raw/camera/left,right 产出的开关，这样"不带 --experiment"那条路径
  // （tests/l2_replay/determinism_test.sh 在用）可以证明没有任何改变。
  std::optional<uw::domain::RigCalibrationSnapshot> rig;
  // 当 --experiment 选择 estimator_mode: imu_preintegration 时改为赋值这一个。
  // 它跟上面的 `rig` 分开，是因为那个刻意只在**带相机**的 rig 上填（它是双目路径的
  // 开关），而 IMU 这条路径要的是像 configs/rig/example_auv_sonar_only.yaml 这种
  // 没有相机的 rig 里的 imu_noise 段和 base_link->imu_link 那条边。
  std::optional<uw::domain::RigCalibrationSnapshot> imu_rig;
  // 抑制 /evidence/relative_pose。IMU 估计器必须完全忽略这条话题，而"同一份场景在
  // 带与不带这条话题时分别生成一次 bag"正是 tests/integration/
  // synthetic_imu_fixture_test.sh 用来证明 IMU 流不依赖它的手段（每种噪声用途各用
  // 自己的 RNG 流，见 MakeStreamRng）。
  bool omit_relative_pose = false;
  // 只在参照分支上使用的旋钮（PREP-B-01 Task 6）。它们改变的只是**评测方**读到的
  // 东西：只碰 /gt/state，不消耗任何 RNG 抽样，而且刻意**不**移动真值消息的 MCAP
  // log time。正是这个组合让它们成为泄漏探测器——只要它们中的任何一个改变了估计轨迹
  // 的哪怕一个字节，就说明真值流进了算法路径
  // （tests/integration/imu_preintegration_smoke_test.sh）。
  bool omit_ground_truth = false;
  double ground_truth_time_offset_s = 0.0;
  double ground_truth_pose_offset_m = 0.0;

  // 第一趟：先找出 --experiment（如果有），在任何显式 CLI 覆盖生效之前，把它的
  // scenario 配置盖到 opt 上。
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--experiment") {
      if (i + 1 >= argc) {
        std::cerr << "missing value for --experiment\n";
        return 1;
      }
      const auto config = uw::runtime::LoadExperimentConfig(argv[++i]);
      ApplyScenarioConfig(config.scenario, opt);
      if (config.rig.cameras_size() > 0) rig = config.rig;
      if (config.estimator_mode == "imu_preintegration") imu_rig = config.rig;
    }
  }

  // 第二趟：显式 flag 覆盖 --experiment 设过的值。
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--experiment") {
      next("--experiment");  // 上面已经消费过了，这里只是把它的值跳过去
    } else if (arg == "--out") {
      opt.out_path = next("--out");
    } else if (arg == "--num-keyframes") {
      opt.num_keyframes = std::stoi(next("--num-keyframes"));
    } else if (arg == "--seed") {
      opt.seed = std::stoull(next("--seed"));
    } else if (arg == "--omit-relative-pose") {
      omit_relative_pose = true;
    } else if (arg == "--omit-ground-truth") {
      omit_ground_truth = true;
    } else if (arg == "--ground-truth-time-offset-s") {
      ground_truth_time_offset_s = std::stod(next("--ground-truth-time-offset-s"));
    } else if (arg == "--ground-truth-pose-offset-m") {
      ground_truth_pose_offset_m = std::stod(next("--ground-truth-pose-offset-m"));
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  // 四条互不相干的噪声流，salt 各不相同（见 MakeStreamRng）。
  std::mt19937_64 pose_rng = MakeStreamRng(opt.seed, 0x9E3779B97F4A7C15ULL);
  std::mt19937_64 sonar_rng = MakeStreamRng(opt.seed, 0xC2B2AE3D27D4EB4FULL);
  std::mt19937_64 landmark_rng = MakeStreamRng(opt.seed, 0x165667B19E3779F9ULL);
  // 第四条独立流，理由与上面三条相同：IMU 跑在 200 Hz，相对其他所有东西抽样次数都是
  // 变动的，所以它跟谁都不能共用一条流——两个方向上都不行。
  std::mt19937_64 imu_rng = MakeStreamRng(opt.seed, 0xD6E8FEB86659FD93ULL);
  std::normal_distribution<double> pose_noise(0.0, opt.relative_pose_noise_m);
  std::normal_distribution<double> range_noise(0.0, opt.sonar_range_noise_m);
  std::normal_distribution<double> bearing_noise(0.0, opt.sonar_bearing_noise_rad);

  // IMU fixture 以及由它带来的一切——静止预滚、时间平移、显式 keyframe 边界、平滑
  // 起步的角度剖面——全都由这一个开关控制，所以其他实验产出的 bag 保持逐字节不变。
  const bool imu_mode = imu_rig.has_value();
  const double pre_roll_s = imu_mode ? kImuPreRollS : 0.0;
  const uint64_t pre_roll_ns = SecondsToNanos(pre_roll_s);
  // 运动段总时长：keyframe 间隔固定 5 Hz，所以是 (帧数 - 1) * 0.2 s。
  const double motion_duration_s = std::max(0, opt.num_keyframes - 1) * kKeyframePeriodS;

  const auto trajectory = BuildGroundTruthTrajectory(opt, /*smooth_start=*/imu_mode);
  const auto targets = BuildSonarTargets(opt);
  uw::runtime::SyntheticSonarFrameSpec sonar_frame_spec;
  // 与 configs/rig/*.yaml 里 sonar_beam_models[0].sensor_id 以及
  // AcousticOpticAssociatorParams::sonar_sensor_id 的默认值对齐——这是必需项
  // （不是可有可无的元数据），因为 SynchronizeAcousticOptic() 会把空 sensor_id 判为
  // 非法时间戳 header 而拒绝掉。
  sonar_frame_spec.sensor_id = "sonar0";
  sonar_frame_spec.provenance = "synth_bag_gen_v1";
  // 只有加载了相机 rig 时才有意义（见上面的 `rig`）；抽样取自它自己的流
  // （landmark_rng），所以它抽不抽、抽多少，都不可能扰动上面的 pose/sonar 流。
  const std::vector<Eigen::Vector3d> visual_landmarks =
      rig.has_value() ? BuildVisualLandmarks(opt, landmark_rng) : std::vector<Eigen::Vector3d>{};

  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(opt.out_path)) {
    std::cerr << "failed to open " << opt.out_path << " for writing\n";
    return 1;
  }

  // 场景级目标清单，只写一次（时间戳为 0，因为它描述的是整个场景而不是某一时刻）。
  {
    uw::domain::MapEvidence targets_evidence;
    targets_evidence.mutable_keyframe_id()->set_value("scenario");
    targets_evidence.mutable_local_frame()->set_value("world");
    targets_evidence.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
    // 借用点云载荷：紧凑打包成 float32 的 xyz 三元组。
    std::string bytes(targets.size() * 3 * sizeof(float), '\0');
    auto* raw = reinterpret_cast<float*>(bytes.data());
    for (std::size_t i = 0; i < targets.size(); ++i) {
      raw[i * 3 + 0] = static_cast<float>(targets[i].x());
      raw[i * 3 + 1] = static_cast<float>(targets[i].y());
      raw[i * 3 + 2] = static_cast<float>(targets[i].z());
    }
    targets_evidence.set_geometry_or_occupancy(bytes);
    writer.WriteMessage("/scenario/sonar_targets", 0, targets_evidence);
  }

  // --- IMU 流（仅 estimator_mode: imu_preintegration）---------------------
  // 在 keyframe 循环之前一次性写完，覆盖 [0, 最后一个 keyframe] 的整个区间，这样每一
  // 段预积分区间的两端都被完整覆盖。消费者是按 log time 顺序读 bag 的
  // （McapEventSource 显式设置了 LogTimeOrder），所以先写这一块并不会让 IMU 在时间上
  // 排到 keyframe 类话题前面去。
  if (imu_mode) {
    const auto& imu_noise = imu_rig->imu_noise();
    const double rate_hz = imu_noise.rate_hz();
    const double gravity_mps2 = imu_noise.gravity_mps2();
    if (!(rate_hz > 0.0) || !(gravity_mps2 > 0.0)) {
      std::cerr << "rig imu_noise.rate_hz and gravity_mps2 must be positive for "
                   "estimator_mode: imu_preintegration\n";
      return 1;
    }
    const double dt_s = 1.0 / rate_hz;
    const Pose3 base_link_T_imu_link = FindRigEdgePose(*imu_rig, "imu_link");

    // 连续时间密度离散化后是 sigma_c * sqrt(rate)；零偏随机游走则是
    // sigma_walk_c * sqrt(dt)。sigma_*_bias 只表示**初始**零偏的量级——这两者是按决策
    // 分开的两个旋钮，见设计文档第 9 节第 3 条——所以它只用来播种那个常量偏置，绝不
    // 参与随机游走。
    std::normal_distribution<double> accel_white(0.0, imu_noise.sigma_accel_c() * std::sqrt(rate_hz));
    std::normal_distribution<double> gyro_white(0.0, imu_noise.sigma_gyro_c() * std::sqrt(rate_hz));
    std::normal_distribution<double> accel_bias_initial(0.0, imu_noise.sigma_accel_bias());
    std::normal_distribution<double> gyro_bias_initial(0.0, imu_noise.sigma_gyro_bias());
    std::normal_distribution<double> accel_bias_step(
        0.0, imu_noise.sigma_accel_bias_walk_c() * std::sqrt(dt_s));
    std::normal_distribution<double> gyro_bias_step(
        0.0, imu_noise.sigma_gyro_bias_walk_c() * std::sqrt(dt_s));

    // 整段数据共用同一个初始零偏，随后逐样本按随机游走漂移。
    Eigen::Vector3d accel_bias(accel_bias_initial(imu_rng), accel_bias_initial(imu_rng),
                               accel_bias_initial(imu_rng));
    Eigen::Vector3d gyro_bias(gyro_bias_initial(imu_rng), gyro_bias_initial(imu_rng),
                              gyro_bias_initial(imu_rng));

    const double total_duration_s = pre_roll_s + motion_duration_s;
    const int imu_sample_count = static_cast<int>(std::llround(total_duration_s * rate_hz));
    for (int sample_index = 0; sample_index <= imu_sample_count; ++sample_index) {
      const double t_s = static_cast<double>(sample_index) / rate_hz;
      // 严格处于预滚区间内时机体静止。正好落在第一个 keyframe 边界上的那个样本也仍然
      // 算作静止样本，这样预滚才是完整的 0.5 s 静止数据，而不是 0.5 s 少一个样本。
      const ImuTruth body_truth =
          t_s <= pre_roll_s
              ? StationaryImuTruth(gravity_mps2)
              : BodyImuTruth(opt, motion_duration_s,
                             std::min(1.0, (t_s - pre_roll_s) / motion_duration_s), gravity_mps2);
      const ImuTruth sensor_truth = ToImuFrame(body_truth, base_link_T_imu_link);

      // 实测读数 = 传感器系真值 + 当前零偏 + 白噪声。
      const Eigen::Vector3d measured_accel =
          sensor_truth.specific_force_mps2 + accel_bias +
          Eigen::Vector3d(accel_white(imu_rng), accel_white(imu_rng), accel_white(imu_rng));
      const Eigen::Vector3d measured_gyro =
          sensor_truth.angular_velocity_radps + gyro_bias +
          Eigen::Vector3d(gyro_white(imu_rng), gyro_white(imu_rng), gyro_white(imu_rng));
      writer.WriteMessage(uw::runtime::kTopicImu, SecondsToNanos(t_s),
                          MakeImuSample(SecondsToNanos(t_s), sample_index, measured_accel,
                                        measured_gyro));

      // 零偏随机游走：写完这个样本之后再往前走一步。
      accel_bias += Eigen::Vector3d(accel_bias_step(imu_rng), accel_bias_step(imu_rng),
                                    accel_bias_step(imu_rng));
      gyro_bias += Eigen::Vector3d(gyro_bias_step(imu_rng), gyro_bias_step(imu_rng),
                                   gyro_bias_step(imu_rng));
    }
  }

  // --- 逐 keyframe 产出：边界 / 真值 / 相对位姿 / 声呐 / 双目 / 深度 -------
  for (int i = 0; i < opt.num_keyframes; ++i) {
    // 5 Hz 的 keyframe；IMU 模式下整体后移一段静止预滚（其他模式下预滚为零，所以那些
    // bag 的时间戳保持原样）。
    const uint64_t t_ns = pre_roll_ns + static_cast<uint64_t>(i) * 200'000'000ULL;
    const std::string kf_id = KeyframeId(i);

    // 先写 keyframe 边界：它才是宣告"这个 keyframe 存在"的事件，下面所有东西都是挂在
    // 它上面的证据。
    if (imu_mode) {
      writer.WriteMessage(uw::runtime::kTopicKeyframeBoundary, t_ns,
                          MakeKeyframeBoundary(t_ns, kf_id));
    }

    // 真值。仅供参照：不会有任何算法输入从它派生（/gt/state 的角色定位见
    // include/runtime/canonical_topics.hpp），下面那三个 --*-ground-truth-* flag 就是
    // 用来证明这一点的。
    if (!omit_ground_truth) {
      uw::domain::StateSnapshot gt;
      gt.mutable_state_id()->set_value(kf_id);
      // 污染只施加在**采集时间戳**上；MCAP 的 log time 仍然是 t_ns。否则，一个
      // （错误地）从 log time 反推 keyframe 时刻的读取方会什么变化都看不到。
      const double gt_time_s =
          static_cast<double>(t_ns) * 1e-9 + ground_truth_time_offset_s;
      *gt.mutable_capture_timestamp() = uw::domain::FromSeconds(gt_time_s);
      Pose3 gt_pose = trajectory[i];
      gt_pose.translation += Eigen::Vector3d::Constant(ground_truth_pose_offset_m);
      *gt.mutable_pose_wb() = gt_pose.ToProto();
      writer.WriteMessage("/gt/state", t_ns, gt);
    }

    // 相邻 keyframe 之间的相对位姿证据（黑盒 VIO 模式，架构文档 8.1 节），平移上叠加
    // 加性噪声。
    if (i > 0 && !omit_relative_pose) {
      const Pose3 true_relative = trajectory[i - 1].Inverse() * trajectory[i];
      Pose3 noisy_relative = true_relative;
      noisy_relative.translation +=
          Eigen::Vector3d(pose_noise(pose_rng), pose_noise(pose_rng), pose_noise(pose_rng));

      uw::domain::RelativePoseMeasurement measurement;
      measurement.mutable_from_keyframe()->set_value(KeyframeId(i - 1));
      measurement.mutable_to_keyframe()->set_value(kf_id);
      *measurement.mutable_relative_pose() = noisy_relative.ToProto();

      uw::domain::EvidenceId evidence_id;
      evidence_id.set_value("relpose_" + kf_id);
      std::vector<uw::domain::ObservationId> sources;
      auto evidence = uw::domain::MakeEvidence<uw::domain::RelativePoseMeasurement>(
          evidence_id, sources, measurement, /*noise_scale=*/1.0, "synth_bag_gen_v1");
      writer.WriteMessage("/evidence/relative_pose", t_ns, evidence);
    }

    // 声呐：对每个落在合理量程内的目标各渲染一帧合成 ping，由 apps/replay_demo 送进
    // 真正的 sonar_cfar_frontend（不是预先算好的 range-bearing 证据——见文件头注释和
    // RenderSyntheticSonarFrame）。"一个在量程内的目标一帧"而不是"一帧涵盖所有目标"
    // 是合成 demo 的简化——它配合的是 apps/replay_demo 里那个有记录的路标关联替身，
    // 不是通用的"单 ping 多目标"传感器模型。
    for (const auto& target : targets) {
      const Eigen::Vector3d local = trajectory[i].Inverse().Apply(target);
      const double range = local.norm();
      if (range > 12.0) continue;  // 这个 keyframe 上超出了声呐量程
      const double bearing = std::atan2(local.y(), local.x());
      const double noisy_range = range + range_noise(sonar_rng);
      const double noisy_bearing = bearing + bearing_noise(sonar_rng);
      sonar_frame_spec.observation_id = kf_id;
      sonar_frame_spec.timestamp_ns = t_ns;
      auto rendered = uw::runtime::RenderSyntheticSonarFrame(
          sonar_frame_spec, noisy_range, noisy_bearing);
      // 渲染不到目标不算致命错误（帧照常写出，只有背景），但要出声——否则一个静默的
      // "只有背景"的 bag 看起来跟正常 bag 一模一样。
      if (!rendered.target_rendered) {
        std::cerr << "warning: target for " << kf_id
                  << " falls outside the synthetic sonar frame (range=" << noisy_range
                  << "m bearing=" << noisy_bearing
                  << "rad) — frame written background-only\n";
      }
      writer.WriteMessage("/raw/sonar_frame", t_ns, rendered.frame);
    }

    // 双目图像（仅当通过 --experiment 加载了带相机的 rig）：挑出真正落在相机视场
    // ——比声呐窄——内的路标与目标，再用 BuildStereoPair 造出这个 keyframe 真实的
    // 双目图像对。
    if (rig.has_value()) {
      const auto* left_intrinsics = FindRigCamera(*rig, "camera_left");
      const auto* right_intrinsics = FindRigCamera(*rig, "camera_right");
      if (left_intrinsics != nullptr && right_intrinsics != nullptr) {
        const auto stereo_geometry = uw::sensor_models::StereoGeometry::Resolve(
            *rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
        if (stereo_geometry.valid) {
          const Pose3 camera_pose = FindRigEdgePose(*rig, "camera_left_link");
          std::vector<VisibleLandmark> visible;
          // 世界系路标 -> 机体系 -> 左相机机体系 -> 相机 optical 系，再投影到像素上，
          // 逐个筛掉在相机背后或视场之外的。
          for (std::size_t li = 0; li < visual_landmarks.size(); ++li) {
            const Eigen::Vector3d local_body = trajectory[i].Inverse().Apply(visual_landmarks[li]);
            const Eigen::Vector3d local_camera_body = camera_pose.Inverse().Apply(local_body);
            const Eigen::Vector3d local_optical =
                uw::sensor_models::OpticalFromBodyRotation() * local_camera_body;
            if (local_optical.z() <= 0.5) continue;  // 在相机背后，或近到不合理
            const Eigen::Vector2d pixel = stereo_geometry.left.Project(local_optical);
            if (pixel.x() < 0 || pixel.x() >= stereo_geometry.left.width || pixel.y() < 0 ||
                pixel.y() >= stereo_geometry.left.height) {
              continue;  // 落在相机视场（比声呐窄）之外
            }
            visible.push_back(VisibleLandmark{static_cast<int>(li), local_optical});
          }
          // 恰好落进相机视场（比声呐窄）里的声呐目标也要画上去：不画的话，双目图在某个
          // 声呐目标投影像素处的视差永远只能读回那个平的 kBackgroundDepthM 背景平面
          // （BuildStereoPair 本来无从知道那里有个目标），于是 AcousticOpticAssociator
          // 的量程门永远找不到匹配的光学候选，无论几何多合理，每一次关联都会是
          // REJECTED/NO_CANDIDATE——而那些专门用来演示一次真实 ACCEPTED 关联的场景
          // （比如 configs/scenario/acoustic_optic_demo.yaml）需要它。给 id 加一个大的
          // 偏移量，可以让每个目标的 patch 图案（见 LandmarkPatchIntensity）与任何视觉
          // 路标的都不相同。
          for (std::size_t ti = 0; ti < targets.size(); ++ti) {
            const Eigen::Vector3d local_body = trajectory[i].Inverse().Apply(targets[ti]);
            const Eigen::Vector3d local_camera_body = camera_pose.Inverse().Apply(local_body);
            const Eigen::Vector3d local_optical =
                uw::sensor_models::OpticalFromBodyRotation() * local_camera_body;
            if (local_optical.z() <= 0.5) continue;
            const Eigen::Vector2d pixel = stereo_geometry.left.Project(local_optical);
            if (pixel.x() < 0 || pixel.x() >= stereo_geometry.left.width || pixel.y() < 0 ||
                pixel.y() >= stereo_geometry.left.height) {
              continue;
            }
            visible.push_back(VisibleLandmark{static_cast<int>(100000 + ti), local_optical});
          }
          auto stereo_pair = BuildStereoPair(stereo_geometry, visible, t_ns, kf_id);
          writer.WriteMessage("/raw/camera/left", t_ns, stereo_pair.first);
          writer.WriteMessage("/raw/camera/right", t_ns, stereo_pair.second);
        }
      }
    }

    // 深度。
    {
      uw::domain::PressureDepthMeasurement measurement;
      // depth_m 是正向下的量（world 为 Z-up）；所以取位姿 z 的相反数来生成它——见
      // schemas/proto/uw/domain/measurement.proto 里 PressureDepthMeasurement 的
      // 字段注释。
      measurement.set_depth_m(-trajectory[i].translation.z());
      measurement.set_sigma_m(0.05);
      uw::domain::EvidenceId evidence_id;
      evidence_id.set_value("depth_" + kf_id);
      uw::domain::ObservationId observation_id;
      observation_id.set_value(kf_id);
      std::vector<uw::domain::ObservationId> sources{observation_id};
      auto evidence = uw::domain::MakeEvidence<uw::domain::PressureDepthMeasurement>(
          evidence_id, sources, measurement, /*noise_scale=*/1.0, "synth_bag_gen_v1");
      writer.WriteMessage("/evidence/depth", t_ns, evidence);
    }
  }

  writer.Close();
  std::cout << "wrote " << opt.num_keyframes << " keyframes to " << opt.out_path << "\n";
  return 0;
}
