// OnlineAssistPipeline 与 StereoBlockMatchDenseDepthProvider 的实现。各类的职责、
// 依赖注入约定、诊断计数含义见头文件；这里讲清楚"一帧数据进来之后到底发生了什么"。
//
// ============================ 本文件主要逻辑 ============================
//
// 【一次事件的完整路径】三个真正驱动算法的入口 OnImageFrame / OnSonarFrame /
// OnVehicleState 形状一致，都是四步：
//   1. 把这条数据喂进 AcousticOpticBuffer（它负责按时间凑齐"立体+声呐+状态"的
//      同步 bundle）；
//   2. 跑本模态自己的检测（图像跑 RunVisualDetection、声呐跑 RunSonarDetection）；
//   3. 如果 buffer 恰好凑出了一个 bundle，就 HandleBundle（即尝试稠密深度）；
//   4. PublishNow() 发布当前态势（受限流约束）。
// 其余入口（IMU/DVL/keyframe/证据/真值/地图）一律"收下并返回 true"——不参与跟踪，
// 但也绝不阻塞在线循环。
//
// 【为什么声呐是关联批次的"节拍器"】视觉检测进来后只是暂存到 pending_visual_，
// 等下一帧声呐到达时一起 Associate + Update。原因是 TargetTracker::Update 整批原子，
// 一旦批里出现已被接受过的 observation_id 就整批拒绝——视觉若各自急着 flush，会先把
// 自己的 id 消费掉，之后就再也配不上声呐了。例外：声呐看起来已经掉线时
// （!SonarRecentlyLive），视觉立刻自己 flush，好让纯视觉航迹及时产出而不是无限等待。
//
// 【健康度与降级】ComputeDegradation 是一条**固定优先级**的判定链：
//   recovering > 两路全失联 > 车辆状态陈旧 > 声呐失联 > 视觉失联 >
//   视觉前端自报降级 > 声呐前端自报降级 > 稠密深度超期 > 健康。
// 前三档会把 guidance_valid 打成 false（引导不可信），之后几档只标 SUSPECT 但引导
// 仍然可用——这是"降级可用"与"不可用"的分界线，改动这条链要非常小心。
//
// 【三处时间口径，别混】
//   - 模态掉线判定（MarkRecoveringIfModalityWasDropped）比的是**相邻两次采集时间之
//     差**，反映传感器自身节拍，与排队/处理延迟无关；
//   - 存活判定（VisualLive/SonarLive/DenseCurrentlyFresh）比的是**当前时刻与最后一次
//     采集时刻之差**；
//   - 外部健康报告的过期判定用的是**我们收到它的时刻**，不信报告方自己的时钟。
//
// 【限流】PublishNow 默认受 min_publish_interval_s 约束（overload 档下入口调用可达
// ~145 次/秒，每次都渲染叠加层 + 重建 JSON 是扛不住的）。但状态跃迁必须立刻可见，
// 所以 UpdateRig 和 Flush 用 force=true 绕过限流。注意限流只挡"发布"，内部跟踪状态
// 该更新的照常更新。
// =======================================================================
#include "application/online_assist_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "frontends/sonar_target_extractor.hpp"
#include "frontends/stereo_optical_depth_frontend.hpp"
#include "frontends/target_fusion_components.hpp"
#include "runtime/acoustic_optic_buffer.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::application {

namespace {

using uw::sensor_models::FindCamera;

uw::domain::Stamp DefaultNow() {
  const double seconds =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  return uw::domain::FromSeconds(seconds);
}

uw::frontends::SensorTargetDetection Wrap(uw::domain::TargetDetection detection,
                                          const uw::domain::ObservationHeader& header) {
  uw::frontends::SensorTargetDetection wrapped;
  wrapped.sensor_id = header.sensor_id().value();
  wrapped.sensor_frame = header.sensor_frame().value();
  wrapped.calibration_version = header.calibration_version().value();
  wrapped.detection = std::move(detection);
  return wrapped;
}

}  // namespace

// ---------------------------------------------------------------------
// StereoBlockMatchDenseDepthProvider
// ---------------------------------------------------------------------

class StereoBlockMatchDenseDepthProvider::Impl {
 public:
  explicit Impl(uw::runtime::StereoMatchingConfig matcher_config) {
    uw::frontends::StereoOpticalDepthFrontendParams params;
    params.matcher.min_texture_variance = matcher_config.min_texture_variance;
    params.matcher.min_uniqueness_margin = matcher_config.min_uniqueness_margin;
    params.matcher.left_right_max_diff_px = matcher_config.left_right_max_diff_px;
    frontend_ = std::make_unique<uw::frontends::StereoOpticalDepthFrontend>(params);
  }

  std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) {
    const auto start = std::chrono::steady_clock::now();
    auto evidence = frontend_->Process(images, rig);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (!evidence.has_value() || elapsed_ms > budget_ms || !evidence->has_optical_depth_prior()) {
      return std::nullopt;
    }
    return evidence->optical_depth_prior();
  }

 private:
  std::unique_ptr<uw::frontends::StereoOpticalDepthFrontend> frontend_;
};

StereoBlockMatchDenseDepthProvider::StereoBlockMatchDenseDepthProvider(
    uw::runtime::StereoMatchingConfig matcher_config)
    : impl_(std::make_unique<Impl>(matcher_config)) {}
StereoBlockMatchDenseDepthProvider::~StereoBlockMatchDenseDepthProvider() = default;

std::optional<uw::domain::OpticalDepthPriorMeasurement> StereoBlockMatchDenseDepthProvider::RunBounded(
    const uw::measurement_api::CameraFrameBundle& images,
    const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) {
  return impl_->RunBounded(images, rig, budget_ms);
}

// ---------------------------------------------------------------------
// OnlineAssistPipeline
// ---------------------------------------------------------------------

struct DegradationDecision {
  uw::domain::HealthReport::Status status = uw::domain::HealthReport::STATUS_HEALTHY;
  std::string reason;
  bool guidance_valid = true;
};

struct ExternalHealthEntry {
  uw::domain::HealthReport report;
  double received_wall_s = 0.0;
};

class OnlineAssistPipeline::Impl {
 public:
  explicit Impl(OnlineAssistPipelineDependencies deps)
      : visual_frontend_(deps.visual_frontend),
        sonar_frontend_(deps.sonar_frontend),
        dense_provider_(deps.dense_depth_provider),
        sink_(deps.sink),
        rig_(deps.rig),
        association_config_(deps.target_association),
        tracker_config_(deps.target_tracker),
        pipeline_config_(deps.pipeline),
        now_(deps.now ? std::move(deps.now) : std::function<uw::domain::Stamp()>(&DefaultNow)),
        buffer_(deps.buffer, rig_) {
    if (!visual_frontend_ || !sonar_frontend_ || !sink_) {
      throw std::invalid_argument(
          "OnlineAssistPipeline requires non-null visual_frontend, sonar_frontend and sink");
    }
    fusion_.emplace(association_config_, tracker_config_);
  }

  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) {
    const auto& image = std::get<uw::domain::ImageFrame>(event.payload);
    auto bundle = buffer_.AddImage(image);
    if (image.header().sensor_id().value() == rig_.cameras(0).sensor_id().value()) {
      RunVisualDetection(image);
    }
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) {
    const auto& sonar = std::get<uw::domain::SonarFrame>(event.payload);
    auto bundle = buffer_.AddSonar(sonar);
    RunSonarDetection(sonar);
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnVehicleState(const uw::runtime::CanonicalEvent& event) {
    const auto& state = std::get<uw::domain::VehicleState>(event.payload);
    if (state.header().sensor_id().value() == rig_.vehicle_state_sensors(0).value()) {
      last_vehicle_state_capture_s_ = uw::domain::ToSeconds(state.header().capture_time());
    }
    auto bundle = buffer_.AddVehicleState(state);
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) {
    const auto& report = std::get<uw::domain::HealthReport>(event.payload);
    // 以**我们收到它的时刻**（now_()）为键，而不是报告方自己的 capture_time。
    // 这与本类别处判断视觉/声呐/车辆状态存活的口径一致（"距离上次听到它过了多久"），
    // 而且对时钟有偏差的报告方也稳健——否则一个时钟跑偏的组件可以让自己的报告
    // 永远显得很新鲜。
    external_health_[report.component_id()] = {report, uw::domain::ToSeconds(now_())};
    PublishNow();
    return true;
  }

  // DVL、真值/参考、通用测量证据和地图证据都不在本管线的算法输入之内：一律收下
  // （绝不因此卡住在线循环），但绝不进入跟踪。
  bool OnImuSample(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnDvlSample(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnReferenceState(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnMapEvidence(const uw::runtime::CanonicalEvent&) { return true; }

  bool Flush() {
    PublishNow(/*force=*/true);
    return true;
  }

  void UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
    const bool version_changed =
        rig.calibration_version().value() != rig_.calibration_version().value();
    buffer_.UpdateRig(rig);
    rig_ = std::move(rig);
    if (!version_changed) return;
    fusion_.emplace(association_config_, tracker_config_);
    pending_visual_.clear();
    pending_sonar_.clear();
    pending_dense_depth_.reset();
    pending_dense_depth_capture_s_.reset();
    recovering_ = true;
    ++diagnostics_.calibration_reset_count;
    // 进入 "recovering" 这个状态跃迁必须让飞手**立刻**看到，不能被
    // min_publish_interval_s 的限流拖延——所以这里 force 发布。
    PublishNow(/*force=*/true);
  }

  OnlineAssistPipelineDiagnostics Diagnostics() const { return diagnostics_; }

 private:
  void RunVisualDetection(const uw::domain::ImageFrame& left_image) {
    const double capture_s = uw::domain::ToSeconds(left_image.header().capture_time());
    MarkRecoveringIfModalityWasDropped(last_visual_capture_s_, capture_s);
    last_visual_capture_s_ = capture_s;
    const auto* intrinsics = FindCamera(rig_, left_image.header().sensor_id().value());
    if (!intrinsics) return;

    const std::optional<uw::domain::OpticalDepthPriorMeasurement> depth =
        DenseCurrentlyFresh(capture_s) ? pending_dense_depth_ : std::nullopt;

    const auto result = visual_frontend_->Process(left_image, depth, *intrinsics);
    last_visual_health_ = result.health;
    latest_path_lateral_offset_m_ = result.path_lateral_offset_m;
    latest_path_offset_sigma_m_ = result.path_offset_sigma_m;

    // 是**替换**不是追加：pending_visual_ 只保存最新一帧视觉的检测结果。
    //
    // 如果在这里把好几帧的检测攒起来，同一个目标的两次重复检测就会进入同一个
    // Associate()+Update() 批次。而 TargetTracker 每批对每条已有航迹最多只配一个
    // 检测——于是第二个没配上的重复检测（目标已被第一个配对的量测占了）会**凭空多出
    // 一条重复航迹**，而不是去加强那条真的。
    //
    // 代价是：夹在两帧声呐之间的那帧视觉，可能还没来得及被关联就被下一帧顶掉。这是
    // 可以接受的——跟踪器自己的预测/门控仍会用幸存的检测收敛到同一条航迹；而真的
    // 声呐掉线时（见下面的 SonarRecentlyLive）就不再替换，改成每帧视觉都 flush。
    pending_visual_.clear();
    for (const auto& target : result.targets) {
      pending_visual_.push_back(Wrap(target, left_image.header()));
    }
    diagnostics_.visual_detection_count += result.targets.size();
    // 正常工作时由声呐驱动关联批次（见 RunSonarDetection）：视觉检测只是暂存进
    // pending_visual_，等下一帧声呐到达配对，而不是立刻 flush。
    //
    // 为什么不能急着 flush：那会在声呐还没机会看到之前，就把这个 observation_id 消费
    // 掉；而 TargetTracker::Update 的批次是原子的，只要批里有任何一个已被接受过的 id
    // 就整批拒绝——于是这个"已经 flush 过又被复用"的陈旧检测，会毒害之后每一次配对。
    //
    // 只有在声呐本身看起来已经不可用时才在这里急着 flush，这样真出现声呐掉线，也能
    // 及时产出纯视觉航迹，而不是无限期干等。
    if (!SonarRecentlyLive(capture_s)) FlushAssociation(capture_s);
  }

  void RunSonarDetection(const uw::domain::SonarFrame& sonar) {
    const double capture_s = uw::domain::ToSeconds(sonar.header().capture_time());
    MarkRecoveringIfModalityWasDropped(last_sonar_capture_s_, capture_s);
    last_sonar_capture_s_ = capture_s;

    const auto hypotheses = sonar_frontend_->ProcessSonarFrame(sonar);
    last_sonar_health_ = sonar_frontend_->Health();
    const auto targets = sonar_extractor_.Extract(hypotheses, sonar);

    pending_sonar_.clear();
    for (const auto& target : targets) {
      pending_sonar_.push_back(Wrap(target, sonar.header()));
    }
    diagnostics_.sonar_detection_count += targets.size();
    FlushAssociation(capture_s);
  }

  bool SonarRecentlyLive(double capture_s) const {
    return last_sonar_capture_s_.has_value() &&
          (capture_s - *last_sonar_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  // FUS-HEALTH-002：从传感器掉线中恢复时，不许把故障前缓存的状态当作"这条流从没断
  // 过"一样静默复用——必须经过重新确认（或者显式的 recovering 状态），引导才重新
  // 可信。
  //
  // 此前只有标定版本变更（UpdateRig，因为 rig 几何本身变了，会强制整体重置关联器/
  // 跟踪器）才走这条路；单一模态的"掉线又恢复"没有对应机制，只能隐式指望缺口期间
  // 卡尔曼协方差自己长大，而不是一道显式的闸。
  //
  // 这里置的是与 UpdateRig **同一个** recovering_ 标志，但刻意**不**像 UpdateRig 那样
  // 重置 fusion_/pending_*/稠密深度：一次模态抖动并不会让 rig 几何失效，也不会让另一
  // 路仍然良好的航迹失效，整体重置属于毫无必要的破坏。FUS-HEALTH-002 真正要的是那道
  // 重新确认的闸（见 FlushAssociation 里的 any_confirmed 判断）。
  //
  // 比的是**采集时间之差**，而不是拿"现在"去比墙钟陈旧度：这里要衡量的是该模态相邻
  // 两次检测在它自己的传感器节拍上是否隔了超过 modality_stale_after_s，与排队/处理
  // 延迟无关。
  void MarkRecoveringIfModalityWasDropped(const std::optional<double>& last_capture_s,
                                          double new_capture_s) {
    if (last_capture_s.has_value() &&
        (new_capture_s - *last_capture_s) > pipeline_config_.modality_stale_after_s) {
      recovering_ = true;
      ++diagnostics_.modality_recovery_count;
    }
  }

  void FlushAssociation(double now_s) {
    const auto association = fusion_->associator().Associate(pending_visual_, pending_sonar_, rig_);
    if (!fusion_->tracker().Update(association.measurements, now_s)) {
      ++diagnostics_.association_reject_count;
    }
    // 刚提交过的 id（无论配对还是单例、无论被接受还是被拒）都不能再提交一次：
    // TargetTracker::Update 的原子批次遇到任何已接受过的 id 就整批拒绝；而被拒批次
    // 里的 id 如果留在这里不清，就会变成下一拍永远陈旧的"配对搭档"，把后续也一起
    // 拖死。所以无条件清空。
    pending_visual_.clear();
    pending_sonar_.clear();
    if (!recovering_) return;
    const auto tracks = fusion_->tracker().Tracks(now_s);
    const bool any_confirmed = std::any_of(tracks.begin(), tracks.end(), [](const auto& track) {
      return track.status == uw::domain::TARGET_TRACK_STATUS_CONFIRMED;
    });
    if (any_confirmed) recovering_ = false;
  }

  void HandleBundle(const uw::runtime::OnlineAcousticOpticBundle& bundle) {
    if (!pipeline_config_.dense.enabled || dense_task_in_flight_) return;
    if (!dense_provider_) {
      ++diagnostics_.dense_attempt_count;
      ++diagnostics_.dense_deadline_missed_count;
      return;
    }
    if (!bundle.images.primary.is_rectified() || !bundle.images.secondary.has_value() ||
        !bundle.images.secondary->is_rectified()) {
      // 校正（rectification）前提不满足：既不尝试，也不算作一次新的失败。
      //
      // 不用担心因此把问题藏起来：ComputeDegradation 判稠密深度健康度看的是
      // pending_dense_depth_ 自身的新鲜度（见 DenseCurrentlyFresh）。所以哪怕校正
      // 链路一直坏着，上一次成功的结果一旦过期，dense_deadline_missed 照样会浮现出来
      // ——这道闸不需要自己去锁存任何原因码。
      return;
    }

    dense_task_in_flight_ = true;
    ++diagnostics_.dense_attempt_count;
    auto result =
        dense_provider_->RunBounded(bundle.images, rig_, pipeline_config_.dense.budget_ms);
    dense_task_in_flight_ = false;

    if (result.has_value()) {
      pending_dense_depth_ = std::move(result);
      pending_dense_depth_capture_s_ = uw::domain::ToSeconds(bundle.images.primary.header().capture_time());
    } else {
      pending_dense_depth_.reset();
      pending_dense_depth_capture_s_.reset();
      ++diagnostics_.dense_deadline_missed_count;
    }
  }

  // 只有在稠密深度已启用、且上一次成功的结果仍在新鲜期内时，才算它"当前正在贡献"
  // ——这与 RunVisualDetection 拿它当深度先验之前所做的判断是同一个检查。
  //
  // 两处调用**故意**传不同的"现在"：这里的调用方 ComputeDegradation 是在发布时刻
  // 汇报整体状态；RunVisualDetection 的调用方则是在决定要不要给某一具体的、更早的
  // 那帧使用深度先验。由于这里的 wall_s 总是 >= 那帧的 capture_s，这个健康判断永远
  // 不会比使用判断更乐观，最多在两者之间那个很窄的窗口里保守一档——**保守是安全的
  // 那个方向**。
  bool DenseCurrentlyFresh(double wall_s) const {
    return pipeline_config_.dense.enabled && pending_dense_depth_.has_value() &&
          pending_dense_depth_capture_s_.has_value() &&
          (wall_s - *pending_dense_depth_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  bool VisualLive(double wall_s) const {
    return last_visual_capture_s_.has_value() &&
          (wall_s - *last_visual_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  bool SonarLive(double wall_s) const {
    return last_sonar_capture_s_.has_value() &&
          (wall_s - *last_sonar_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  DegradationDecision ComputeDegradation(double wall_s) const {
    if (recovering_) return {uw::domain::HealthReport::STATUS_RECOVERING, "recovering", false};

    const bool visual_live = VisualLive(wall_s);
    const bool sonar_live = SonarLive(wall_s);
    const bool vehicle_state_ok =
        last_vehicle_state_capture_s_.has_value() &&
        (wall_s - *last_vehicle_state_capture_s_) <= pipeline_config_.vehicle_state_stale_after_s;

    if (!visual_live && !sonar_live) {
      return {uw::domain::HealthReport::STATUS_UNAVAILABLE, "all_assist_unavailable", false};
    }
    if (!vehicle_state_ok) {
      return {uw::domain::HealthReport::STATUS_UNAVAILABLE, "vehicle_state_stale", false};
    }
    if (!sonar_live) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "sonar_unavailable", true};
    }
    if (!visual_live) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "visual_unavailable", true};
    }
    // 下面视觉排在声呐前面（此时两路都存活、且都是前端自报的降级）：这是**刻意
    // 设定的固定优先级**，不是代码顺序的巧合，与它上方那条优先级链一脉相承。
    //
    // 如果视觉和声呐前端同时报降级，呈现给飞手的首要原因取视觉那条；声呐那条并没有
    // 丢，它仍完整出现在 sensor_health() 里，只是没有被提升到 system_health()。
    if (last_visual_health_.has_value() &&
        last_visual_health_->status() != uw::domain::HealthReport::STATUS_HEALTHY) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, last_visual_health_->reason_code(), true};
    }
    if (last_sonar_health_.has_value() &&
        last_sonar_health_->status() != uw::domain::HealthReport::STATUS_HEALTHY) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, last_sonar_health_->reason_code(), true};
    }
    if (pipeline_config_.dense.enabled && !DenseCurrentlyFresh(wall_s)) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "dense_deadline_missed", true};
    }
    return {uw::domain::HealthReport::STATUS_HEALTHY, "", true};
  }

  // 每一次 OnImageFrame/OnSonarFrame/OnVehicleState/OnHealthReport 都会无条件走到
  // 这里——overload 档下（相机 1.25 倍 + 声呐 20Hz + 状态 100Hz）可达约 145 次/秒。
  //
  // 不限流的话，每一次都会在 AssistOutputSink::Publish 里驱动一次完整的 HMI 叠加层
  // 渲染 + JSON 状态重建，这些都不便宜——见 docs/archive/rov-realtime-closed-loop-
  // code-review-2026-08-27.md 的 C1 条。
  //
  // 注意限流的边界：内部跟踪状态（fusion_、pending_*、last_*_capture_s_）在调用方
  // 走到这里之前就已经更新过了，不受限流影响；被限的**只有**真正发往 `sink_` 的那次
  // 发布。
  void PublishNow(bool force = false) {
    const double wall_s = uw::domain::ToSeconds(now_());
    if (!force && pipeline_config_.min_publish_interval_s > 0.0 && last_publish_wall_s_.has_value() &&
        (wall_s - *last_publish_wall_s_) < pipeline_config_.min_publish_interval_s) {
      return;
    }
    last_publish_wall_s_ = wall_s;

    const auto decision = ComputeDegradation(wall_s);

    uw::domain::OperatorAssistState state;
    const auto track_set = fusion_->tracker().ToProtoSet(wall_s);
    if (track_set.has_value()) {
      *state.mutable_target_tracks() = *track_set;
      const bool any_fused = std::any_of(
          track_set->tracks().begin(), track_set->tracks().end(), [](const auto& track) {
            bool has_visual = false, has_sonar = false;
            for (const auto source : track.sources()) {
              has_visual |= (source == uw::domain::ASSIST_SOURCE_VISUAL);
              has_sonar |= (source == uw::domain::ASSIST_SOURCE_SONAR);
            }
            return has_visual && has_sonar;
          });
      if (any_fused) ++diagnostics_.fused_track_publish_count;
    }

    // 这里用"视觉当前是否存活"来把关，而不是只看 has_value()。
    //
    // 否则：相机掉线前最后一帧算出的路径偏移量会被永远重复发布，而它自己没有任何
    // 陈旧信号——guidance_valid 只依据航迹/车辆状态那几项判定，那几项对**这个字段**
    // 的时龄一无所知。飞手会照着一个早已过时的横向偏移去操舵。
    if (latest_path_lateral_offset_m_.has_value() && VisualLive(wall_s)) {
      state.set_has_path_lateral_offset(true);
      state.set_path_lateral_offset_m(*latest_path_lateral_offset_m_);
      if (latest_path_offset_sigma_m_.has_value()) {
        state.set_path_offset_sigma_m(*latest_path_offset_sigma_m_);
      }
    }

    uw::domain::HealthReport health;
    health.set_component_id("online_assist_pipeline");
    health.set_status(decision.status);
    health.set_reason_code(decision.reason);
    *state.mutable_system_health() = health;

    double newest_capture_s = -std::numeric_limits<double>::infinity();
    for (const auto& capture : {last_visual_capture_s_, last_sonar_capture_s_, last_vehicle_state_capture_s_}) {
      if (capture.has_value()) newest_capture_s = std::max(newest_capture_s, *capture);
    }
    state.set_data_age_ms(std::isfinite(newest_capture_s)
                              ? std::max(0.0, (wall_s - newest_capture_s) * 1000.0)
                              : 0.0);
    state.set_guidance_valid(decision.guidance_valid);
    state.set_degradation_reason(decision.reason);

    if (last_visual_health_.has_value()) *state.add_sensor_health() = *last_visual_health_;
    if (last_sonar_health_.has_value()) *state.add_sensor_health() = *last_sonar_health_;
    // 过期的外部健康报告直接丢弃，而不是一直重复发布：不同于本管线自己的视觉/声呐
    // 存活判定，一个不再发送的外部报告方没有任何别的信号能表明它最后那条报告已经
    // 过时了。
    for (const auto& [component_id, entry] : external_health_) {
      (void)component_id;
      if ((wall_s - entry.received_wall_s) <= pipeline_config_.modality_stale_after_s) {
        *state.add_sensor_health() = entry.report;
      }
    }

    sink_->Publish(state);
    ++diagnostics_.published_count;
  }

  uw::measurement_api::VisualAssistFrontend* visual_frontend_;
  uw::measurement_api::SonarFrontend* sonar_frontend_;
  DenseDepthProvider* dense_provider_;
  AssistOutputSink* sink_;

  uw::domain::RigCalibrationSnapshot rig_;
  uw::runtime::TargetAssociationConfig association_config_;
  uw::runtime::TargetTrackerConfig tracker_config_;
  uw::runtime::OnlineAssistPipelineConfig pipeline_config_;
  std::function<uw::domain::Stamp()> now_;

  uw::runtime::AcousticOpticBuffer buffer_;
  std::optional<uw::frontends::TargetFusionComponents> fusion_;
  uw::frontends::SonarTargetExtractor sonar_extractor_;

  std::vector<uw::frontends::SensorTargetDetection> pending_visual_;
  std::vector<uw::frontends::SensorTargetDetection> pending_sonar_;

  std::optional<double> last_visual_capture_s_;
  std::optional<double> last_sonar_capture_s_;
  std::optional<double> last_vehicle_state_capture_s_;
  std::optional<double> last_publish_wall_s_;

  std::optional<uw::domain::HealthReport> last_visual_health_;
  std::optional<uw::domain::HealthReport> last_sonar_health_;
  std::map<std::string, ExternalHealthEntry> external_health_;

  std::optional<double> latest_path_lateral_offset_m_;
  std::optional<double> latest_path_offset_sigma_m_;

  std::optional<uw::domain::OpticalDepthPriorMeasurement> pending_dense_depth_;
  std::optional<double> pending_dense_depth_capture_s_;
  // 防止稠密计算被重入派发。由于目前稠密工作是同步跑的（见 DenseDepthProvider 的
  // 注释），每次调用返回时它一定又是 false。保留它是为了将来换成异步实现时，派发这
  // 一侧不需要再动管线代码。
  bool dense_task_in_flight_ = false;

  bool recovering_ = false;

  OnlineAssistPipelineDiagnostics diagnostics_;
};

OnlineAssistPipeline::OnlineAssistPipeline(OnlineAssistPipelineDependencies deps)
    : impl_(std::make_unique<Impl>(std::move(deps))) {}
OnlineAssistPipeline::~OnlineAssistPipeline() = default;
OnlineAssistPipeline::OnlineAssistPipeline(OnlineAssistPipeline&&) noexcept = default;
OnlineAssistPipeline& OnlineAssistPipeline::operator=(OnlineAssistPipeline&&) noexcept = default;

bool OnlineAssistPipeline::OnImageFrame(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnImageFrame(event);
}
bool OnlineAssistPipeline::OnSonarFrame(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnSonarFrame(event);
}
bool OnlineAssistPipeline::OnImuSample(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnImuSample(event);
}
bool OnlineAssistPipeline::OnDvlSample(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnDvlSample(event);
}
bool OnlineAssistPipeline::OnVehicleState(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnVehicleState(event);
}
bool OnlineAssistPipeline::OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnKeyframeBoundary(event);
}
bool OnlineAssistPipeline::OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnMeasurementEvidence(event);
}
bool OnlineAssistPipeline::OnReferenceState(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnReferenceState(event);
}
bool OnlineAssistPipeline::OnHealthReport(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnHealthReport(event);
}
bool OnlineAssistPipeline::OnMapEvidence(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnMapEvidence(event);
}
bool OnlineAssistPipeline::Flush() { return impl_->Flush(); }

void OnlineAssistPipeline::UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
  impl_->UpdateRig(std::move(rig));
}

OnlineAssistPipelineDiagnostics OnlineAssistPipeline::Diagnostics() const {
  return impl_->Diagnostics();
}

}  // namespace uw::application
