// 在线声光目标辅助这一条链路的装配处：图像 / 声呐 / 车辆状态经规范的
// PipelineInputPort 进来，带来源标记的 TargetTracks 和一份由 HealthReport 支撑的
// 降级说明经"只留最新"的 AssistOutputSink 出去。见 docs/archive/superpowers/plans/
// 2026-08-24-acoustic-optic-online-tracking.md Task 6。
//
// 一条核心设计：**视觉与声呐的目标检测彼此完全独立地跑**。声呐掉线不会阻止纯视觉
// 航迹继续发布，反之亦然——这种在单侧失效下继续工作的能力，正是一个"降级模式辅助
// 系统"存在的意义。如果做成必须两路都齐才出结果，那它在最需要它的时候恰好不可用。
//
// 唯一被卡在 AcousticOpticBuffer"立体 + 声呐 + 状态完全同步"这个 bundle 后面的路径，
// 是本地稠密立体深度。因为只有这项计算真的贵到需要做实时预算决策，见下面的
// DenseDepthProvider。
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "application/assist_output_sink.hpp"
#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "measurement_api/frontend.hpp"
#include "measurement_api/target_frontend.hpp"
#include "runtime/config.hpp"

namespace uw::application {

// 可注入的本地稠密立体深度补全接口。
//
// 生产实现是拿 frontends::StereoOpticalDepthFrontend 外面套一层墙钟预算检查（见下面
// 的 StereoBlockMatchDenseDepthProvider）；测试里注入一个假实现，就能确定性地覆盖
// 管线的预算 / 质量 / 在途 三种门控，而不必真的付出块匹配的计算代价。
//
// RunBounded 返回 nullopt 涵盖了管线视为"本周期没有可用深度"的**全部**结果——质量
// 不合格、超预算、或者干脆失败。它们对外一律呈现为同一个 reason_code
// "dense_deadline_missed"：面向飞手的词汇表不区分稠密深度**为什么**没交付，只区分
// 交没交付。飞手需要的是"现在能不能信这个深度"，不是失败分类学。
class DenseDepthProvider {
 public:
  virtual ~DenseDepthProvider() = default;
  virtual std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) = 0;
};

// 生产默认的稠密深度实现：frontends::StereoOpticalDepthFrontend + 一次事后的墙钟
// 预算检查。
//
// 注意是**事后**：块匹配本身不可抢占，没法中途叫停，所以超预算只能在算完之后才发现，
// 然后把结果丢掉。也就是说这个预算保证的是"不会把超时结果拿去用"，而不是"不会占用
// 超时的 CPU"。
class StereoBlockMatchDenseDepthProvider final : public DenseDepthProvider {
 public:
  explicit StereoBlockMatchDenseDepthProvider(uw::runtime::StereoMatchingConfig matcher_config);
  ~StereoBlockMatchDenseDepthProvider() override;

  std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

// 除了几个纯配置 / 数据值，其余字段都是调用方持有的非拥有指针——OnlineAssistPipeline
// 不接管 visual_frontend / sonar_frontend / dense_depth_provider / sink 的生命周期，
// 与本仓库别处注入前端的做法一致（参考 AcousticOpticDepthFusionFrontend 的调用方）。
//
// dense_depth_provider 允许为空：此时即便 dense.enabled 为 true，稠密深度也永远不会
// 跑，效果等同于"每次本该触发时都恒定地 dense_deadline_missed"。
struct OnlineAssistPipelineDependencies {
  uw::measurement_api::VisualAssistFrontend* visual_frontend = nullptr;
  uw::measurement_api::SonarFrontend* sonar_frontend = nullptr;
  DenseDepthProvider* dense_depth_provider = nullptr;
  AssistOutputSink* sink = nullptr;

  uw::domain::RigCalibrationSnapshot rig;
  uw::runtime::AcousticOpticBufferConfig buffer;
  uw::runtime::TargetAssociationConfig target_association;
  uw::runtime::TargetTrackerConfig target_tracker;
  uw::runtime::OnlineAssistPipelineConfig pipeline;

  // 用于 data_age_ms / 陈旧判定的墙钟，与所有传感器 header 的 capture_time 处在同一个
  // domain Stamp 时间系里。
  //
  // **不是 steady_clock**：这里要跟传感器时间戳相减，必须同一个时间系。稠密深度自己的
  // 预算计时另用 steady_clock，因为那量的是真实 CPU 耗时，跟传感器时间无关——两者是
  // 两回事，别混用。
  //
  // 默认取 system_clock；测试注入一个 FakeClock 闭包，与它们喂进来的 capture_time
  // 同步推进。
  std::function<uw::domain::Stamp()> now;
};

struct OnlineAssistPipelineDiagnostics {
  uint64_t published_count = 0;
  uint64_t dense_attempt_count = 0;
  uint64_t dense_deadline_missed_count = 0;
  uint64_t calibration_reset_count = 0;
  // 每当某一路（视觉或声呐）的检测在距离**它自己**上一次检测超过
  // modality_stale_after_s 之后才到达，就 +1（FUS-HEALTH-002）。
  //
  // 它与随后那个 "recovering" 状态多快被清掉无关（那取决于跟踪器重新确认的时机，从
  // 外部并不可靠可观测）。正因为如此，这个计数器是唯一能可靠证明"掉线+恢复这个触发
  // 确实发生过"的东西。
  uint64_t modality_recovery_count = 0;
  // TargetTracker::Update 是整批原子的：输入里出现非有限值、乱序、或者已经被接受过的
  // observation_id，就整批拒绝、不做任何改动。
  //
  // 从 FlushAssociation 的返回值上看这是一次静默的空操作，所以这个计数器是唯一能看出
  // "曾经有关联批次被这样丢掉"的信号。
  uint64_t association_reject_count = 0;

  // 前端原始输出的累计计数（对应 SIM-ACC-002 / FUS-ACC-001：一次验收运行必须拿出
  // 非零的声呐/视觉检测数和非零的融合航迹，而不是只证明"进程没崩"）。
  //
  // 统计点选在**前端输出边界**（RunSonarDetection / RunVisualDetection 各自 `targets`
  // 的每个元素），而不是从已发布的航迹状态反推。所以一个最终没能关联进任何航迹的
  // 检测，在这里照样计入——这才反映前端真实的产出能力。
  uint64_t sonar_detection_count = 0;
  uint64_t visual_detection_count = 0;
  // 导出的航迹集合中"至少存在一条同时带 ASSIST_SOURCE_VISUAL 和 ASSIST_SOURCE_SONAR
  // 标记"的 PublishNow() 调用次数。
  //
  // 它是"确实出现过真正的声光融合航迹"的一个廉价近似指标，**不是**不同融合航迹的
  // 条数——同一条航迹只要还处于融合状态，每次发布都会被重复计一次。之所以这样够用：
  // 下游验收门禁实际只检查它是否非零。
  uint64_t fused_track_publish_count = 0;
};

class OnlineAssistPipeline final : public PipelineInputPort {
 public:
  explicit OnlineAssistPipeline(OnlineAssistPipelineDependencies deps);
  ~OnlineAssistPipeline() override;
  OnlineAssistPipeline(OnlineAssistPipeline&&) noexcept;
  OnlineAssistPipeline& operator=(OnlineAssistPipeline&&) noexcept;
  OnlineAssistPipeline(const OnlineAssistPipeline&) = delete;
  OnlineAssistPipeline& operator=(const OnlineAssistPipeline&) = delete;

  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnImuSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnDvlSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnVehicleState(const uw::runtime::CanonicalEvent& event) override;
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) override;
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool OnReferenceState(const uw::runtime::CanonicalEvent& event) override;
  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) override;
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool Flush() override;

  // 显式的标定变更入口（rig 标定本身不是 CanonicalEvent，见 include/runtime/config.hpp
  // 的 rig 层）。
  //
  // 它会重置关联器 / 跟踪器和 AcousticOpticBuffer、清掉在途的稠密计算，并立即发布一个
  // "recovering" 状态。在新标定下重新确认（CONFIRMED）出一条航迹之前，引导一直是无效的
  // ——旧标定下建立的航迹在新外参下不再可信，继续用比没有更危险。
  void UpdateRig(uw::domain::RigCalibrationSnapshot rig);

  OnlineAssistPipelineDiagnostics Diagnostics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uw::application
