// uw::adapters::HoloOceanRealtimeSink 的具体实现（这条接缝为什么存在，见那个头文件
// 的注释）。
//
// 本文件是 HoloOcean 实时闭环里**唯一真正持有** LiveEventSource + OnlineAssistPipeline
// 的地方。它刻意不 include 任何 ROS2 头，所以
// tools/lint/check_layer_dependencies.py 的 ROS 厂商检查不会命中它；作为交换，它可以
// 自由依赖 `application` 角色所允许的整套 application/runtime/opencv_adapters/frontends
// 栈。ROS2 那一侧只看到 HoloOceanRealtimeSink 这个纯虚接口。
//
// ============================ 本文件主要逻辑 ============================
//
// 三个类，各管一段：
//   - RealtimeAssistOutputSink：辅助结果的出口。把最新飞手图 + 最新声呐帧与
//     OperatorAssistState 合成叠加画面交给 output_，同时拼状态 JSON、喂运行期指标、
//     按需落盘 run report。
//   - ForwardingPort：一层极薄的 PipelineInputPort 转发，直通 OnlineAssistPipeline。
//   - OnlineAssistRealtimeSink：把上面这些和 LiveEventSource 装到一起，并起一条泵
//     线程跑 PumpEvents。
//
// 数据流：ROS2 回调线程调 On*Camera/OnSonar/OnVehicleState -> Submit() 投进
// LiveEventSource 的有界队列 -> 泵线程从队列取出、经 ForwardingPort 进管线 ->
// 管线算完回调 RealtimeAssistOutputSink::Publish -> 出到 ROS2。**跨线程的交接点只有
// 队列这一处**，这是整个设计里最要紧的一条。
//
// 两处容易被忽略但很关键的时间处理：
//   1. 这条链路上所有 capture_time 都是 CLOCK_DOMAIN_SIMULATION，不是墙钟。所以
//      deps.now 接的是 sim_clock_ 而不是 system_clock（详见构造函数里的注释）。
//   2. 飞手相机是**纯呈现路径**，绝不进管线（见 OnPilotCamera）。
// =======================================================================
#include "adapters/holoocean_realtime_sink.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>

#include "adapters/opencv_visual_assist_frontend.hpp"
#include "adapters/operator_overlay_renderer.hpp"
#include "adapters/sim_wall_clock_estimator.hpp"
#include "application/assist_output_sink.hpp"
#include "application/event_pump.hpp"
#include "application/holoocean_status_json.hpp"
#include "application/online_assist_pipeline.hpp"
#include "application/pipeline_input_port.hpp"
#include "application/replay_pipeline.hpp"
#include "application/runtime_metrics_collector.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/live_event_source.hpp"

namespace uw::adapters {
namespace {

uw::domain::RigCalibrationSnapshot ResolveRig(const HoloOceanRealtimeSinkConfig& config) {
  if (config.rig_config_path.empty()) {
    std::cerr << "holoocean_realtime_sink: WARNING no rig_config_path parameter given -- using "
                 "a placeholder identity-extrinsic rig. Every bearing/range projection is "
                 "geometrically wrong until a real calibrated rig is supplied; this run must "
                 "not be treated as real-machine acceptance evidence (FUS-CAL-001).\n";
    return config.fallback_rig;
  }
  // 这里刻意**不捕获**异常：既然操作者明确指定了 rig_config_path，那加载失败就应该
  // 让节点起不来，而不是悄悄退回上面那份（错误的）占位 rig。静默回退会让人以为自己
  // 用的是真标定，跑出一堆几何上完全错误却看起来正常的结果。
  return uw::runtime::LoadRigConfig(config.rig_config_path);
}

// 与上面 ResolveRig 同一套策略，只不过管的是声呐 CFAR/聚类 + 目标关联/跟踪器 +
// 降级时序这些参数（FUS-AC-002）：路径为空 -> 用硬编码的结构体默认值并大声警告；
// 路径给了但加载失败 -> 直接报错，绝不静默回退。
uw::runtime::PlatformDefaultsConfig ResolvePlatformDefaults(const HoloOceanRealtimeSinkConfig& config) {
  if (config.platform_config_path.empty()) {
    std::cerr << "holoocean_realtime_sink: WARNING no platform_config_path parameter given -- "
                 "using hardcoded C++ defaults for sonar CFAR/clustering, target association/"
                 "tracker gates and degradation timing instead of a versioned config file "
                 "(FUS-AC-002). Editing configs/defaults/platform.yaml has no effect on this "
                 "run until platform_config_path is set.\n";
    return uw::runtime::PlatformDefaultsConfig{};
  }
  return uw::runtime::LoadPlatformDefaultsConfig(config.platform_config_path);
}

int64_t SteadyNowNs() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count < 0 ? 0 : count;
}

double WallNowSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// RuntimeMetricsConfig::queue_lane_capacities 必须与下面 source_ 自己的
// LaneQueueConfig::capacity 保持一致（顺序是 localization/correction/mapping/evidence，
// 即 LiveEventSource::HealthReports() 文档承诺的固定顺序）。
//
// 由于 source_ 一律用 LiveSourceConfig{} 的默认值构造，这里就直接硬编码成相同的值，
// 而不是再把同一份配置穿两遍。**代价是改了那边要记得同步改这里。**
uw::application::RuntimeMetricsConfig MakeMetricsConfig(const HoloOceanRealtimeSinkConfig& config) {
  uw::application::RuntimeMetricsConfig metrics_config;
  metrics_config.deadline_ms = config.deadline_ms;
  metrics_config.queue_lane_capacities = {64, 32, 16, 256};
  return metrics_config;
}

// 面向 HoloOceanRealtimeOutput 的 AssistOutputSink 实现。
//
// 它通过 OperatorOverlayRenderer 把"最新飞手图 + 最新声呐帧"（都是只留最新、不做
// 无界缓冲）与 OperatorAssistState 合成，然后把结果交给 `output`——也就是 ROS2 节点
// 本身，由它发布 sensor_msgs/Image + std_msgs/String。
class RealtimeAssistOutputSink final : public uw::application::AssistOutputSink {
 public:
  // `source` 和 `metrics` 的生命周期长于本 sink：OnlineAssistRealtimeSink 里
  // source_ 和 metrics_ 都声明在 output_sink_ 之前，所以走到这里时它们已经构造完毕；
  // 析构则按声明的逆序进行（output_sink_ 先走）。这条依赖的是**成员声明顺序**，
  // 调整成员顺序时要留意。
  //
  // `pipeline` 此刻还拿不到：它是外层构造函数**函数体**里才建的 unique_ptr，而那时
  // output_sink_ 早已作为 deps.sink 存在了。所以留了下面的 SetPipeline，等外层拿到
  // 指针后再回填。
  RealtimeAssistOutputSink(HoloOceanRealtimeOutput& output, const uw::runtime::LiveEventSource& source,
                            uw::application::RuntimeMetricsCollector& metrics, std::string run_report_path)
      : output_(output), source_(source), metrics_(metrics), run_report_path_(std::move(run_report_path)) {}

  void Publish(const uw::domain::OperatorAssistState& state) override {
    std::optional<uw::domain::ImageFrame> pilot_image;
    std::optional<uw::domain::SonarFrame> sonar_frame;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      pilot_image = latest_pilot_image_;
      sonar_frame = latest_sonar_frame_;
    }
    if (pilot_image.has_value()) {
      const auto overlay = renderer_.Render(*pilot_image, sonar_frame, state);
      if (overlay.has_value()) output_.PublishOverlay(*overlay);
    }
    const auto queue_health = source_.HealthReports();
    output_.PublishStatus(uw::application::BuildOnlineAssistStatusJson(state, queue_health));

    // Publish() 只会在泵线程内部、由 OnlineAssistPipeline::PublishNow() 同步调用
    // （见那个类里的 sink_->Publish(state)）。而那正是拥有并修改 pipeline_ 的
    // diagnostics_ 的同一条线程，所以这里通过 SetPipeline 存下的裸指针去读诊断，
    // 除了 metrics_ 自己那把 mutex 之外不需要额外同步。
    //
    // 换句话说这条无锁读法**依赖"Publish 只在泵线程被调"这个前提**——哪天有别的线程
    // 也来调 Publish，这里就得改。
    const double wall_now_s = WallNowSeconds();
    metrics_.ObservePublish(state, wall_now_s);
    metrics_.ObserveQueueHealth(queue_health);
    if (pipeline_ != nullptr) metrics_.ObserveDiagnostics(pipeline_->Diagnostics());
    metrics_.SampleResourceUsage(wall_now_s - process_start_wall_s_);
    MaybeWriteReport(wall_now_s);
  }

  void SetLatestPilotImage(uw::domain::ImageFrame image) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_pilot_image_ = std::move(image);
  }

  void SetLatestSonarFrame(uw::domain::SonarFrame frame) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_sonar_frame_ = std::move(frame);
  }

  void SetPipeline(const uw::application::OnlineAssistPipeline& pipeline) { pipeline_ = &pipeline; }

 private:
  void MaybeWriteReport(double wall_now_s) {
    if (run_report_path_.empty()) return;
    // 直接搭 Publish() 自己的限流节奏（C1 的 min_publish_interval_s，默认 100ms），
    // 而不另起一条写文件的线程——见 HoloOceanRealtimeSinkConfig::run_report_path 的
    // 注释。
    //
    // 这里再加一道 >=1s 的闸，是为了把真正的落盘动作（开 fstream + 完整序列化 JSON）
    // 挡在常规发布路径之外：realtime_gate.py 只需要一份新鲜度在一两秒内的报告，不需要
    // 每次限流发布都重写一遍。
    if (last_report_write_wall_s_.has_value() && wall_now_s - *last_report_write_wall_s_ < 1.0) return;
    last_report_write_wall_s_ = wall_now_s;
    std::ofstream out(run_report_path_, std::ios::trunc);
    if (!out.is_open()) {
      std::cerr << "holoocean_realtime_sink: WARNING failed to open run_report_path '" << run_report_path_
                << "' for writing\n";
      return;
    }
    out << metrics_.BuildReportJson();
  }

  HoloOceanRealtimeOutput& output_;
  const uw::runtime::LiveEventSource& source_;
  uw::application::RuntimeMetricsCollector& metrics_;
  std::string run_report_path_;
  const uw::application::OnlineAssistPipeline* pipeline_ = nullptr;
  const double process_start_wall_s_ = WallNowSeconds();
  std::optional<double> last_report_write_wall_s_;
  uw::opencv_adapters::OperatorOverlayRenderer renderer_;
  std::mutex latest_mutex_;
  std::optional<uw::domain::ImageFrame> latest_pilot_image_;
  std::optional<uw::domain::SonarFrame> latest_sonar_frame_;
};

// 直通到真实 OnlineAssistPipeline 的 PipelineInputPort 转发器。
//
// 角色与 apps/online_assist_smoke.cpp 里的 ReferenceCountingPort 相同，只是少了那个
// 真值投递计数器——本 sink 永远不会收到真值面的事件，因为持有它的 ROS2 网关根本不
// 订阅 /uw/sim/ground_truth。
class ForwardingPort final : public uw::application::PipelineInputPort {
 public:
  explicit ForwardingPort(uw::application::OnlineAssistPipeline& pipeline) : pipeline_(pipeline) {}

  bool OnImageFrame(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnImageFrame(e); }
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnSonarFrame(e); }
  bool OnImuSample(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnImuSample(e); }
  bool OnDvlSample(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnDvlSample(e); }
  bool OnVehicleState(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnVehicleState(e); }
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnKeyframeBoundary(e);
  }
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnMeasurementEvidence(e);
  }
  bool OnReferenceState(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnReferenceState(e);
  }
  bool OnHealthReport(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnHealthReport(e); }
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnMapEvidence(e); }
  bool Flush() override { return pipeline_.Flush(); }

 private:
  uw::application::OnlineAssistPipeline& pipeline_;
};

class OnlineAssistRealtimeSink final : public HoloOceanRealtimeSink {
 public:
  OnlineAssistRealtimeSink(HoloOceanRealtimeOutput& output, HoloOceanRealtimeSinkConfig config)
      : platform_defaults_(ResolvePlatformDefaults(config)),
        source_(uw::runtime::LiveSourceConfig{}),
        visual_frontend_((uw::opencv_adapters::VisualAssistParams{})),
        sonar_frontend_(uw::application::BuildSonarCfarFrontendParams(platform_defaults_.sonar_frontend)),
        metrics_(MakeMetricsConfig(config)),
        output_sink_(output, source_, metrics_, config.run_report_path) {
    uw::application::OnlineAssistPipelineDependencies deps;
    deps.rig = ResolveRig(config);
    deps.visual_frontend = &visual_frontend_;
    deps.sonar_frontend = &sonar_frontend_;
    deps.dense_depth_provider = nullptr;  // 稠密深度保持关闭——与本仓库目前所有 app 一致
    deps.target_association = platform_defaults_.target_association;
    deps.target_tracker = platform_defaults_.target_tracker;
    deps.pipeline = platform_defaults_.online_assist;
    deps.pipeline.dense.enabled = false;
    deps.sink = &output_sink_;
    // 经这个 sink 进来的每一条传感器 header，其 capture_time 都是
    // CLOCK_DOMAIN_SIMULATION（见 holoocean_live_conversion.cpp），不是墙钟时间。
    //
    // 如果这里把 `now` 直接接到 system_clock::now()，那么所有陈旧 / 降级判定都会拿
    // 一个 ~0 秒的仿真时间戳去减一个 ~1.7e9 秒的 Unix 时间戳——从第一个 tick 起就永久
    // 报"不可用"。sim_clock_ 就是用来桥接这两个时间域的，见
    // include/adapters/sim_wall_clock_estimator.hpp。
    deps.now = [this] { return sim_clock_.EstimateNow(); };
    pipeline_ = std::make_unique<uw::application::OnlineAssistPipeline>(std::move(deps));
    output_sink_.SetPipeline(*pipeline_);
    port_ = std::make_unique<ForwardingPort>(*pipeline_);

    pump_thread_ = std::thread([this] {
      try {
        report_promise_.set_value(uw::application::PumpEvents(source_, *port_));
      } catch (const std::exception& error) {
        // 目前没有任何地方调 report_promise_.get_future()，所以异常如果丢在这里，
        // 就等于泵线程无声无息地死了——节点的辅助输出停止更新，却没有任何地方给出
        // 信号。至少把它打出来，让节点自己的 stderr/日志里看得见。
        //
        // 用 std::cerr 而不是 rclcpp 的 logger，是因为本类刻意不依赖 ROS（见文件头）。
        std::cerr << "holoocean_realtime_sink: PumpEvents thread terminated: " << error.what() << '\n';
        source_.Close();
        try {
          report_promise_.set_exception(std::current_exception());
        } catch (...) {
        }
      } catch (...) {
        std::cerr << "holoocean_realtime_sink: PumpEvents thread terminated: unknown exception\n";
        source_.Close();
        try {
          report_promise_.set_exception(std::current_exception());
        } catch (...) {
        }
      }
    });
  }

  ~OnlineAssistRealtimeSink() override {
    source_.Close();
    if (pump_thread_.joinable()) pump_thread_.join();
  }

  void OnLeftCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraLeft, std::move(frame));
  }
  void OnRightCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraRight, std::move(frame));
  }
  void OnMainCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraMain, std::move(frame));
  }
  void OnPilotCamera(uw::domain::ImageFrame frame) override {
    // 纯呈现用途：只缓存给叠加合成器，**绝不**投进 LiveEventSource / 
    // OnlineAssistPipeline。这是计划里明确要求的"飞手通路独立"——飞手看到的画面不能
    // 因为算法管线阻塞或降级而卡住。
    output_sink_.SetLatestPilotImage(std::move(frame));
  }
  void OnSonar(uw::domain::SonarFrame frame) override {
    output_sink_.SetLatestSonarFrame(frame);
    Submit(uw::runtime::kTopicSonarFrame, std::move(frame));
  }
  void OnVehicleState(uw::domain::VehicleState state) override {
    Submit(uw::runtime::kTopicVehicleState, std::move(state));
  }

 private:
  template <typename Payload>
  void Submit(const char* topic, Payload payload) {
    // header 是指向 payload 内部的引用——所有要从它读的东西，都必须在下面 payload 被
    // move 进 source_.Submit **之前**读完，否则就是读已被移走的对象。
    const auto& header = payload.header();
    if (header.clock_domain() == uw::domain::CLOCK_DOMAIN_SIMULATION) {
      // RTF 要的是**原始**的 (仿真时间, 墙钟时间) 这一对，而不是 sim_clock_ 自己
      // 锚定/外推出来的估计值。把 EstimateNow() 再喂回去，量到的只会是"我们当初假设
      // 的 RTF 有多接近 1"，而不是真实比值——自己证明自己。
      metrics_.ObserveSimTime(uw::domain::ToSeconds(header.capture_time()), WallNowSeconds());
      if constexpr (std::is_same_v<Payload, uw::domain::VehicleState>) {
        // 用的是**这条消息更新锚点之前**的锚（更新发生在下面的
        // sim_clock_.Observe(header)）。如果放到之后再调 EstimateNow()，那这条消息
        // 自己就成了锚点，算出来的时龄恒等于约 0——毫无意义的自证。
        metrics_.ObserveVehicleState(header, uw::domain::ToSeconds(sim_clock_.EstimateNow()));
      }
    }
    // 在这条消息被 move 进队列之前，用它自己的 capture_time 给 sim_clock_ 打锚。
    // 收进来的这一刻正是能配出一对新鲜的 (仿真采集时刻, 墙钟收到时刻) 样本的时机；
    // 放在这里而不是放到之后处理该事件的泵线程上，是为了让锚点尽可能贴近当前——
    // 事件在队列里排多久是不确定的，泵线程那边打锚会把排队延迟算进去。
    sim_clock_.Observe(header);
    const auto status = source_.Submit(
        {topic, static_cast<uint64_t>(SteadyNowNs()), ++source_sequence_, std::move(payload)});
    if (status == uw::runtime::LiveSubmitStatus::kClosed) {
      // 节点正在关闭（source_.Close() 已经调过），没什么可做的。其余状态都是正常的
      // 接受/降级结果，LiveEventSource 自己的统计里已经记着了，这里不必重复处理。
      return;
    }
  }

  uw::adapters::SimWallClockEstimator sim_clock_;
  // 必须声明在 sonar_frontend_ 之前：成员初始化顺序看的是**声明顺序**，不是初始化
  // 列表里的书写顺序，而 sonar_frontend_ 的初始化式要读 platform_defaults_.sonar_frontend。
  // 顺序写反就会读到未初始化的对象。
  uw::runtime::PlatformDefaultsConfig platform_defaults_;
  uw::runtime::LiveEventSource source_;
  uw::opencv_adapters::OpenCvVisualAssistFrontend visual_frontend_;
  uw::frontends::SonarCfarFrontend sonar_frontend_;
  // 声明在 output_sink_ 之前——后者的构造函数要拿它的引用（与上面 source_ 同一套
  // 既定做法）。
  uw::application::RuntimeMetricsCollector metrics_;
  RealtimeAssistOutputSink output_sink_;
  std::unique_ptr<uw::application::OnlineAssistPipeline> pipeline_;
  std::unique_ptr<ForwardingPort> port_;
  std::atomic<uint64_t> source_sequence_{0};
  std::thread pump_thread_;
  std::promise<uw::runtime::EventSourceReport> report_promise_;
};

}  // namespace

std::unique_ptr<HoloOceanRealtimeSink> MakeOnlineAssistRealtimeSink(
    HoloOceanRealtimeOutput& output, HoloOceanRealtimeSinkConfig config) {
  return std::make_unique<OnlineAssistRealtimeSink>(output, std::move(config));
}

}  // namespace uw::adapters
