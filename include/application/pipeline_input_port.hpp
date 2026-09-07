// 与"数据从哪来"解耦的 CanonicalEvent 接收端口。
//
// 离线 MCAP 回放和实时数据源（LiveEventSource）走的是同一条路：都由 PumpEvents
// （event_pump.hpp）把事件喂进同一个 PipelineInputPort 实现。正因为有这层端口，
// 算法代码才不需要知道自己的输入究竟来自 bag 文件还是仿真器实时话题。
//
// 硬性约束：实现类不得把 MCAP / ROS2 / 厂商 SDK 的类型从这个接口反向漏出去
// （见 docs/archive/superpowers/plans/2026-08-24-live-replay-unified-ingress.md
// 第 4/1.1 节）——一旦漏了，隔离就白做了。
//
// 所有 On* 的返回值语义统一：true = 继续读，false = 请事件源立刻停止。
#pragma once

#include "runtime/canonical_event.hpp"

namespace uw::application {

class PipelineInputPort {
 public:
  virtual ~PipelineInputPort() = default;

  virtual bool OnImageFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnImuSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnDvlSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnVehicleState(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) = 0;
  // 真值 / 仅供参考的数据（目前只有 /gt/state）。绝对不能被转手到任何在线算法
  // 读得到的地方——它只允许流向评测。
  virtual bool OnReferenceState(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnHealthReport(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) = 0;

  // 由 PumpEvents 恰好调用一次，且仅在底层 EventSource 正常读完
  // （EventSourceStatus::kCompleted）时调用。事件源打不开、或者被本端口自己的
  // On* 返回 false 提前叫停的情况下，都不会调到这里——所以 Flush 里可以安全地
  // 假设"我拿到的是一份完整的流"，做跨记录的收尾校验。
  virtual bool Flush() = 0;
};

}  // namespace uw::application
