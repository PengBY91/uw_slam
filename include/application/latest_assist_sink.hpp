// "只留最新一份"的 AssistOutputSink 实现，供在线飞手辅助管线使用。
//
// Publish 永远是覆盖，不是入队：下游消费者（画面叠加渲染器、以后可能的 ROS2
// 发布器）只关心当前最新的一条引导信息，攒一堆过期的反而有害——飞手看到的必须
// 是"现在"的态势。所以这里刻意不做缓冲队列。
//
// Publish 和 Latest 会被不同线程调用（管线线程写、渲染/发布线程读），因此用一把
// mutex 保护；Latest 返回的是拷贝而不是引用，调用方拿到手之后就与后续覆盖无关。
#pragma once

#include <mutex>
#include <optional>

#include "application/assist_output_sink.hpp"
#include "domain/domain.hpp"

namespace uw::application {

class LatestAssistSink final : public AssistOutputSink {
 public:
  void Publish(const uw::domain::OperatorAssistState& state) override;
  std::optional<uw::domain::OperatorAssistState> Latest() const;

 private:
  mutable std::mutex mutex_;
  std::optional<uw::domain::OperatorAssistState> latest_;
};

}  // namespace uw::application
