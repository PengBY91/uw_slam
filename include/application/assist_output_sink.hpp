// 在线辅助结果的输出端口（application 层对"结果往哪送"的唯一抽象）。
//
// OnlineAssistPipeline 每算出一份 OperatorAssistState 就往这里 Publish 一次，至于
// 它最终是被叠加渲染到 HMI 画面上、发成 ROS2 话题、还是只在内存里留最新一份
// （LatestAssistSink），管线本身完全不需要知道。保持这个方向，才不会让 ROS2 之类
// 的具体传输方式从下游反向渗进 application 层。
#pragma once

#include "domain/domain.hpp"

namespace uw::application {

class AssistOutputSink {
 public:
  virtual ~AssistOutputSink() = default;
  virtual void Publish(const uw::domain::OperatorAssistState& state) = 0;
};

}  // namespace uw::application
