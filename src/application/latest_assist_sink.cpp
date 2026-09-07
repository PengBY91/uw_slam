// LatestAssistSink 的实现：一把 mutex + 一个 optional，写覆盖、读拷贝。
// 语义与取舍见头文件。
#include "application/latest_assist_sink.hpp"

namespace uw::application {

void LatestAssistSink::Publish(const uw::domain::OperatorAssistState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = state;
}

std::optional<uw::domain::OperatorAssistState> LatestAssistSink::Latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

}  // namespace uw::application
