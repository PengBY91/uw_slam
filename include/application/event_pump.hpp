// 把 EventSource 里流出来的 CanonicalEvent 逐条派发给 PipelineInputPort。
//
// 这是全仓库唯一一处把"CanonicalPayload 这个 variant 当前是哪一路"翻译成
// "该调 PipelineInputPort 的哪个方法"的地方——调用方一律不自己写这个 switch。
// 好处是：新增一种 payload 类型时，编译期就能在这里的 std::visit 里发现遗漏，
// 而不是让每个 InputPort 实现各自漏判一次。
//
// 两条使用约定：
//   - 任一 On* 返回 false 会让 EventSource 提前停下（用于"读到坏数据就中止"）；
//   - 只有事件源正常读完（kCompleted）才会调一次 Flush()，中途失败或被叫停都不会。
#pragma once

#include "application/pipeline_input_port.hpp"
#include "runtime/event_source.hpp"

namespace uw::application {

uw::runtime::EventSourceReport PumpEvents(uw::runtime::EventSource& source, PipelineInputPort& input);

}  // namespace uw::application
