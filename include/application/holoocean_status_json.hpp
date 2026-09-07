// /uw/hmi/status 话题载荷的 JSON 格式化（面向 ROS 的调用方是
// holoocean_realtime_sink.cpp 里的 RealtimeAssistOutputSink）。
//
// 单独拆成一个文件的原因很实际：这段纯拼字符串的逻辑原本藏在
// holoocean_realtime_sink.cpp 的匿名 namespace 里，**测试覆盖为零**——拆出来才能
// 直接写单测。它只依赖 domain 类型，不碰 runtime / OpenCV / ROS2，所以单测里不需要
// 拉起任何环境。
//
// queue_health 这一节的由来见 docs/archive/rov-realtime-closed-loop-code-review-
// 2026-08-27.md 的 B2 条：LiveEventSource::HealthReports() 早就算出了每条车道的
// 背压 / 丢帧统计，但此前没有任何地方把它呈现给飞手——飞手只能等数据陈旧到下游
// 报 staleness 才间接察觉，这不满足 FUS-Q-002 / FUS-RT-001 的可观测性要求。
#pragma once

#include <array>
#include <string>

#include "domain/domain.hpp"

namespace uw::application {

// 顺序是固定的，与 LiveEventSource::HealthReports() 自己文档里承诺的返回顺序一致：
// localization、correction、mapping、evidence。调用方按下标对号入座。
std::string BuildOnlineAssistStatusJson(const uw::domain::OperatorAssistState& state,
                                        const std::array<uw::domain::HealthReport, 4>& queue_health);

}  // namespace uw::application
