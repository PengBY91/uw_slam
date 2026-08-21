// Per platform architecture section 12.1: three ORTHOGONAL state machines
// (system tracking state, per-modality health, mapping throttling). State
// transitions require a minimum hold time before another transition is
// accepted, specifically to avoid oscillation near a threshold (section
// 12.1: "避免权重在阈值附近震荡"). Mapping state must never directly decide
// localization state (section 12.1's explicit rule) — that is enforced by
// these being separate classes with no shared mutable state, not by a
// runtime check.
#pragma once

#include <chrono>
#include <string>

namespace uw::runtime {

using Clock = std::chrono::steady_clock;

enum class SystemState {
  kBoot,
  kWarmup,
  kInitializing,
  kTracking,
  kDegraded,
  kLost,
  kRelocalizing,
  kRecovering,
};

enum class ModalityHealthState {
  kHealthy,
  kSuspect,
  kUnavailable,
  kRecovering,
};

enum class MappingState {
  kFull,
  kThrottled,
  kKeyframeOnly,
  kPaused,
};

// Shared hold-time gate: State::Request(new_state, reason, now) is a no-op
// (returns false) if less than `min_hold` has elapsed since the last
// accepted transition, UNLESS `force` bypasses it (reserved for hard
// faults, e.g. a fatal sensor dropout that must not wait out a hold timer).
template <typename StateEnum>
class HysteresisStateMachine {
 public:
  HysteresisStateMachine(StateEnum initial, std::chrono::milliseconds min_hold)
      : state_(initial), min_hold_(min_hold), last_transition_(Clock::now()) {}

  StateEnum state() const { return state_; }
  const std::string& reason_code() const { return reason_code_; }
  Clock::time_point last_transition() const { return last_transition_; }

  bool Request(StateEnum new_state, std::string reason_code, bool force = false) {
    const auto now = Clock::now();
    if (new_state == state_) return false;
    if (!force && (now - last_transition_) < min_hold_) return false;
    state_ = new_state;
    reason_code_ = std::move(reason_code);
    last_transition_ = now;
    return true;
  }

 private:
  StateEnum state_;
  std::chrono::milliseconds min_hold_;
  Clock::time_point last_transition_;
  std::string reason_code_;
};

using SystemStateMachine = HysteresisStateMachine<SystemState>;
using ModalityHealthStateMachine = HysteresisStateMachine<ModalityHealthState>;
using MappingStateMachine = HysteresisStateMachine<MappingState>;

}  // namespace uw::runtime
