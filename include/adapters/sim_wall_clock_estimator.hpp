// Bridges HoloOcean's CLOCK_DOMAIN_SIMULATION capture timestamps into the
// "now" domain OnlineAssistPipelineDependencies::now documents it expects:
// "the same domain Stamp frame as every sensor header's capture_time" (see
// include/application/online_assist_pipeline.hpp). Wiring deps.now straight
// to system_clock::now() violates that contract whenever capture_time is
// simulation time -- HoloOcean's sim clock starts near zero each episode
// while wall time is ~1.7e9s, so every staleness check would see an
// astronomical age and the pipeline would report itself permanently
// STATUS_UNAVAILABLE. See docs/archive/rov-realtime-closed-loop-code-review-
// 2026-08-27.md finding A1.
#pragma once

#include <functional>
#include <mutex>

#include "domain/domain.hpp"

namespace uw::adapters {

// Tracks an anchor (last observed simulation capture time, wall time at
// that observation) and extrapolates "current simulation time" from it
// using wall-clock elapsed time -- i.e. assumes the sim clock advances at
// the same rate as the wall clock between anchors (RTF ~= 1, which
// SIM-TIME-005 requires at nominal load anyway). Re-anchoring on every
// observed simulation-domain header keeps drift bounded to "how much the
// two clocks could plausibly diverge since the last message on any
// modality", not unboundedly -- the highest-rate input (vehicle state,
// 50-100 Hz nominal/overload) keeps this tight in practice.
//
// Thread-safe: Observe() is expected to run on the ingestion thread/callback
// that first sees each sensor message, EstimateNow() on whatever thread
// later asks the pipeline for its degradation/staleness state.
class SimWallClockEstimator {
 public:
  // wall_now_s defaults to system_clock, matching the domain Stamp epoch
  // used elsewhere in this codebase (uw::domain::ToStamp/FromSeconds);
  // tests inject a fake to control wall-clock advancement deterministically.
  explicit SimWallClockEstimator(std::function<double()> wall_now_s = nullptr);

  // Re-anchors using one observation's capture_time, but only if its
  // clock_domain is CLOCK_DOMAIN_SIMULATION -- a header stamped in any other
  // domain (e.g. a synthetic/replay fixture using system-realtime) must not
  // perturb this estimator, since mixing domains into one anchor would
  // silently corrupt every subsequent estimate.
  void Observe(const uw::domain::ObservationHeader& header);

  // Current best estimate of "now" in the simulation clock domain. Before
  // the first Observe(), falls back to wall time -- harmless, since every
  // pipeline staleness check that consults this also gates on its own
  // last-capture bookkeeping being unset at that point.
  uw::domain::Stamp EstimateNow() const;

 private:
  std::function<double()> wall_now_s_;
  mutable std::mutex mutex_;
  double anchor_sim_s_ = 0.0;
  double anchor_wall_s_ = 0.0;
  bool has_anchor_ = false;
};

}  // namespace uw::adapters
