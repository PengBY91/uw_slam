// Regression coverage for docs/archive/rov-realtime-closed-loop-code-review-
// 2026-08-27.md finding A1: OnlineAssistPipelineDependencies::now must
// track HoloOcean's CLOCK_DOMAIN_SIMULATION capture times, not wall time
// directly, or every staleness check downstream sees an astronomical age.
// FakeClock-based pipeline tests (tests/application/online_assist_pipeline_
// test.cpp) cannot exercise this: their fixtures keep capture time and
// "now" synchronized by construction, exactly the assumption this
// estimator exists to stop production code from silently relying on.
#include "adapters/sim_wall_clock_estimator.hpp"

#include <gtest/gtest.h>

using uw::adapters::SimWallClockEstimator;
using uw::domain::CLOCK_DOMAIN_SIMULATION;
using uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME;
using uw::domain::ObservationHeader;

namespace {

class FakeWallClock {
 public:
  double operator()() const { return seconds_; }
  void Set(double seconds) { seconds_ = seconds; }

 private:
  double seconds_ = 0.0;
};

ObservationHeader MakeHeader(double capture_s, uw::domain::ClockDomain domain) {
  ObservationHeader header;
  *header.mutable_capture_time() = uw::domain::FromSeconds(capture_s);
  header.set_clock_domain(domain);
  return header;
}

TEST(SimWallClockEstimatorTest, FallsBackToWallTimeBeforeFirstObservation) {
  FakeWallClock wall;
  wall.Set(1'700'000'000.0);
  SimWallClockEstimator estimator([&wall] { return wall(); });

  EXPECT_NEAR(uw::domain::ToSeconds(estimator.EstimateNow()), 1'700'000'000.0, 1e-6);
}

TEST(SimWallClockEstimatorTest, ExtrapolatesSimTimeFromWallElapsedSinceAnchor) {
  FakeWallClock wall;
  wall.Set(1'000.0);
  SimWallClockEstimator estimator([&wall] { return wall(); });

  // HoloOcean sim clock starts near zero while wall clock is a large Unix
  // timestamp -- the exact mismatch this estimator exists to bridge.
  estimator.Observe(MakeHeader(100.0, CLOCK_DOMAIN_SIMULATION));

  wall.Set(1'005.0);  // 5s of wall time passes with no new observation.
  EXPECT_NEAR(uw::domain::ToSeconds(estimator.EstimateNow()), 105.0, 1e-6);
}

TEST(SimWallClockEstimatorTest, ReanchorsOnEachNewSimulationObservation) {
  FakeWallClock wall;
  wall.Set(1'000.0);
  SimWallClockEstimator estimator([&wall] { return wall(); });

  estimator.Observe(MakeHeader(100.0, CLOCK_DOMAIN_SIMULATION));
  wall.Set(1'010.0);
  estimator.Observe(MakeHeader(200.0, CLOCK_DOMAIN_SIMULATION));  // fresh anchor

  wall.Set(1'012.0);
  EXPECT_NEAR(uw::domain::ToSeconds(estimator.EstimateNow()), 202.0, 1e-6);
}

TEST(SimWallClockEstimatorTest, IgnoresObservationsOutsideSimulationClockDomain) {
  FakeWallClock wall;
  wall.Set(1'000.0);
  SimWallClockEstimator estimator([&wall] { return wall(); });

  estimator.Observe(MakeHeader(100.0, CLOCK_DOMAIN_SIMULATION));
  wall.Set(1'005.0);
  // A system-realtime-stamped header (e.g. a synthetic fixture) must not
  // perturb the simulation-domain anchor.
  estimator.Observe(MakeHeader(9'999.0, CLOCK_DOMAIN_SYSTEM_REALTIME));

  EXPECT_NEAR(uw::domain::ToSeconds(estimator.EstimateNow()), 105.0, 1e-6);
}

}  // namespace
