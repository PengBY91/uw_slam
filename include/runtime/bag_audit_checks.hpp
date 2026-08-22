// Pure-logic checks for apps/bag_audit.cpp, separated from I/O so they are
// unit-testable without a real MCAP file — matching this repo's general
// pattern of keeping I/O glue
// thin in apps/ and logic in a library (see e.g. evaluation/trajectory_
// metrics.hpp's pure ComputeAte vs. apps/replay_demo.cpp's I/O around it).
#pragma once

#include <cstdint>
#include <limits>
#include <set>
#include <string>

#include "domain/domain.hpp"

namespace uw::runtime {

int64_t StampToNanos(const uw::domain::Stamp& s);
bool StampIsZero(const uw::domain::Stamp& s);

// Accumulated findings for one topic that carries a full ObservationHeader
// (ImageFrame/SonarFrame/ImuSample/DvlSample — NOT StateSnapshot, which has
// only a bare capture_timestamp, or MeasurementEvidence, which has no
// timestamp/frame field at all; see apps/bag_audit.cpp's header comment for
// why those two are handled separately).
struct HeaderStats {
  uint64_t count = 0;
  bool capture_time_ever_populated = false;
  bool receive_time_ever_populated = false;
  bool capture_time_monotonic = true;
  bool receive_time_monotonic = true;
  int64_t last_capture_ns = std::numeric_limits<int64_t>::min();
  int64_t last_receive_ns = std::numeric_limits<int64_t>::min();
  std::set<int> clock_domains;          // distinct ClockDomain enum values seen
  std::set<std::string> sensor_frames;  // distinct header.sensor_frame values seen
};

// Folds one message's header into `stats`, in the order messages are read
// from the bag (file order) — monotonicity is checked against that order,
// not re-sorted.
void AccumulateHeader(HeaderStats& stats, const uw::domain::ObservationHeader& header);

// BFS over RigCalibrationSnapshot.frame_tree, traversed as an undirected
// graph (a real TF query resolves a chain regardless of which direction
// each edge happens to be declared) — returns whether `frame` is reachable
// from "base_link". "base_link" itself always resolves (it's the root).
bool FrameResolves(const uw::domain::RigCalibrationSnapshot& rig, const std::string& frame);

}  // namespace uw::runtime
