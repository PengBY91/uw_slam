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
  // First populated capture time, in bag order. Together with
  // last_capture_ns this gives the topic's time span, which is what lets an
  // audit report where a stream starts and at what mean rate it runs --
  // needed to check the IMU stationary pre-roll that
  // docs/imu-preintegration-design-2026-09-03.md section 7 requires before
  // the first keyframe boundary. Only meaningful when
  // capture_time_ever_populated is true.
  int64_t first_capture_ns = std::numeric_limits<int64_t>::max();
  // How many capture / receive times were folded in. These equal `count`
  // today (every message contributes, see AccumulateHeader) and exist so
  // rate and ordering are computed from the timestamp series itself rather
  // than from the message count, which need not stay the same thing.
  uint64_t capture_time_count = 0;
  uint64_t receive_time_count = 0;
  // All-zero stamps seen AFTER a populated one. The qualifier is the whole
  // point: in a simulation clock domain t = 0 is a real instant, so a
  // leading zero stamp is indistinguishable from "this stream starts at
  // zero" -- which is exactly what apps/synth_bag_gen.cpp's stationary
  // pre-roll produces -- and must not be called a defect. A zero stamp
  // arriving after a populated one cannot be read that way: it is either a
  // missing timestamp or a message out of order, and either way the topic's
  // span, rate and ordering stop meaning anything.
  uint64_t unpopulated_capture_time_after_populated_count = 0;
  uint64_t unpopulated_receive_time_after_populated_count = 0;
  // Non-decreasing ("monotonic") is what every canonical topic must satisfy.
  // Keyframe boundaries additionally must be STRICTLY increasing -- two
  // boundaries at the same instant would define an empty preintegration
  // interval -- so that stronger property is tracked separately instead of
  // tightening the shared flag and turning legitimate same-timestamp sensor
  // messages (e.g. several sonar pings written at one keyframe tick) into
  // audit failures.
  bool capture_time_strictly_increasing = true;
  std::set<int> clock_domains;          // distinct ClockDomain enum values seen
  std::set<std::string> sensor_frames;  // distinct header.sensor_frame values seen
};

// True when an all-zero stamp arrived after a populated one, i.e. the topic
// has a genuinely missing or out-of-order timestamp and its span, rate and
// ordering are meaningless. A stream that simply starts at t = 0 is not
// this case -- see the counters above.
bool HasMixedStampPopulation(const HeaderStats& stats);

// Mean message rate over the topic's capture-time span, or 0 when fewer
// than two capture times were seen (no span to divide by).
double MeanCaptureRateHz(const HeaderStats& stats);

// Folds one message's header into `stats`, in the order messages are read
// from the bag (file order) — monotonicity is checked against that order,
// not re-sorted.
void AccumulateHeader(HeaderStats& stats, const uw::domain::ObservationHeader& header);

// BFS over RigCalibrationSnapshot.frame_tree, traversed as an undirected
// graph (a real TF query resolves a chain regardless of which direction
// each edge happens to be declared) — returns whether `frame` is reachable
// from "base_link". "base_link" itself always resolves (it's the root).
// Accumulated findings for /keyframe/boundary (PREP-B-01). The boundary
// stream is the sole contract for where an IMU preintegration interval
// starts and ends, so on top of the usual header checks an audit has to
// know that the keyframe ids are unique and that the times strictly
// increase -- a duplicate id or a repeated instant silently drops or
// empties an interval downstream (ReplayInputAccumulator rejects both, and
// this is how a bag is checked for the same defect before replay).
struct KeyframeBoundaryStats {
  HeaderStats header;
  bool keyframe_ids_unique = true;
  bool keyframe_id_ever_empty = false;
  bool source_ever_empty = false;
  std::set<std::string> keyframe_ids;
};

void AccumulateKeyframeBoundary(KeyframeBoundaryStats& stats,
                                const uw::domain::KeyframeBoundary& boundary);

// First-moment statistics over a window of ImuSample readings, used to
// judge whether a stretch of a bag is stationary.
//
// Stationarity is judged on the window MEAN, never per sample: at 200 Hz
// with a realistic accelerometer noise density (configs/rig/*.yaml's
// sigma_accel_c = 2.0e-3 m/s^2/sqrt(Hz) discretizes to ~0.028 m/s^2 per
// axis) roughly 8% of individual samples exceed the 0.05 m/s^2 threshold in
// docs/imu-preintegration-design-2026-09-03.md section 7 purely from white
// noise, while the 0.5 s window mean sits two orders of magnitude inside
// it. Norms are taken of the mean vector, not averaged over per-sample
// norms, so that zero-mean noise actually cancels.
struct ImuWindowStats {
  uint64_t sample_count = 0;
  // Samples whose vectors are not exactly 3 entries or are non-finite.
  // Counted and skipped rather than folded in, so one bad sample cannot
  // turn a stationary window into a NaN mean.
  uint64_t malformed_sample_count = 0;
  double gyro_sum_radps[3] = {0.0, 0.0, 0.0};
  double accel_sum_mps2[3] = {0.0, 0.0, 0.0};
};

void AccumulateImuSample(ImuWindowStats& stats, const uw::domain::ImuSample& sample);

// Norm of the mean gyro / accelerometer vector over the window, or 0 when
// no well-formed sample was accumulated.
double MeanGyroNormRadps(const ImuWindowStats& stats);
double MeanAccelNormMps2(const ImuWindowStats& stats);

// FNV-1a 64-bit rolling digest over raw payload bytes in bag order. This
// exists so an audit can report "these two bags carry the same /raw/imu
// stream" without comparing whole files -- the point of the check is that
// changing what else a bag contains (cameras, relative-pose evidence) must
// not perturb the IMU stream, so the other topics legitimately differ. It
// is a fixture-comparison aid, not a cryptographic integrity check.
inline constexpr uint64_t kFnv1a64OffsetBasis = 14695981039346656037ULL;
uint64_t Fnv1a64Update(uint64_t hash, const std::string& bytes);
std::string ToHex64(uint64_t value);

bool FrameResolves(const uw::domain::RigCalibrationSnapshot& rig, const std::string& frame);

}  // namespace uw::runtime
