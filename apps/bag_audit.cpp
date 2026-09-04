// Audits a canonical MCAP bag against this repo's fixed
// canonical topic vocabulary and (optionally) a rig calibration, checking:
// topic presence, per-topic message-count plausibility, capture/receive-time
// monotonicity, clock-domain consistency, and TF-chain resolution for every
// sensor_frame referenced.
//
// This is NOT a generic MCAP inspector. It knows the same seven topics
// apps/replay_demo, apps/synth_bag_gen, and adapters/holoocean's
// record_session.py already produce/consume, and reads each with its known
// concrete protobuf type via uw::runtime::ReadMcapMessages<T> — the same
// pattern apps/replay_demo itself uses. There is no lower-level generic-
// topic enumeration here; an unrecognized topic in the bag is simply not
// audited (out of scope, not silently claimed to be fine).
//
// Camera left/right are always required — this repo's whole recording
// architecture gates every other topic on a camera keyframe existing (see
// adapters/holoocean/uw_holoocean_adapter/record_session.py's
// _write_keyframe and apps/synth_bag_gen.cpp's own keyframe-anchored
// writes). Every other topic is optional unless named via --require (e.g.
// --require /raw/sonar_frame --require /raw/imu --require /raw/dvl to audit
// a full B4-style recording against docs/uw-slam-real-recording-spec-
// 2026-08-22.md).
//
// Not every canonical topic carries the same fields — this is a real schema
// constraint, not an oversight, and drives which checks run on which topic:
//   - /raw/camera/{left,right} (ImageFrame), /raw/sonar_frame (SonarFrame),
//     /raw/imu (ImuSample), /raw/dvl (DvlSample): all carry a full
//     ObservationHeader (capture_time, receive_time, clock_domain,
//     sensor_frame) — get the full time/clock/TF check set.
//   - /gt/state (StateSnapshot): only a bare `capture_timestamp` (no
//     receive_time, no clock_domain, no sensor_frame) — capture-time
//     monotonicity only.
//   - /evidence/depth (MeasurementEvidence): no timestamp or frame field at
//     all, only ObservationId references back to a keyframe id — presence/
//     count-plausibility only (its MCAP log time is reported instead, which
//     is the only time this topic carries).
//   - /keyframe/boundary (KeyframeBoundary, PREP-B-01): a full
//     ObservationHeader plus the keyframe identity. On top of the header
//     checks its ids must be unique and its capture times STRICTLY
//     increasing — this stream is the sole contract for where an IMU
//     preintegration interval starts and ends, so a duplicate id or a
//     repeated instant silently drops or empties an interval downstream.
//
// Finally a "== machine summary ==" block prints one `key=value` line per
// reported quantity. Prose above it is for humans; this block is what
// scripts parse (tests/integration/synthetic_imu_fixture_test.sh), so its
// keys are stable and its values are never embedded in a sentence.
//
// Rate plausibility, concretely: a "kfN"-keyed /gt/state or /evidence/depth
// message is emitted AT MOST ONCE per stereo camera keyframe (synth_bag_gen
// and record_session.py both key them on the keyframe id), so "plausible"
// for those means count(kf-keyed) <= count(camera_left). Tick-keyed
// ("tickN") GT/depth and /raw/imu, /raw/dvl are written at each sensor's
// own rate by record_session.py (multi-rate recording, PREP-A-04) and are
// therefore NOT bounded by the camera — an absolute Hz is something this
// tool has no independent way to verify per bag. /raw/sonar_frame is
// likewise unbounded: apps/synth_bag_gen.cpp writes one
// SonarFrame per in-range target per keyframe (a documented v1
// simplification, see RenderSyntheticSonarFrame's call site), so a
// synthetic bag can and does have MORE sonar messages than camera
// keyframes — a real architectural fact this tool must not misreport as a
// violation (an earlier draft of this check assumed a 1:1-or-fewer bound
// for every non-camera topic and had to be corrected against this exact
// case). Camera left/right matching each other (the stereo-pair invariant)
// is checked whenever either side is present. A bag with NO camera pair at
// all (monocular / sensor-only recording -- the contract vehicle has a
// single camera, see PREP-A-04) skips every camera-keyframe-bounded check:
// there is no keyframe count to bound by, and record_session.py writes
// GT/depth/IMU/DVL per tick in that case.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>

#include "domain/domain.hpp"
#include "runtime/bag_audit_checks.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_io.hpp"

using uw::runtime::AccumulateHeader;
using uw::runtime::FrameResolves;
using uw::runtime::HeaderStats;
using uw::runtime::KeyframeBoundaryStats;
using uw::runtime::MeanCaptureRateHz;
using uw::runtime::StampIsZero;
using uw::runtime::StampToNanos;

namespace {

template <typename T>
HeaderStats CollectHeaderStats(const std::string& bag_path, const std::string& topic) {
  HeaderStats stats;
  uw::runtime::ReadMcapMessages<T>(bag_path, topic, [&](uint64_t, const T& message) {
    AccumulateHeader(stats, message.header());
  });
  return stats;
}

// "kfN" ids are stereo-keyframe-keyed (one per camera-bearing tick, bounded
// by the camera keyframe count); anything else ("tickN") is a per-tick
// reading written at the sensor's own rate and is NOT bounded by the camera.
bool IsKeyframeKeyed(const std::string& id) { return id.rfind("kf", 0) == 0; }

struct StateStats {
  uint64_t count = 0;
  uint64_t keyframe_keyed = 0;
  bool capture_time_monotonic = true;
  bool capture_time_ever_populated = false;
  bool has_capture_span = false;
  int64_t first_capture_ns = 0;
  int64_t last_capture_ns = std::numeric_limits<int64_t>::min();
};

StateStats CollectStateStats(const std::string& bag_path, const std::string& topic) {
  StateStats stats;
  uw::runtime::ReadMcapMessages<uw::domain::StateSnapshot>(
      bag_path, topic, [&](uint64_t, const uw::domain::StateSnapshot& msg) {
        ++stats.count;
        if (IsKeyframeKeyed(msg.state_id().value())) ++stats.keyframe_keyed;
        // Same convention as AccumulateHeader: t = 0 is a real instant and
        // belongs in the span; "was this ever stamped" is a whole-topic
        // property, tracked separately below.
        if (!StampIsZero(msg.capture_timestamp())) stats.capture_time_ever_populated = true;
        const int64_t ns = StampToNanos(msg.capture_timestamp());
        if (stats.has_capture_span) {
          if (ns < stats.last_capture_ns) stats.capture_time_monotonic = false;
        } else {
          stats.first_capture_ns = ns;
        }
        stats.has_capture_span = true;
        stats.last_capture_ns = ns;
      });
  return stats;
}

struct EvidenceStats {
  uint64_t count = 0;
  uint64_t keyframe_keyed = 0;
  // MeasurementEvidence carries no timestamp field, so the MCAP log time is
  // the only time information this topic has. It is reported (not checked
  // for monotonicity) because a bag's depth stream still has to line up in
  // time with the rest of the recording.
  bool has_log_time_span = false;
  uint64_t first_log_time_ns = 0;
  uint64_t last_log_time_ns = 0;
};

EvidenceStats CollectEvidenceStats(const std::string& bag_path, const std::string& topic) {
  EvidenceStats stats;
  uw::runtime::ReadMcapMessages<uw::domain::MeasurementEvidence>(
      bag_path, topic, [&](uint64_t log_time_ns, const uw::domain::MeasurementEvidence& msg) {
        ++stats.count;
        if (msg.source_observations_size() > 0 && IsKeyframeKeyed(msg.source_observations(0).value())) {
          ++stats.keyframe_keyed;
        }
        if (!stats.has_log_time_span) stats.first_log_time_ns = log_time_ns;
        stats.has_log_time_span = true;
        stats.last_log_time_ns = log_time_ns;
      });
  return stats;
}

// FNV-1a over one topic's serialized payloads, in bag order. Reported per
// topic so two bags can be compared on one stream at a time: the point of
// such a comparison is that the OTHER topics legitimately differ (different
// rig, evidence deliberately omitted), which rules out comparing files.
template <typename T>
uint64_t CollectPayloadDigest(const std::string& bag_path, const std::string& topic) {
  uint64_t digest = uw::runtime::kFnv1a64OffsetBasis;
  std::string payload;
  uw::runtime::ReadMcapMessages<T>(bag_path, topic, [&](uint64_t, const T& message) {
    payload.clear();
    message.SerializeToString(&payload);
    digest = uw::runtime::Fnv1a64Update(digest, payload);
  });
  return digest;
}

KeyframeBoundaryStats CollectKeyframeBoundaryStats(const std::string& bag_path,
                                                    const std::string& topic) {
  KeyframeBoundaryStats stats;
  uw::runtime::ReadMcapMessages<uw::domain::KeyframeBoundary>(
      bag_path, topic, [&](uint64_t, const uw::domain::KeyframeBoundary& boundary) {
        uw::runtime::AccumulateKeyframeBoundary(stats, boundary);
      });
  return stats;
}

// IMU needs its own collector rather than plain CollectHeaderStats because
// two of its reported quantities are not header properties: the stationary
// pre-roll statistics (which need the first keyframe boundary's time to
// know where the pre-roll window ends) and the payload digest (which is how
// two bags are compared on this one topic without comparing whole files).
struct ImuBagStats {
  HeaderStats header;
  // Ill-formed payloads anywhere on /raw/imu. Counted separately from the
  // pre-roll window's own tally: the window is bounded and only exists when
  // the bag has keyframe boundaries, so reporting its count as "the bag's
  // malformed IMU samples" would read 0 for a bag whose every sample after
  // the first boundary is broken.
  uint64_t malformed_sample_count = 0;
  uw::runtime::ImuWindowStats pre_roll;
  // The single sample immediately after the first keyframe boundary. Read
  // together with the pre-roll means it answers "does this stream step into
  // motion at the instant the first keyframe starts?" -- a step there means
  // the recording has no continuous motion across the seam, so a stationary
  // initialization is describing a state the vehicle is no longer in one
  // sample later. Reported, never judged: a real recording that starts
  // moving immediately is legitimate, it just cannot be initialized as
  // stationary.
  uw::runtime::ImuWindowStats first_post_boundary;
  uint64_t payload_digest = uw::runtime::kFnv1a64OffsetBasis;
};

ImuBagStats CollectImuStats(const std::string& bag_path, const std::string& topic,
                            bool has_first_boundary, int64_t first_boundary_ns) {
  ImuBagStats stats;
  std::string payload;
  uw::runtime::ReadMcapMessages<uw::domain::ImuSample>(
      bag_path, topic, [&](uint64_t, const uw::domain::ImuSample& sample) {
        AccumulateHeader(stats.header, sample.header());
        payload.clear();
        sample.SerializeToString(&payload);
        stats.payload_digest = uw::runtime::Fnv1a64Update(stats.payload_digest, payload);
        uw::runtime::ImuWindowStats whole_stream;
        uw::runtime::AccumulateImuSample(whole_stream, sample);
        stats.malformed_sample_count += whole_stream.malformed_sample_count;
        // No StampIsZero guard below: t = 0 is a real instant in a
        // simulation clock domain and is exactly where a stationary pre-roll
        // starts, so skipping it would under-report the window by one sample.
        if (!has_first_boundary) return;
        const int64_t capture_ns = StampToNanos(sample.header().capture_time());
        if (capture_ns <= first_boundary_ns) {
          uw::runtime::AccumulateImuSample(stats.pre_roll, sample);
        } else if (stats.first_post_boundary.sample_count == 0 &&
                   stats.first_post_boundary.malformed_sample_count == 0) {
          uw::runtime::AccumulateImuSample(stats.first_post_boundary, sample);
        }
      });
  return stats;
}

struct Options {
  std::string bag_path;
  std::string rig_path;  // empty = TF check skipped
  std::unordered_set<std::string> required_topics;
};

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << arg << "\n";
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--bag") {
      opt.bag_path = next();
    } else if (arg == "--rig") {
      opt.rig_path = next();
    } else if (arg == "--require") {
      opt.required_topics.insert(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }
  if (opt.bag_path.empty()) {
    std::cerr << "usage: bag_audit --bag <path.mcap> [--rig <rig.yaml>] [--require <topic>]...\n";
    return 1;
  }

  bool ok = true;
  auto fail = [&](const std::string& message) {
    std::cerr << "FAIL: " << message << "\n";
    ok = false;
  };

  std::optional<uw::domain::RigCalibrationSnapshot> rig;
  if (!opt.rig_path.empty()) {
    rig = uw::runtime::LoadRigConfig(opt.rig_path);
    std::cout << "rig: " << opt.rig_path << " (" << rig->frame_tree_size() << " frame_tree edges)\n";
  } else {
    std::cout << "rig: none given — TF-chain checks skipped\n";
  }

  auto is_required = [&](const std::string& topic) { return opt.required_topics.count(topic) > 0; };

  const auto left = CollectHeaderStats<uw::domain::ImageFrame>(opt.bag_path, "/raw/camera/left");
  const auto right = CollectHeaderStats<uw::domain::ImageFrame>(opt.bag_path, "/raw/camera/right");
  const auto main_camera = CollectHeaderStats<uw::domain::ImageFrame>(opt.bag_path, "/raw/camera/main");
  const auto sonar = CollectHeaderStats<uw::domain::SonarFrame>(opt.bag_path, "/raw/sonar_frame");
  const auto boundary = CollectKeyframeBoundaryStats(opt.bag_path, uw::runtime::kTopicKeyframeBoundary);
  // The boundary stream is read before the IMU one on purpose: the first
  // boundary's capture time is what bounds the stationary pre-roll window
  // the IMU stats are measured over.
  const auto imu_stats = CollectImuStats(opt.bag_path, "/raw/imu",
                                          boundary.header.capture_time_ever_populated,
                                          boundary.header.first_capture_ns);
  const HeaderStats& imu = imu_stats.header;
  const auto dvl = CollectHeaderStats<uw::domain::DvlSample>(opt.bag_path, "/raw/dvl");
  const auto gt_state = CollectStateStats(opt.bag_path, "/gt/state");
  const auto depth = CollectEvidenceStats(opt.bag_path, "/evidence/depth");
  // Reported, never required: an IMU-mode bag legitimately still carries
  // relative-pose evidence (the estimator must ignore it), and a bag that
  // omits it is equally legitimate. What matters is that the count is
  // visible, so "the estimator built zero relative-pose factors" can be
  // read against "the bag actually contained some".
  const auto relative_pose = CollectEvidenceStats(opt.bag_path, "/evidence/relative_pose");

  // --- 1) presence ---------------------------------------------------
  // Camera left/right are required as a PAIR whenever either is present:
  // a stereo bag with one side missing is a broken recording. A bag with
  // neither is a monocular / sensor-only recording (the contract vehicle,
  // PREP-A-04 in docs/ROV平台到货前准备工作规格-2026-09-02.md, carries a single
  // camera), which is legitimate -- pass --require for the topics such a
  // bag must carry instead of relying on the stereo anchor.
  std::cout << "\n== topic presence ==\n";
  auto report_presence = [&](const std::string& topic, uint64_t count, bool required) {
    const std::string status = count > 0 ? "present" : (required ? "MISSING (required)" : "absent (optional)");
    std::cout << "  " << topic << ": " << status << " (" << count << " messages)\n";
    if (count == 0 && required) fail(topic + " is required but has zero messages");
  };
  const bool has_stereo = left.count > 0 && right.count > 0;
  if (!has_stereo) {
    std::cout << "  stereo pair: absent -- monocular / sensor-only bag (a lone camera on one side is "
                 "legitimate), stereo-keyframe bounds below are skipped\n";
  }
  report_presence("/raw/camera/left", left.count, is_required("/raw/camera/left"));
  report_presence("/raw/camera/right", right.count, is_required("/raw/camera/right"));
  report_presence("/raw/camera/main", main_camera.count, is_required("/raw/camera/main"));
  report_presence("/gt/state", gt_state.count, is_required("/gt/state"));
  report_presence("/evidence/depth", depth.count, is_required("/evidence/depth"));
  report_presence("/raw/sonar_frame", sonar.count, is_required("/raw/sonar_frame"));
  report_presence("/raw/imu", imu.count, is_required("/raw/imu"));
  report_presence("/raw/dvl", dvl.count, is_required("/raw/dvl"));
  report_presence(uw::runtime::kTopicKeyframeBoundary, boundary.header.count,
                  is_required(uw::runtime::kTopicKeyframeBoundary));

  // --- 2) rate plausibility -------------------------------------------
  std::cout << "\n== rate plausibility (relative to camera keyframe count) ==\n";
  if (has_stereo && left.count != right.count) {
    fail("camera left/right message counts differ (" + std::to_string(left.count) + " vs " +
         std::to_string(right.count) + ") — stereo pairing is broken");
  } else if (has_stereo) {
    std::cout << "  camera left/right paired: " << left.count << " == " << right.count << "\n";
  } else if (left.count > 0 || right.count > 0) {
    std::cout << "  monocular camera on " << (left.count > 0 ? "/raw/camera/left" : "/raw/camera/right")
              << " (" << (left.count > 0 ? left.count : right.count) << " frames), no stereo pairing to check\n";
  }
  // Only keyframe-keyed ("kfN") GT/depth messages are bounded by the camera
  // keyframe count; tick-keyed ones are written at the sensor's own rate
  // (record_session.py multi-rate semantics, PREP-A-04). IMU/DVL/sonar are
  // never bounded — see this file's header comment.
  auto check_keyframe_keyed_bound = [&](const std::string& topic, uint64_t keyframe_keyed, uint64_t total) {
    const uint64_t tick_keyed = total - keyframe_keyed;
    if (has_stereo && keyframe_keyed > left.count) {
      fail(topic + " has more keyframe-keyed messages (" + std::to_string(keyframe_keyed) +
           ") than camera keyframes (" + std::to_string(left.count) +
           ") — a \"kfN\"-keyed message is written at most once per stereo keyframe");
    } else if (total > 0) {
      std::cout << "  " << topic << ": " << keyframe_keyed << " keyframe-keyed"
                << (has_stereo ? " <= " + std::to_string(left.count) + " camera keyframes" : "") << ", "
                << tick_keyed << " per-tick (not bounded)\n";
    }
  };
  check_keyframe_keyed_bound("/gt/state", gt_state.keyframe_keyed, gt_state.count);
  check_keyframe_keyed_bound("/evidence/depth", depth.keyframe_keyed, depth.count);
  auto report_unbounded = [&](const std::string& topic, uint64_t count, const char* why) {
    if (count > 0) std::cout << "  " << topic << ": " << count << " messages (" << why << ")\n";
  };
  report_unbounded("/raw/camera/main", main_camera.count,
                   "monocular gimbal camera (PREP-A-03), own rate, never a stereo keyframe");
  report_unbounded("/raw/imu", imu.count, "own rate, not bounded by camera keyframes");
  report_unbounded("/raw/dvl", dvl.count, "own rate, not bounded by camera keyframes");
  report_unbounded("/raw/sonar_frame", sonar.count,
                   "not bounded by camera-keyframe count — multiple sonar pings per keyframe is legitimate");
  report_unbounded(uw::runtime::kTopicKeyframeBoundary, boundary.header.count,
                   "one per estimator keyframe, set by the keyframe scheduler rather than by any sensor");

  // --- 2b) keyframe-boundary contract ---------------------------------
  std::cout << "\n== keyframe boundary contract ==\n";
  if (boundary.header.count == 0) {
    std::cout << "  " << uw::runtime::kTopicKeyframeBoundary
              << " absent — no IMU preintegration interval contract in this bag\n";
  } else {
    if (boundary.keyframe_id_ever_empty) fail("a KeyframeBoundary carries an empty keyframe_id");
    if (boundary.source_ever_empty) fail("a KeyframeBoundary carries an empty source");
    if (!boundary.keyframe_ids_unique) {
      fail("KeyframeBoundary keyframe ids are not unique (" +
           std::to_string(boundary.keyframe_ids.size()) + " distinct ids across " +
           std::to_string(boundary.header.count) +
           " boundaries) — a repeated id silently drops an interval downstream");
    }
    if (!boundary.header.capture_time_strictly_increasing) {
      fail("KeyframeBoundary capture times are not strictly increasing — two boundaries at "
           "the same instant define an empty preintegration interval");
    }
    if (boundary.keyframe_ids_unique && boundary.header.capture_time_strictly_increasing &&
        !boundary.keyframe_id_ever_empty && !boundary.source_ever_empty) {
      std::cout << "  " << boundary.header.count
                << " boundaries: ids unique and non-empty, capture times strictly increasing\n";
    }
  }

  // --- 3) capture/receive-time monotonicity ---------------------------
  std::cout << "\n== capture/receive-time monotonicity ==\n";
  auto check_header_time = [&](const std::string& topic, const HeaderStats& stats) {
    if (stats.count == 0) return;  // nothing to check on an absent topic
    if (uw::runtime::HasMixedStampPopulation(stats)) {
      // Reported as its own defect rather than as a bare "not monotonic",
      // which would send a reader looking for an out-of-order producer
      // instead of a missing timestamp.
      fail(topic + ": " +
           std::to_string(stats.unpopulated_capture_time_after_populated_count) +
           " capture_time and " +
           std::to_string(stats.unpopulated_receive_time_after_populated_count) +
           " receive_time stamp(s) are all-zero AFTER an already-stamped message — a missing or "
           "out-of-order timestamp, which makes this topic's span, rate and ordering meaningless "
           "(a stream that simply starts at t = 0 is not this case)");
    } else if (!stats.capture_time_ever_populated) {
      fail(topic + ": capture_time is never populated (all-zero Stamp on every message)");
    } else if (!stats.capture_time_monotonic) {
      fail(topic + ": capture_time is not monotonically non-decreasing");
    }
    if (!stats.receive_time_ever_populated) {
      fail(topic + ": receive_time is never populated (all-zero Stamp on every message)");
    } else if (!stats.receive_time_monotonic) {
      fail(topic + ": receive_time is not monotonically non-decreasing");
    }
    if (stats.capture_time_ever_populated && stats.receive_time_ever_populated &&
        stats.capture_time_monotonic && stats.receive_time_monotonic) {
      std::cout << "  " << topic << ": capture_time and receive_time both populated and monotonic\n";
    }
  };
  check_header_time("/raw/camera/left", left);
  check_header_time("/raw/camera/right", right);
  check_header_time("/raw/camera/main", main_camera);
  check_header_time("/raw/sonar_frame", sonar);
  check_header_time("/raw/imu", imu);
  check_header_time("/raw/dvl", dvl);
  check_header_time(uw::runtime::kTopicKeyframeBoundary, boundary.header);
  if (gt_state.count > 0) {
    if (!gt_state.capture_time_monotonic) {
      fail("/gt/state: capture_timestamp is not monotonically non-decreasing");
    } else {
      std::cout << "  /gt/state: capture_timestamp monotonic (no receive_time/clock_domain field to check — "
                   "StateSnapshot doesn't carry an ObservationHeader)\n";
    }
  }
  if (depth.count > 0) {
    std::cout << "  /evidence/depth: no timestamp field on MeasurementEvidence — time check does not apply\n";
  }

  // --- 4) clock-domain consistency ------------------------------------
  std::cout << "\n== clock-domain consistency ==\n";
  std::set<int> all_clock_domains;
  for (const auto* stats : {&left, &right, &main_camera, &sonar, &imu, &dvl, &boundary.header}) {
    all_clock_domains.insert(stats->clock_domains.begin(), stats->clock_domains.end());
  }
  all_clock_domains.erase(static_cast<int>(uw::domain::CLOCK_DOMAIN_UNSPECIFIED));
  if (all_clock_domains.size() > 1) {
    std::string domains;
    for (int d : all_clock_domains) domains += std::to_string(d) + " ";
    fail("bag mixes " + std::to_string(all_clock_domains.size()) +
         " distinct ClockDomain values across topics (" + domains + ") — see time.proto's ClockDomain enum");
  } else if (all_clock_domains.size() == 1) {
    std::cout << "  single clock domain across all header-bearing topics: "
              << *all_clock_domains.begin() << "\n";
  } else {
    std::cout << "  no header-bearing topic present to check\n";
  }

  // --- 5) TF-chain resolution ------------------------------------------
  std::cout << "\n== TF-chain resolution ==\n";
  if (!rig.has_value()) {
    std::cout << "  skipped: no --rig given\n";
  } else {
    std::set<std::string> all_frames;
    for (const auto* stats : {&left, &right, &main_camera, &sonar, &imu, &dvl, &boundary.header}) {
      all_frames.insert(stats->sensor_frames.begin(), stats->sensor_frames.end());
    }
    if (all_frames.empty()) {
      std::cout << "  no sensor_frame values referenced by any present topic\n";
    }
    for (const auto& frame : all_frames) {
      if (FrameResolves(*rig, frame)) {
        std::cout << "  " << frame << ": resolves from base_link\n";
      } else {
        fail("sensor_frame '" + frame + "' does not resolve from base_link in " + opt.rig_path +
             "'s frame_tree — a message claims this frame but the rig calibration has no edge for it");
      }
    }
  }

  // --- 6) machine summary ----------------------------------------------
  // Everything above is prose for a human reading an audit; this block is
  // what scripts parse. One `key=value` per line, values never embedded in
  // a sentence, keys stable across releases.
  std::cout << "\n== machine summary ==\n";
  auto emit_count = [](const std::string& key, uint64_t value) {
    std::cout << "summary." << key << "=" << value << "\n";
  };
  auto emit_bool = [](const std::string& key, bool value) {
    std::cout << "summary." << key << "=" << (value ? "true" : "false") << "\n";
  };
  auto emit_double = [](const std::string& key, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9f", value);
    std::cout << "summary." << key << "=" << buffer << "\n";
  };
  auto emit_nanos_as_seconds = [&](const std::string& key, bool present, int64_t ns) {
    if (!present) return;
    emit_double(key, static_cast<double>(ns) * 1e-9);
  };
  auto emit_header_topic = [&](const std::string& key, const HeaderStats& stats) {
    emit_count(key + ".count", stats.count);
    // A topic mixing stamped and unstamped messages has already failed
    // above; publishing its span and rate would put a first_capture_s of 0
    // and a rate divided by a ~1.7e18 ns span into a machine-readable block
    // that a script is entitled to trust.
    const bool has_usable_times =
        stats.capture_time_ever_populated && !uw::runtime::HasMixedStampPopulation(stats);
    emit_nanos_as_seconds(key + ".first_capture_s", has_usable_times, stats.first_capture_ns);
    emit_nanos_as_seconds(key + ".last_capture_s", has_usable_times, stats.last_capture_ns);
    if (has_usable_times && stats.capture_time_count >= 2) {
      emit_double(key + ".mean_rate_hz", MeanCaptureRateHz(stats));
    }
  };

  emit_header_topic("camera_left", left);
  emit_header_topic("camera_right", right);
  emit_header_topic("camera_main", main_camera);
  emit_header_topic("sonar_frame", sonar);
  emit_header_topic("imu", imu);
  emit_header_topic("dvl", dvl);
  emit_header_topic("keyframe_boundary", boundary.header);

  emit_count("gt_state.count", gt_state.count);
  emit_nanos_as_seconds("gt_state.first_capture_s", gt_state.capture_time_ever_populated,
                        gt_state.first_capture_ns);
  emit_nanos_as_seconds("gt_state.last_capture_s", gt_state.capture_time_ever_populated,
                        gt_state.last_capture_ns);

  emit_count("evidence_relative_pose.count", relative_pose.count);
  if (relative_pose.count > 0) {
    std::cout << "summary.evidence_relative_pose.payload_digest="
              << uw::runtime::ToHex64(CollectPayloadDigest<uw::domain::MeasurementEvidence>(
                     opt.bag_path, "/evidence/relative_pose"))
              << "\n";
  }
  if (sonar.count > 0) {
    std::cout << "summary.sonar_frame.payload_digest="
              << uw::runtime::ToHex64(CollectPayloadDigest<uw::domain::SonarFrame>(
                     opt.bag_path, "/raw/sonar_frame"))
              << "\n";
  }
  emit_count("evidence_depth.count", depth.count);
  emit_nanos_as_seconds("evidence_depth.first_log_time_s", depth.has_log_time_span,
                        static_cast<int64_t>(depth.first_log_time_ns));
  emit_nanos_as_seconds("evidence_depth.last_log_time_s", depth.has_log_time_span,
                        static_cast<int64_t>(depth.last_log_time_ns));

  if (boundary.header.count > 0) {
    emit_count("keyframe_boundary.distinct_ids", boundary.keyframe_ids.size());
    emit_bool("keyframe_boundary.ids_unique", boundary.keyframe_ids_unique);
    emit_bool("keyframe_boundary.capture_strictly_increasing",
              boundary.header.capture_time_strictly_increasing);
  }

  if (imu.count > 0) {
    emit_count("imu.malformed_sample_count", imu_stats.malformed_sample_count);
    // The stationary pre-roll: IMU samples at or before the first keyframe
    // boundary. docs/imu-preintegration-design-2026-09-03.md section 7
    // requires at least 0.5 s of it, with the window MEAN gyro norm below
    // 0.01 rad/s and the window MEAN specific-force norm within 0.05 m/s^2
    // of gravity. Reported, never judged here — the threshold belongs to
    // the initializer and to whichever fixture is being checked, not to a
    // generic bag audit (a real recording legitimately may not start at
    // rest).
    if (boundary.header.capture_time_ever_populated && imu.capture_time_ever_populated) {
      emit_double("imu.pre_roll_s",
                  static_cast<double>(boundary.header.first_capture_ns - imu.first_capture_ns) * 1e-9);
      emit_count("imu.pre_roll_sample_count", imu_stats.pre_roll.sample_count);
      emit_count("imu.pre_roll_malformed_sample_count", imu_stats.pre_roll.malformed_sample_count);
      emit_double("imu.pre_roll_gyro_mean_norm_radps",
                  uw::runtime::MeanGyroNormRadps(imu_stats.pre_roll));
      emit_double("imu.pre_roll_accel_mean_norm_mps2",
                  uw::runtime::MeanAccelNormMps2(imu_stats.pre_roll));
      if (imu_stats.first_post_boundary.sample_count == 1) {
        emit_double("imu.first_post_boundary_gyro_norm_radps",
                    uw::runtime::MeanGyroNormRadps(imu_stats.first_post_boundary));
        emit_double("imu.first_post_boundary_accel_norm_mps2",
                    uw::runtime::MeanAccelNormMps2(imu_stats.first_post_boundary));
      }
    }
    std::cout << "summary.imu.payload_digest=" << uw::runtime::ToHex64(imu_stats.payload_digest)
              << "\n";
  }

  std::cout << "\n" << (ok ? "RESULT: PASS" : "RESULT: FAIL") << "\n";
  return ok ? 0 : 1;
}
