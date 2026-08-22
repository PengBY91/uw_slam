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
//     count-plausibility only.
//
// Rate plausibility, concretely: /gt/state, /evidence/depth, /raw/imu, and
// /raw/dvl are each emitted AT MOST ONCE per camera keyframe in both
// synth_bag_gen and record_session.py (one sensor reading per tick), so
// "plausible" for those four means count(topic) <= count(camera_left) — not
// an assumed absolute Hz this repo has no independent way to verify per-bag
// (see the recording spec's own note that sonar/IMU/DVL are downsampled to
// camera-keyframe rate, not their native rate). /raw/sonar_frame is
// deliberately EXCLUDED from that bound: apps/synth_bag_gen.cpp writes one
// SonarFrame per in-range target per keyframe (a documented v1
// simplification, see RenderSyntheticSonarFrame's call site), so a
// synthetic bag can and does have MORE sonar messages than camera
// keyframes — a real architectural fact this tool must not misreport as a
// violation (an earlier draft of this check assumed a 1:1-or-fewer bound
// for every non-camera topic and had to be corrected against this exact
// case). Camera left/right matching each other (the stereo-pair invariant)
// is checked unconditionally.
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>

#include "domain/domain.hpp"
#include "runtime/bag_audit_checks.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_io.hpp"

using uw::runtime::AccumulateHeader;
using uw::runtime::FrameResolves;
using uw::runtime::HeaderStats;
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

struct StateStats {
  uint64_t count = 0;
  bool capture_time_monotonic = true;
  int64_t last_capture_ns = std::numeric_limits<int64_t>::min();
};

StateStats CollectStateStats(const std::string& bag_path, const std::string& topic) {
  StateStats stats;
  uw::runtime::ReadMcapMessages<uw::domain::StateSnapshot>(
      bag_path, topic, [&](uint64_t, const uw::domain::StateSnapshot& msg) {
        ++stats.count;
        if (StampIsZero(msg.capture_timestamp())) return;
        const int64_t ns = StampToNanos(msg.capture_timestamp());
        if (ns < stats.last_capture_ns) stats.capture_time_monotonic = false;
        stats.last_capture_ns = ns;
      });
  return stats;
}

uint64_t CountMessages(const std::string& bag_path, const std::string& topic) {
  uint64_t count = 0;
  uw::runtime::ReadMcapMessages<uw::domain::MeasurementEvidence>(
      bag_path, topic, [&](uint64_t, const uw::domain::MeasurementEvidence&) { ++count; });
  return count;
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
  const auto sonar = CollectHeaderStats<uw::domain::SonarFrame>(opt.bag_path, "/raw/sonar_frame");
  const auto imu = CollectHeaderStats<uw::domain::ImuSample>(opt.bag_path, "/raw/imu");
  const auto dvl = CollectHeaderStats<uw::domain::DvlSample>(opt.bag_path, "/raw/dvl");
  const auto gt_state = CollectStateStats(opt.bag_path, "/gt/state");
  const uint64_t depth_count = CountMessages(opt.bag_path, "/evidence/depth");

  // --- 1) presence ---------------------------------------------------
  // Camera left/right are always required regardless of --require: this
  // repo's recording architecture has nothing else to anchor on without
  // them (see this file's header comment).
  std::cout << "\n== topic presence ==\n";
  auto report_presence = [&](const std::string& topic, uint64_t count, bool required) {
    const std::string status = count > 0 ? "present" : (required ? "MISSING (required)" : "absent (optional)");
    std::cout << "  " << topic << ": " << status << " (" << count << " messages)\n";
    if (count == 0 && required) fail(topic + " is required but has zero messages");
  };
  report_presence("/raw/camera/left", left.count, true);
  report_presence("/raw/camera/right", right.count, true);
  report_presence("/gt/state", gt_state.count, is_required("/gt/state"));
  report_presence("/evidence/depth", depth_count, is_required("/evidence/depth"));
  report_presence("/raw/sonar_frame", sonar.count, is_required("/raw/sonar_frame"));
  report_presence("/raw/imu", imu.count, is_required("/raw/imu"));
  report_presence("/raw/dvl", dvl.count, is_required("/raw/dvl"));

  // --- 2) rate plausibility -------------------------------------------
  std::cout << "\n== rate plausibility (relative to camera keyframe count) ==\n";
  if (left.count != right.count) {
    fail("camera left/right message counts differ (" + std::to_string(left.count) + " vs " +
         std::to_string(right.count) + ") — stereo pairing is broken");
  } else {
    std::cout << "  camera left/right paired: " << left.count << " == " << right.count << "\n";
  }
  auto check_bounded_by_camera = [&](const std::string& topic, uint64_t count) {
    if (count > left.count) {
      fail(topic + " has more messages (" + std::to_string(count) + ") than camera keyframes (" +
           std::to_string(left.count) + ") — every canonical topic in this architecture is written at most "
           "once per camera-bearing tick, see this file's header comment");
    } else {
      std::cout << "  " << topic << ": " << count << " <= " << left.count << " camera keyframes\n";
    }
  };
  check_bounded_by_camera("/gt/state", gt_state.count);
  check_bounded_by_camera("/evidence/depth", depth_count);
  check_bounded_by_camera("/raw/imu", imu.count);
  check_bounded_by_camera("/raw/dvl", dvl.count);
  // /raw/sonar_frame deliberately NOT bounded by camera count — see this
  // file's header comment (multiple sonar pings per keyframe is legitimate).
  if (sonar.count > 0) {
    std::cout << "  /raw/sonar_frame: " << sonar.count
              << " messages (not bounded by camera-keyframe count — multiple sonar pings per keyframe "
                 "is architecturally legitimate, see this file's header comment)\n";
  }

  // --- 3) capture/receive-time monotonicity ---------------------------
  std::cout << "\n== capture/receive-time monotonicity ==\n";
  auto check_header_time = [&](const std::string& topic, const HeaderStats& stats) {
    if (stats.count == 0) return;  // nothing to check on an absent topic
    if (!stats.capture_time_ever_populated) {
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
  check_header_time("/raw/sonar_frame", sonar);
  check_header_time("/raw/imu", imu);
  check_header_time("/raw/dvl", dvl);
  if (gt_state.count > 0) {
    if (!gt_state.capture_time_monotonic) {
      fail("/gt/state: capture_timestamp is not monotonically non-decreasing");
    } else {
      std::cout << "  /gt/state: capture_timestamp monotonic (no receive_time/clock_domain field to check — "
                   "StateSnapshot doesn't carry an ObservationHeader)\n";
    }
  }
  if (depth_count > 0) {
    std::cout << "  /evidence/depth: no timestamp field on MeasurementEvidence — time check does not apply\n";
  }

  // --- 4) clock-domain consistency ------------------------------------
  std::cout << "\n== clock-domain consistency ==\n";
  std::set<int> all_clock_domains;
  for (const auto* stats : {&left, &right, &sonar, &imu, &dvl}) {
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
    for (const auto* stats : {&left, &right, &sonar, &imu, &dvl}) {
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

  std::cout << "\n" << (ok ? "RESULT: PASS" : "RESULT: FAIL") << "\n";
  return ok ? 0 : 1;
}
