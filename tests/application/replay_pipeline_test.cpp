#include "application/replay_pipeline.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/inertial_prior_residual.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/mcap_io.hpp"
#include "runtime/synthetic_sonar.hpp"
#include "sensor_models/geometry.hpp"

namespace {

using uw::runtime::McapProtobufWriter;

void WriteFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << content;
}

// Redirects std::cout for its lifetime so a test can assert on
// RunReplayPipeline's own diagnostic prints (e.g. "added N depth factors")
// -- these are the only external signal for which evidence-consumption path
// actually fired, short of threading extra counters back out of the
// function purely for testing.
class CoutCapture {
 public:
  CoutCapture() : old_buf_(std::cout.rdbuf(captured_.rdbuf())) {}
  ~CoutCapture() { std::cout.rdbuf(old_buf_); }
  std::string str() const { return captured_.str(); }

 private:
  std::ostringstream captured_;
  std::streambuf* old_buf_;
};

// Finds `marker` in `text` and returns the integer immediately preceding
// it (e.g. ExtractIntBefore("added 3 depth factors", " depth factors") ==
// 3), or -1 if `marker` is absent or not preceded by digits.
int ExtractIntBefore(const std::string& text, const std::string& marker) {
  const auto marker_pos = text.find(marker);
  if (marker_pos == std::string::npos) return -1;
  std::size_t start = marker_pos;
  while (start > 0 && std::isdigit(static_cast<unsigned char>(text[start - 1]))) --start;
  if (start == marker_pos) return -1;
  return std::stoi(text.substr(start, marker_pos - start));
}

uw::domain::ImageFrame MakeCameraImage(const std::string& frame, double capture_time_s,
                                       const std::string& observation_id) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_observation_id()->set_value(observation_id);
  *image.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(capture_time_s);
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->mutable_sensor_id()->set_value(
      frame == "camera_left_link" ? "camera_left" : "camera_right");
  image.set_width(32);
  image.set_height(32);
  image.set_row_stride_bytes(32);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string(32 * 32, static_cast<char>(90)));
  return image;
}

}  // namespace

TEST(ReplayPipeline, RejectsEmptyBagPath) {
  uw::application::ReplayOptions options;
  EXPECT_EQ(uw::application::RunReplayPipeline(options, "test-commit"), 1);
}

TEST(ReplayPipeline, ConvertsEveryTypedSonarFrontendConfigFieldToRuntimeParameters) {
  uw::runtime::SonarFrontendConfig config;
  config.training_cells = 24;
  config.guard_cells = 8;
  config.probability_false_alarm = 0.004;
  config.detector_threshold = 72;
  config.dbscan_eps_m = 0.31;
  config.dbscan_min_samples = 5;
  config.default_range_sigma_m = 0.07;
  config.default_bearing_sigma_rad = 0.015;

  const auto params = uw::application::BuildSonarCfarFrontendParams(config);

  EXPECT_EQ(params.cfar.num_training_cells, 24);
  EXPECT_EQ(params.cfar.num_guard_cells, 8);
  EXPECT_DOUBLE_EQ(params.cfar.probability_false_alarm, 0.004);
  EXPECT_EQ(params.detector_threshold, 72);
  EXPECT_DOUBLE_EQ(params.dbscan_eps_m, 0.31);
  EXPECT_EQ(params.dbscan_min_samples, 5);
  EXPECT_DOUBLE_EQ(params.default_range_sigma_m, 0.07);
  EXPECT_DOUBLE_EQ(params.default_bearing_sigma_rad, 0.015);
}

TEST(DecideTrackingStatus, ConvergedAndHealthyIsTracking) {
  uw::application::ReplayTrackingInputs inputs;
  inputs.solver_converged = true;
  inputs.vo_enabled = false;
  EXPECT_EQ(uw::application::DecideTrackingStatus(inputs),
            uw::domain::StateSnapshot::TRACKING_STATUS_TRACKING);
}

TEST(DecideTrackingStatus, StalledSolverIsDegradedEvenWithAPose) {
  uw::application::ReplayTrackingInputs inputs;
  inputs.solver_converged = false;
  inputs.vo_enabled = false;
  EXPECT_EQ(uw::application::DecideTrackingStatus(inputs),
            uw::domain::StateSnapshot::TRACKING_STATUS_DEGRADED);
}

TEST(DecideTrackingStatus, VoSuspectIsDegraded) {
  uw::application::ReplayTrackingInputs inputs;
  inputs.solver_converged = true;
  inputs.vo_enabled = true;
  inputs.vo_health = uw::domain::HealthReport::STATUS_SUSPECT;
  EXPECT_EQ(uw::application::DecideTrackingStatus(inputs),
            uw::domain::StateSnapshot::TRACKING_STATUS_DEGRADED);
}

TEST(DecideTrackingStatus, VoUnavailableIsLost) {
  uw::application::ReplayTrackingInputs inputs;
  inputs.solver_converged = true;
  inputs.vo_enabled = true;
  inputs.vo_health = uw::domain::HealthReport::STATUS_UNAVAILABLE;
  EXPECT_EQ(uw::application::DecideTrackingStatus(inputs),
            uw::domain::StateSnapshot::TRACKING_STATUS_LOST);
}

TEST(BuildStateSnapshotTest, FillsAllFieldsAndDedupesSortsContributingEvidence) {
  uw::application::StateSnapshotInputs inputs;
  inputs.state_id = "kf3";
  inputs.state_version = 4;
  inputs.pose.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  inputs.capture_timestamp = uw::domain::FromSeconds(10.41);
  inputs.calibration_version = "raw_v1+opencv_rectified_v1_abc";
  inputs.tracking_status = uw::domain::StateSnapshot::TRACKING_STATUS_TRACKING;
  uw::domain::EvidenceId e_b, e_a, e_a_dup;
  e_b.set_value("evidence_b");
  e_a.set_value("evidence_a");
  e_a_dup.set_value("evidence_a");
  inputs.contributing_evidence = {e_b, e_a, e_a_dup};

  const auto snapshot = uw::application::BuildStateSnapshot(inputs);
  EXPECT_EQ(snapshot.state_id().value(), "kf3");
  EXPECT_EQ(snapshot.state_version().value(), 4u);
  EXPECT_NEAR(uw::domain::ToSeconds(snapshot.capture_timestamp()), 10.41, 1e-6);
  EXPECT_EQ(snapshot.calibration_version().value(), "raw_v1+opencv_rectified_v1_abc");
  EXPECT_EQ(snapshot.tracking_status(), uw::domain::StateSnapshot::TRACKING_STATUS_TRACKING);
  EXPECT_NEAR(snapshot.pose_wb().matrix_row_major(3), 1.0, 1e-9);

  ASSERT_EQ(snapshot.contributing_measurements_size(), 2);
  EXPECT_EQ(snapshot.contributing_measurements(0).value(), "evidence_a");
  EXPECT_EQ(snapshot.contributing_measurements(1).value(), "evidence_b");
}

TEST(CountDepthContributions, CountsOpticalOnlyAndAcousticOpticSeparately) {
  uw::domain::FusedDepthMeasurement fused;
  std::string mask;
  mask += static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID);
  mask += static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  mask += static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  mask += static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
  fused.set_contribution_mask(mask);

  const auto counts = uw::application::CountDepthContributions(fused);
  EXPECT_EQ(counts.optical_only_points, 2u);
  EXPECT_EQ(counts.acoustic_optic_points, 1u);
}

TEST(CountDepthContributions, EmptyMaskCountsZero) {
  uw::domain::FusedDepthMeasurement fused;
  const auto counts = uw::application::CountDepthContributions(fused);
  EXPECT_EQ(counts.optical_only_points, 0u);
  EXPECT_EQ(counts.acoustic_optic_points, 0u);
}

namespace {

uw::estimation::GaussNewtonSummary MakeConvergedSolver() {
  uw::estimation::GaussNewtonSummary summary;
  summary.converged = true;
  summary.iterations = 5;
  return summary;
}

uw::evaluation::AteResult MakeGoodAte() {
  uw::evaluation::AteResult ate;
  ate.rmse_m = 0.05;
  ate.num_matched_poses = 10;
  return ate;
}

}  // namespace

TEST(EvaluateReplayGates, OpticalOnlyContributionPassesNonemptyMapButFailsAcousticOpticGates) {
  uw::runtime::PlatformDefaultsConfig defaults;
  defaults.require_converged = true;
  defaults.require_nonempty_map = true;
  defaults.min_acoustic_optic_accepted = 1;
  defaults.min_acoustic_optic_map_points = 1;

  uw::application::MapContributionCounts contributions;
  contributions.optical_only_points = 100;
  contributions.acoustic_optic_points = 0;

  const auto failures = uw::application::EvaluateReplayGates(defaults, MakeConvergedSolver(), MakeGoodAte(),
                                                              /*num_landmarks=*/0, contributions,
                                                              /*num_acoustic_optic_accepted=*/0);
  ASSERT_EQ(failures.size(), 2u);
  EXPECT_NE(failures[0].find("acoustic-optic accepted"), std::string::npos);
  EXPECT_NE(failures[1].find("acoustic-optic map points"), std::string::npos);
}

TEST(EvaluateReplayGates, AcousticOpticContributionPassesAllGates) {
  uw::runtime::PlatformDefaultsConfig defaults;
  defaults.require_converged = true;
  defaults.require_nonempty_map = true;
  defaults.min_acoustic_optic_accepted = 1;
  defaults.min_acoustic_optic_map_points = 1;

  uw::application::MapContributionCounts contributions;
  contributions.optical_only_points = 50;
  contributions.acoustic_optic_points = 20;

  const auto failures = uw::application::EvaluateReplayGates(defaults, MakeConvergedSolver(), MakeGoodAte(),
                                                              /*num_landmarks=*/0, contributions,
                                                              /*num_acoustic_optic_accepted=*/3);
  EXPECT_TRUE(failures.empty());
}

TEST(EvaluateReplayGates, StalledSolverAlwaysFailsRegardlessOfOtherGates) {
  uw::runtime::PlatformDefaultsConfig defaults;  // all opt-in gates left disabled
  uw::estimation::GaussNewtonSummary stalled;
  stalled.converged = false;
  stalled.iterations = 30;

  const auto failures = uw::application::EvaluateReplayGates(
      defaults, stalled, MakeGoodAte(), /*num_landmarks=*/5, uw::application::MapContributionCounts{},
      /*num_acoustic_optic_accepted=*/0);
  ASSERT_EQ(failures.size(), 1u);
  EXPECT_NE(failures[0].find("did not converge"), std::string::npos);
}

TEST(EvaluateReplayGates, AcousticOpticGatesDisabledByDefault) {
  uw::runtime::PlatformDefaultsConfig defaults;
  uw::application::MapContributionCounts contributions;  // all zero
  const auto failures = uw::application::EvaluateReplayGates(defaults, MakeConvergedSolver(), MakeGoodAte(),
                                                              /*num_landmarks=*/0, contributions,
                                                              /*num_acoustic_optic_accepted=*/0);
  EXPECT_TRUE(failures.empty());
}

// Task 5 acceptance: replay no longer feeds raw camera images directly to
// the strict (is_rectified()==true-requiring) stereo frontends. This uses a
// deliberately NON-PARALLEL rig (right camera yawed 5 degrees) so
// StereoRectificationContext::Create() must take the general
// cv::stereoRectify path, not the identity fast path Task 2 covers on its
// own -- proving the general OpenCV path is actually reachable from the
// application, not just from opencv_adapters' own unit tests.
TEST(ReplayPipeline, RectifiesNonParallelRigBeforeStereoFrontendsAndPopulatesBothCalibrationHashes) {
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_rectification_test";
  std::filesystem::remove_all(root);

  WriteFile(root / "rig" / "test_rig.yaml",
            "calibration_version: \"test_rig_v1\"\n"
            "frame_tree:\n"
            "  - parent_frame: base_link\n"
            "    child_frame: camera_left_link\n"
            "    transform_row_major:\n"
            "      [1,0,0,0.15, 0,1,0,0.06, 0,0,1,0, 0,0,0,1]\n"
            "  - parent_frame: base_link\n"
            "    child_frame: camera_right_link\n"
            "    transform_row_major:\n"
            "      [0.9961947,-0.0871557,0,0.15, 0.0871557,0.9961947,0,-0.06, 0,0,1,0, 0,0,0,1]\n"
            "cameras:\n"
            "  - sensor_id: camera_left\n"
            "    width: 32\n"
            "    height: 32\n"
            "    k_matrix_row_major: [40,0,16, 0,40,16, 0,0,1]\n"
            "    distortion: [0,0,0,0]\n"
            "    distortion_model: plumb_bob\n"
            "  - sensor_id: camera_right\n"
            "    width: 32\n"
            "    height: 32\n"
            "    k_matrix_row_major: [42,0,17, 0,42,15, 0,0,1]\n"
            "    distortion: [0,0,0,0]\n"
            "    distortion_model: plumb_bob\n"
            "time_offset_seconds:\n"
            "  camera_left: 0.0\n"
            "  camera_right: 0.0\n"
            "time_offset_provenance:\n"
            "  camera_left: measured:test-clock\n"
            "  camera_right: measured:test-clock\n");

  WriteFile(root / "scenario" / "test_scenario.yaml", "seed: 1\nnum_keyframes: 2\n");

  WriteFile(root / "defaults" / "test_defaults.yaml",
            "gates:\n"
            "  require_converged: false\n"
            "  require_nonempty_map: false\n");

  WriteFile(root / "experiment" / "test_experiment.yaml",
            "rig: rig/test_rig.yaml\n"
            "scenario: scenario/test_scenario.yaml\n"
            "defaults: defaults/test_defaults.yaml\n"
            "estimator_mode: black_box_vio\n"
            "output:\n"
            "  write_run_manifest: true\n");

  const auto bag_path = root / "test.mcap";
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(bag_path.string()));
    ASSERT_TRUE(writer.WriteMessage("/raw/camera/left", 0,
                                    MakeCameraImage("camera_left_link", 0.0, "kf0")));
    ASSERT_TRUE(writer.WriteMessage("/raw/camera/right", 0,
                                    MakeCameraImage("camera_right_link", 0.0, "kf0")));
    ASSERT_TRUE(writer.WriteMessage("/raw/camera/left", 200000000,
                                    MakeCameraImage("camera_left_link", 0.2, "kf1")));
    ASSERT_TRUE(writer.WriteMessage("/raw/camera/right", 200000000,
                                    MakeCameraImage("camera_right_link", 0.2, "kf1")));

    uw::domain::PressureDepthMeasurement kf0_depth;
    kf0_depth.set_depth_m(2.0);
    kf0_depth.set_sigma_m(0.05);
    uw::domain::EvidenceId depth_evidence_id;
    depth_evidence_id.set_value("depth_kf0");
    uw::domain::ObservationId kf0_obs;
    kf0_obs.set_value("kf0");
    const auto depth_evidence = uw::domain::MakeEvidence(depth_evidence_id, {kf0_obs}, kf0_depth,
                                                          /*noise_scale=*/1.0, "test_depth_v1");
    ASSERT_TRUE(writer.WriteMessage("/evidence/depth", 0, depth_evidence));

    uw::domain::RelativePoseMeasurement relative_pose;
    relative_pose.mutable_from_keyframe()->set_value("kf0");
    relative_pose.mutable_to_keyframe()->set_value("kf1");
    uw::sensor_models::Pose3 relative;
    relative.translation = Eigen::Vector3d(0.1, 0.0, 0.0);
    *relative_pose.mutable_relative_pose() = relative.ToProto();
    uw::domain::EvidenceId relative_evidence_id;
    relative_evidence_id.set_value("relpose_kf0_kf1");
    const auto relative_evidence = uw::domain::MakeEvidence(relative_evidence_id, {}, relative_pose,
                                                             /*noise_scale=*/1.0, "test_relpose_v1");
    ASSERT_TRUE(writer.WriteMessage("/evidence/relative_pose", 200000000, relative_evidence));

    writer.Close();
  }

  uw::application::ReplayOptions options;
  options.bag_path = bag_path.string();
  options.experiment_path = (root / "experiment" / "test_experiment.yaml").string();
  options.out_prefix = (root / "out").string();

  int result = 0;
  std::string captured_stdout;
  {
    CoutCapture capture;
    result = uw::application::RunReplayPipeline(options, "test-commit");
    captured_stdout = capture.str();
  }
  // Not 1: a stereo-rectification Create() failure returns 1 before any
  // work happens. A non-converged-solver gate failure would return 2, but
  // require_converged is disabled above specifically so that can't mask a
  // rectification failure as a "gate" failure instead.
  EXPECT_NE(result, 1) << "replay pipeline failed before/during stereo rectification setup\n"
                       << captured_stdout;

  // Targeted assertions per evidence-consumption path (Task 4 Step 4):
  // the aggregate determinism gate alone doesn't prove each of these
  // per-topic filters survived the ReadMcapMessages<T> -> ReplayInputData
  // migration correctly.
  ASSERT_TRUE(std::filesystem::exists(root / "out_trajectory.tum"));
  std::ifstream trajectory(root / "out_trajectory.tum");
  std::string first_line;
  std::getline(trajectory, first_line);
  ASSERT_FALSE(first_line.empty());

  // kf0 anchor z: fixed directly from its own PressureDepthMeasurement
  // (depth_m=2.0 above) rather than the solver, so this must be exact, not
  // just converged-close. TUM line order is
  // "timestamp x y z qx qy qz qw".
  {
    std::istringstream first_line_stream(first_line);
    double timestamp = 0.0, x = 0.0, y = 0.0, z = 0.0;
    first_line_stream >> timestamp >> x >> y >> z;
    EXPECT_NEAR(z, -2.0, 1e-9) << "kf0 anchor z should come from its own depth evidence, not the solver";
  }

  // Regular depth factor count: exactly one PressureDepthMeasurement
  // evidence was written above (for kf0).
  EXPECT_EQ(ExtractIntBefore(captured_stdout, " depth factors"), 1) << captured_stdout;

  // Acoustic-optic fusion trigger: both keyframes carry a rectifiable
  // stereo pair, so the acoustic-optic pass must run over both -- this
  // fixture's flat, textureless synthetic pixels (MakeCameraImage) never
  // pass the stereo matcher's texture gate, so 0 map evidence points is the
  // correct outcome here, not evidence the pass didn't run.
  EXPECT_EQ(ExtractIntBefore(captured_stdout, " keyframes with camera data"), 2) << captured_stdout;
  EXPECT_EQ(ExtractIntBefore(captured_stdout, " map evidence points added"), 0) << captured_stdout;

  ASSERT_TRUE(std::filesystem::exists(root / "out_run_manifest.json"));
  std::ifstream manifest_file(root / "out_run_manifest.json");
  std::ostringstream manifest_content;
  manifest_content << manifest_file.rdbuf();
  const std::string manifest = manifest_content.str();

  const auto calibration_hash_pos = manifest.find("\"calibration_hash\": \"");
  ASSERT_NE(calibration_hash_pos, std::string::npos);
  const auto calibration_hash_start = calibration_hash_pos + std::string("\"calibration_hash\": \"").size();
  const auto calibration_hash_end = manifest.find('"', calibration_hash_start);
  const std::string calibration_hash =
      manifest.substr(calibration_hash_start, calibration_hash_end - calibration_hash_start);

  const auto derived_hash_pos = manifest.find("\"derived_calibration_hash\": \"");
  ASSERT_NE(derived_hash_pos, std::string::npos);
  const auto derived_hash_start = derived_hash_pos + std::string("\"derived_calibration_hash\": \"").size();
  const auto derived_hash_end = manifest.find('"', derived_hash_start);
  const std::string derived_hash = manifest.substr(derived_hash_start, derived_hash_end - derived_hash_start);

  EXPECT_FALSE(calibration_hash.empty());
  EXPECT_FALSE(derived_hash.empty());
  EXPECT_NE(calibration_hash, derived_hash);

  std::filesystem::remove_all(root);
}

// ---------------------------------------------------------------------------
// estimator_mode: imu_preintegration (PREP-B-01 Task 5)
//
// These build a bag rather than a ReplayInputData directly: the properties
// under test are about what the PIPELINE does with a bag's contents --
// specifically that ground truth and relative-pose evidence reach the
// evaluator and nothing else -- so the accumulator and the event pump have
// to be inside the test, not stubbed around.
//
// The fixture keeps the vehicle stationary for its whole length. That makes
// the expected answer exact (every keyframe at the anchor pose) so a graph
// assembly bug shows up as a factor count or a fail-closed error rather
// than as a slightly-off ATE; end-to-end accuracy is Task 6's smoke test,
// not this file's job.
namespace {

class CerrCapture {
 public:
  CerrCapture() : old_buf_(std::cerr.rdbuf(captured_.rdbuf())) {}
  ~CerrCapture() { std::cerr.rdbuf(old_buf_); }
  std::string str() const { return captured_.str(); }

 private:
  std::ostringstream captured_;
  std::streambuf* old_buf_;
};

// Pulls the value out of one `summary.<key>=<value>` line. Anchored on the
// whole key and on the '=', so a key that is a prefix of another can never
// be matched by accident.
std::string SummaryValue(const std::string& text, const std::string& key) {
  const std::string needle = "summary." + key + "=";
  const auto pos = text.find(needle);
  if (pos == std::string::npos) return "<missing>";
  const auto start = pos + needle.size();
  const auto end = text.find('\n', start);
  return text.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

constexpr double kFixtureGravity = 9.80665;
constexpr double kFixtureImuRateHz = 200.0;
constexpr double kFixturePreRollS = 0.75;
constexpr double kFixtureKeyframePeriodS = 0.2;
constexpr double kFixtureDepthM = 2.0;

struct ImuFixtureOptions {
  int num_keyframes = 3;
  bool write_keyframe_boundaries = true;
  bool write_ground_truth = false;
  // Applied to the ground-truth branch ONLY, and never to its MCAP log
  // time: the point is to move what an evaluator would read while leaving
  // every algorithm input bit-identical.
  double ground_truth_time_offset_s = 0.0;
  double ground_truth_pose_offset_m = 0.0;
  bool write_conflicting_relative_pose = false;
  // Drops every IMU sample in [imu_gap_start_s, imu_gap_start_s + duration),
  // which is how a real dropout looks to the preintegration frontend.
  double imu_gap_start_s = -1.0;
  double imu_gap_duration_s = 0.0;
};

uw::domain::ObservationHeader MakeFixtureHeader(const std::string& observation_id,
                                                const std::string& sensor_id,
                                                const std::string& sensor_frame, double time_s) {
  uw::domain::ObservationHeader header;
  header.mutable_observation_id()->set_value(observation_id);
  header.mutable_sensor_id()->set_value(sensor_id);
  header.mutable_sensor_frame()->set_value(sensor_frame);
  *header.mutable_capture_time() = uw::domain::FromSeconds(time_s);
  *header.mutable_receive_time() = header.capture_time();
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  return header;
}

void WriteImuFixture(const std::filesystem::path& root, const ImuFixtureOptions& options) {
  WriteFile(root / "rig" / "imu_rig.yaml",
            "calibration_version: \"imu_test_rig_v1\"\n"
            "frame_tree:\n"
            "  - parent_frame: base_link\n"
            "    child_frame: imu_link\n"
            "    transform_row_major:\n"
            "      [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
            "  - parent_frame: base_link\n"
            "    child_frame: sonar_link\n"
            "    transform_row_major:\n"
            "      [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
            "vehicle_state_sensors:\n"
            "  - rov-state\n"
            "imu_noise:\n"
            "  sigma_gyro_c: 1.6968e-4\n"
            "  sigma_accel_c: 2.0e-3\n"
            "  sigma_gyro_bias: 1.9393e-5\n"
            "  sigma_accel_bias: 3.0e-3\n"
            "  sigma_gyro_bias_walk_c: 1.0e-5\n"
            "  sigma_accel_bias_walk_c: 1.0e-4\n"
            "  rate_hz: 200\n"
            "  gravity_mps2: 9.80665\n"
            "sonar_beam_models:\n"
            "  - sensor_id: sonar0\n"
            "    horizontal_fov_rad: 2.09\n"
            "    elevation_aperture_rad: 0.19\n"
            "    range_resolution_m: 0.02\n"
            "    nominal_speed_of_sound_mps: 1500.0\n"
            "    sonar_enabled: true\n"
            "depth_models:\n"
            "  - sensor_id: depth0\n"
            "    noise_sigma_m: 0.05\n"
            "    depth_enabled: true\n"
            "time_offset_seconds:\n"
            "  sonar0: 0.0\n"
            "  imu0: 0.0\n"
            "  depth0: 0.0\n"
            "  rov-state: 0.0\n"
            "time_offset_provenance:\n"
            "  sonar0: measured:test-clock\n"
            "  imu0: measured:test-clock\n"
            "  depth0: measured:test-clock\n"
            "  rov-state: measured:test-clock\n");
  WriteFile(root / "scenario" / "imu_scenario.yaml", "seed: 1\nnum_keyframes: 3\n");
  WriteFile(root / "defaults" / "imu_defaults.yaml",
            "gates:\n"
            "  require_converged: false\n"
            "  require_nonempty_map: false\n");
  WriteFile(root / "experiment" / "imu_experiment.yaml",
            "rig: rig/imu_rig.yaml\n"
            "scenario: scenario/imu_scenario.yaml\n"
            "defaults: defaults/imu_defaults.yaml\n"
            "estimator_mode: imu_preintegration\n"
            "factor_builders:\n"
            "  - imu_preintegration_v1\n"
            "  - sonar_range_v1\n"
            "  - depth_v1\n"
            "output:\n"
            "  write_run_manifest: false\n");

  const double last_keyframe_s =
      kFixturePreRollS + (options.num_keyframes - 1) * kFixtureKeyframePeriodS;
  auto to_ns = [](double seconds) {
    return static_cast<uint64_t>(std::llround(seconds * 1e9));
  };

  McapProtobufWriter writer;
  ASSERT_TRUE(writer.Open((root / "imu.mcap").string()));

  // Stationary for the whole run: specific force is pure gravity on body z,
  // angular rate is exactly zero.
  const int imu_sample_count = static_cast<int>(std::llround(last_keyframe_s * kFixtureImuRateHz));
  for (int i = 0; i <= imu_sample_count; ++i) {
    const double time_s = static_cast<double>(i) / kFixtureImuRateHz;
    if (options.imu_gap_start_s >= 0.0 && time_s >= options.imu_gap_start_s &&
        time_s < options.imu_gap_start_s + options.imu_gap_duration_s) {
      continue;
    }
    uw::domain::ImuSample sample;
    *sample.mutable_header() =
        MakeFixtureHeader("imu_" + std::to_string(i), "imu0", "imu_link", time_s);
    for (double value : {0.0, 0.0, kFixtureGravity}) sample.add_linear_acceleration_mps2(value);
    for (int axis = 0; axis < 3; ++axis) sample.add_angular_velocity_radps(0.0);
    ASSERT_TRUE(writer.WriteMessage("/raw/imu", to_ns(time_s), sample));
  }

  for (int i = 0; i < options.num_keyframes; ++i) {
    const double time_s = kFixturePreRollS + i * kFixtureKeyframePeriodS;
    const uint64_t time_ns = to_ns(time_s);
    const std::string kf_id = "kf" + std::to_string(i);

    if (options.write_keyframe_boundaries) {
      uw::domain::KeyframeBoundary boundary;
      *boundary.mutable_header() =
          MakeFixtureHeader("boundary_" + kf_id, "keyframe_scheduler", "base_link", time_s);
      boundary.mutable_keyframe_id()->set_value(kf_id);
      boundary.set_source("test_fixed_interval_v1");
      ASSERT_TRUE(writer.WriteMessage("/keyframe/boundary", time_ns, boundary));
    }

    if (options.write_ground_truth) {
      uw::domain::StateSnapshot gt;
      gt.mutable_state_id()->set_value(kf_id);
      *gt.mutable_capture_timestamp() =
          uw::domain::FromSeconds(time_s + options.ground_truth_time_offset_s);
      uw::sensor_models::Pose3 pose;
      pose.translation = Eigen::Vector3d(options.ground_truth_pose_offset_m,
                                          options.ground_truth_pose_offset_m,
                                          -kFixtureDepthM + options.ground_truth_pose_offset_m);
      *gt.mutable_pose_wb() = pose.ToProto();
      // Log time deliberately left at the untampered instant.
      ASSERT_TRUE(writer.WriteMessage("/gt/state", time_ns, gt));
    }

    uw::domain::PressureDepthMeasurement depth;
    depth.set_depth_m(kFixtureDepthM);
    depth.set_sigma_m(0.05);
    uw::domain::EvidenceId depth_id;
    depth_id.set_value("depth_" + kf_id);
    uw::domain::ObservationId depth_source;
    depth_source.set_value(kf_id);
    ASSERT_TRUE(writer.WriteMessage(
        "/evidence/depth", time_ns,
        uw::domain::MakeEvidence(depth_id, {depth_source}, depth, 1.0, "test_depth_v1")));

    // One sonar ping per keyframe at a fixed range/bearing: the CFAR
    // frontend has to actually detect it for a sonar range factor to exist,
    // so this reuses the same renderer synth_bag_gen does.
    uw::runtime::SyntheticSonarFrameSpec sonar_spec;
    sonar_spec.sensor_id = "sonar0";
    sonar_spec.provenance = "test_sonar_v1";
    sonar_spec.observation_id = kf_id;
    sonar_spec.timestamp_ns = time_ns;
    const auto rendered = uw::runtime::RenderSyntheticSonarFrame(sonar_spec, 5.0, 0.0);
    ASSERT_TRUE(rendered.target_rendered);
    ASSERT_TRUE(writer.WriteMessage("/raw/sonar_frame", time_ns, rendered.frame));

    if (options.write_conflicting_relative_pose && i > 0) {
      uw::domain::RelativePoseMeasurement relative_pose;
      relative_pose.mutable_from_keyframe()->set_value("kf" + std::to_string(i - 1));
      relative_pose.mutable_to_keyframe()->set_value(kf_id);
      uw::sensor_models::Pose3 relative;
      // Grossly inconsistent with "stationary": 2 m per 0.2 s keyframe.
      relative.translation = Eigen::Vector3d(2.0, 0.0, 0.0);
      *relative_pose.mutable_relative_pose() = relative.ToProto();
      uw::domain::EvidenceId relative_id;
      relative_id.set_value("relpose_" + kf_id);
      ASSERT_TRUE(writer.WriteMessage(
          "/evidence/relative_pose", time_ns,
          uw::domain::MakeEvidence(relative_id, {}, relative_pose, 1.0, "test_relpose_v1")));
    }
  }
  writer.Close();
}

struct ImuRunResult {
  int exit_code = 0;
  std::string stdout_text;
  std::string stderr_text;
  std::string trajectory;
};

ImuRunResult RunImuFixture(const std::filesystem::path& root, const ImuFixtureOptions& options) {
  std::filesystem::remove_all(root);
  WriteImuFixture(root, options);

  uw::application::ReplayOptions replay_options;
  replay_options.bag_path = (root / "imu.mcap").string();
  replay_options.experiment_path = (root / "experiment" / "imu_experiment.yaml").string();
  replay_options.out_prefix = (root / "out").string();

  ImuRunResult result;
  {
    CoutCapture out_capture;
    CerrCapture err_capture;
    result.exit_code = uw::application::RunReplayPipeline(replay_options, "test-commit");
    result.stdout_text = out_capture.str();
    result.stderr_text = err_capture.str();
  }
  std::ifstream trajectory(root / "out_trajectory.tum");
  std::ostringstream contents;
  contents << trajectory.rdbuf();
  result.trajectory = contents.str();
  return result;
}

}  // namespace

TEST(ReplayPipelineImu, AssemblesTheGraphFromBoundariesImuDepthAndSonarAlone) {
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_imu_assembly";
  ImuFixtureOptions options;  // no ground truth, no relative-pose evidence
  const auto run = RunImuFixture(root, options);
  ASSERT_EQ(run.exit_code, 0) << run.stdout_text << run.stderr_text;

  EXPECT_EQ(SummaryValue(run.stdout_text, "keyframe_boundary_count"), "3") << run.stdout_text;
  EXPECT_EQ(SummaryValue(run.stdout_text, "imu_factor_count"), "2") << run.stdout_text;
  EXPECT_EQ(SummaryValue(run.stdout_text, "relative_pose_factor_count"), "0") << run.stdout_text;
  EXPECT_EQ(SummaryValue(run.stdout_text, "initialization"), "stationary") << run.stdout_text;
  EXPECT_GT(std::stoi(SummaryValue(run.stdout_text, "depth_factor_count")), 0) << run.stdout_text;
  EXPECT_GT(std::stoi(SummaryValue(run.stdout_text, "sonar_range_factor_count")), 0)
      << run.stdout_text;
  EXPECT_EQ(SummaryValue(run.stdout_text, "keyframe_count"), "3") << run.stdout_text;

  std::filesystem::remove_all(root);
}

TEST(ReplayPipelineImu, ProducesTheSameTrajectoryWithNoGroundTruthCorrectGroundTruthAndTamperedGroundTruth) {
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_imu_gt";

  ImuFixtureOptions without_gt;
  const auto no_gt = RunImuFixture(root, without_gt);
  ASSERT_EQ(no_gt.exit_code, 0) << no_gt.stdout_text << no_gt.stderr_text;
  ASSERT_FALSE(no_gt.trajectory.empty());

  ImuFixtureOptions with_gt;
  with_gt.write_ground_truth = true;
  const auto correct_gt = RunImuFixture(root, with_gt);
  ASSERT_EQ(correct_gt.exit_code, 0) << correct_gt.stdout_text << correct_gt.stderr_text;

  ImuFixtureOptions moved_gt;
  moved_gt.write_ground_truth = true;
  moved_gt.ground_truth_pose_offset_m = 3.0;
  const auto displaced_gt = RunImuFixture(root, moved_gt);
  ASSERT_EQ(displaced_gt.exit_code, 0) << displaced_gt.stdout_text << displaced_gt.stderr_text;

  ImuFixtureOptions tampered;
  tampered.write_ground_truth = true;
  tampered.ground_truth_time_offset_s = 5.0;
  tampered.ground_truth_pose_offset_m = 3.0;
  const auto tampered_gt = RunImuFixture(root, tampered);
  ASSERT_EQ(tampered_gt.exit_code, 0) << tampered_gt.stdout_text << tampered_gt.stderr_text;

  // Byte-for-byte, not near-equal: ground truth must reach the evaluator
  // and nothing else, so no amount of moving it can perturb a single digit
  // of the estimate -- including the timestamps, which in this mode come
  // from /keyframe/boundary rather than from /gt/state.
  EXPECT_EQ(correct_gt.trajectory, no_gt.trajectory);
  EXPECT_EQ(displaced_gt.trajectory, no_gt.trajectory);
  EXPECT_EQ(tampered_gt.trajectory, no_gt.trajectory);

  // ...and every one of those tampering axes was real, i.e. the evaluator,
  // which IS allowed to read ground truth, saw exactly what was changed.
  // Without this the equalities above would also hold if the pipeline had
  // quietly ignored the ground truth branch altogether.
  EXPECT_EQ(SummaryValue(no_gt.stdout_text, "ate_matched_poses"), "0") << no_gt.stdout_text;
  EXPECT_EQ(SummaryValue(correct_gt.stdout_text, "ate_matched_poses"), "3")
      << correct_gt.stdout_text;
  EXPECT_LT(std::stod(SummaryValue(correct_gt.stdout_text, "ate_rmse_m")), 1e-6)
      << correct_gt.stdout_text;
  // Moving the poses moves the reported error...
  EXPECT_EQ(SummaryValue(displaced_gt.stdout_text, "ate_matched_poses"), "3")
      << displaced_gt.stdout_text;
  EXPECT_GT(std::stod(SummaryValue(displaced_gt.stdout_text, "ate_rmse_m")), 1.0)
      << displaced_gt.stdout_text;
  // ...and moving their timestamps by 5 s puts them outside the evaluator's
  // matching window entirely.
  EXPECT_EQ(SummaryValue(tampered_gt.stdout_text, "ate_matched_poses"), "0")
      << tampered_gt.stdout_text;

  std::filesystem::remove_all(root);
}

TEST(ReplayPipelineImu, IgnoresRelativePoseEvidenceThatContradictsTheImuStream) {
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_imu_relpose";

  ImuFixtureOptions baseline;
  const auto without = RunImuFixture(root, baseline);
  ASSERT_EQ(without.exit_code, 0) << without.stdout_text << without.stderr_text;

  ImuFixtureOptions conflicting;
  conflicting.write_conflicting_relative_pose = true;
  const auto with = RunImuFixture(root, conflicting);
  ASSERT_EQ(with.exit_code, 0) << with.stdout_text << with.stderr_text;

  EXPECT_EQ(SummaryValue(with.stdout_text, "relative_pose_factor_count"), "0") << with.stdout_text;
  EXPECT_EQ(with.trajectory, without.trajectory)
      << "relative-pose evidence claiming 2 m per keyframe must not move a stationary solution";

  std::filesystem::remove_all(root);
}

TEST(ReplayPipelineImu, FailsClosedWhenAnImuGapWouldTruncateTheTrajectory) {
  // A dropout longer than the frontend's max_sample_gap_s costs the
  // interval its edge. With no other relative-motion source in this sensor
  // set, the destination keyframe -- and everything after it -- then never
  // enters the graph, so the run would write a SHORTER trajectory and score
  // an ATE over that prefix alone, which can read better than the full run
  // rather than worse. That has to be a failure, not a quietly different
  // answer.
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_imu_gap";
  ImuFixtureOptions options;
  options.write_ground_truth = true;
  // Inside the kf1 -> kf2 interval (boundaries at 0.75 / 0.95 / 1.15 s).
  options.imu_gap_start_s = 1.00;
  options.imu_gap_duration_s = 0.10;  // 20 samples at 200 Hz, well over the 50 ms limit
  const auto run = RunImuFixture(root, options);
  EXPECT_NE(run.exit_code, 0) << run.stdout_text;
  EXPECT_NE(run.stdout_text.find("imu interval kf1 -> kf2 rejected"), std::string::npos)
      << run.stdout_text;
  EXPECT_NE(run.stderr_text.find("keyframe boundaries made it into the graph"), std::string::npos)
      << run.stderr_text;
  // The rejected interval must not have left kf2 behind as a free, wholly
  // unconstrained inertial state -- that would turn a recoverable rejection
  // into a structural-singularity failure with a misleading message.
  EXPECT_EQ(run.stderr_text.find("STRUCTURALLY SINGULAR GRAPH"), std::string::npos)
      << run.stderr_text;
  std::filesystem::remove_all(root);
}

TEST(ReplayPipelineImu, FailsClosedWithoutEnoughKeyframeBoundariesEvenWithCompleteGroundTruth) {
  const auto root = std::filesystem::temp_directory_path() / "uw_replay_imu_no_boundary";
  ImuFixtureOptions options;
  options.write_keyframe_boundaries = false;
  options.write_ground_truth = true;  // complete, and still not a substitute
  const auto run = RunImuFixture(root, options);
  EXPECT_NE(run.exit_code, 0) << run.stdout_text;
  EXPECT_NE(run.stderr_text.find("imu_preintegration: fewer than two keyframe boundaries"),
            std::string::npos)
      << run.stderr_text;
  std::filesystem::remove_all(root);
}

// ---------------------------------------------------------------------------
// The structural check and the summary format are the two things Task 6's
// smoke test depends on, so they get direct tests rather than only being
// exercised through a whole pipeline run -- in particular the singular-graph
// case, which the pipeline itself can no longer produce (it only ever adds
// an inertial state together with the edge or prior that constrains it) and
// which would therefore never be covered by an end-to-end test.
namespace {

// A stand-in for any pose-pair edge (relative pose, loop closure): 6 rows
// on two 7-parameter pose blocks. Only its shape matters here.
class PosePairEdgeStub : public uw::measurement_api::ResidualBlock {
 public:
  int ResidualDim() const override { return 6; }
  std::vector<int> ParameterBlockSizes() const override { return {7, 7}; }
  bool Evaluate(const std::vector<const double*>&, double* residuals,
                std::vector<double*>*) const override {
    for (int i = 0; i < 6; ++i) residuals[i] = 0.0;
    return true;
  }
};

std::unique_ptr<uw::measurement_api::ResidualBlock> MakeUnitInertialPrior() {
  Eigen::Matrix<double, 9, 1> target = Eigen::Matrix<double, 9, 1>::Zero();
  Eigen::Matrix<double, 9, 1> sigma = Eigen::Matrix<double, 9, 1>::Ones();
  return uw::factor_builders::InertialPriorResidual::Create(target, sigma);
}

}  // namespace

TEST(CheckGraphObservability, AcceptsAnAnchoredInertialStateHeldByItsPrior) {
  uw::estimation::PoseGraphProblem problem;
  problem.AddKeyframe("kf0", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  problem.AddInertialState("kf0", {});
  problem.AddResidualBlockOnParameters(
      MakeUnitInertialPrior(), {uw::estimation::PoseGraphProblem::InertialRef("kf0")});

  const auto observability = uw::application::CheckGraphObservability(problem);
  EXPECT_TRUE(observability.problems.empty())
      << (observability.problems.empty() ? "" : observability.problems.front());
  // The pose is fixed, so only the 9 inertial columns are free, and the
  // prior supplies exactly 9 rows.
  EXPECT_EQ(observability.free_parameter_dim, 9);
  EXPECT_EQ(observability.residual_dim, 9);
}

TEST(CheckGraphObservability, FlagsAnInertialStateThatNoResidualReaches) {
  uw::estimation::PoseGraphProblem problem;
  problem.AddKeyframe("kf0", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  problem.AddInertialState("kf0", {});
  problem.AddResidualBlockOnParameters(
      MakeUnitInertialPrior(), {uw::estimation::PoseGraphProblem::InertialRef("kf0")});
  // A second keyframe whose inertial state exists but is reached by nothing:
  // this is the shape a dropped IMU interval would leave behind.
  problem.AddKeyframe("kf1", uw::sensor_models::Pose3::Identity());
  problem.AddInertialState("kf1", {});

  const auto observability = uw::application::CheckGraphObservability(problem);
  ASSERT_FALSE(observability.problems.empty());
  bool mentions_inertial_kf1 = false;
  bool mentions_pose_kf1 = false;
  for (const auto& problem_text : observability.problems) {
    if (problem_text.find("inertial state 'kf1'") != std::string::npos) mentions_inertial_kf1 = true;
    if (problem_text.find("keyframe pose 'kf1'") != std::string::npos) mentions_pose_kf1 = true;
  }
  EXPECT_TRUE(mentions_inertial_kf1);
  // The pose block of the same keyframe is a SEPARATE parameter and is
  // equally unconstrained here; reporting only one of the two would let the
  // other slip through on a graph where just one is missing.
  EXPECT_TRUE(mentions_pose_kf1);
}

TEST(CheckGraphObservability, FlagsAnUnderdeterminedGraphEvenWhenEveryBlockIsReferenced) {
  uw::estimation::PoseGraphProblem problem;
  problem.AddKeyframe("kf0", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  // 7 free pose columns, reached only by a 1-row depth-style residual would
  // be ideal; the prior on an inertial block gives the same shape with less
  // machinery: 9 free inertial columns + 7 free pose columns = 16, against
  // 9 residual rows.
  problem.AddKeyframe("kf1", uw::sensor_models::Pose3::Identity());
  problem.AddInertialState("kf1", {});
  problem.AddResidualBlockOnParameters(
      MakeUnitInertialPrior(), {uw::estimation::PoseGraphProblem::InertialRef("kf1")});

  const auto observability = uw::application::CheckGraphObservability(problem);
  // 6 minimal DOF for the free pose + 9 for the inertial block.
  EXPECT_EQ(observability.free_parameter_dim, 15);
  EXPECT_EQ(observability.residual_dim, 9);
  bool mentions_underdetermined = false;
  for (const auto& problem_text : observability.problems) {
    if (problem_text.find("underdetermined") != std::string::npos) mentions_underdetermined = true;
  }
  EXPECT_TRUE(mentions_underdetermined);
}

TEST(CheckGraphObservability, AcceptsAPlainRelativePoseChainWithNoOtherFactors) {
  // The regression this guards: a pose is STORED as 7 numbers but has 6
  // degrees of freedom. Counting the stored size would make every fully
  // determined relative-pose chain -- 6 rows per edge, one free pose per
  // edge -- look underdetermined and hard-fail modes that carry no depth or
  // sonar factors at all.
  uw::estimation::PoseGraphProblem problem;
  problem.AddKeyframe("kf0", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  for (int i = 1; i <= 4; ++i) {
    const std::string previous = "kf" + std::to_string(i - 1);
    const std::string current = "kf" + std::to_string(i);
    problem.AddKeyframe(current, uw::sensor_models::Pose3::Identity());
    problem.AddResidualBlock(std::make_unique<PosePairEdgeStub>(), {previous, current});
  }
  const auto observability = uw::application::CheckGraphObservability(problem);
  EXPECT_EQ(observability.free_parameter_dim, 24);  // 4 free poses x 6 DOF
  EXPECT_EQ(observability.residual_dim, 24);        // 4 edges x 6 rows
  EXPECT_TRUE(observability.problems.empty())
      << (observability.problems.empty() ? "" : observability.problems.front());
}

TEST(CheckGraphObservability, IgnoresResidualsWiredOnlyToFixedBlocks) {
  // An edge between two fixed keyframes is a constant. Counting its rows
  // would inflate the tally and hide a rank deficiency elsewhere.
  uw::estimation::PoseGraphProblem problem;
  problem.AddKeyframe("kf0", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  problem.AddKeyframe("kf1", uw::sensor_models::Pose3::Identity(), /*fixed=*/true);
  problem.AddResidualBlock(std::make_unique<PosePairEdgeStub>(), {"kf0", "kf1"});
  problem.AddKeyframe("kf2", uw::sensor_models::Pose3::Identity());
  problem.AddResidualBlock(std::make_unique<PosePairEdgeStub>(), {"kf1", "kf2"});

  const auto observability = uw::application::CheckGraphObservability(problem);
  EXPECT_EQ(observability.free_parameter_dim, 6);
  EXPECT_EQ(observability.residual_dim, 6) << "the fixed-only edge must not be counted";
  EXPECT_TRUE(observability.problems.empty())
      << (observability.problems.empty() ? "" : observability.problems.front());
}

TEST(FormatReplayRunSummary, EmitsOneAnchoredKeyValueLinePerField) {
  uw::application::ReplayRunSummary summary;
  summary.estimator_mode = "imu_preintegration";
  summary.solver = "gauss_newton_v1";
  summary.solver_converged = true;
  summary.solver_iterations = 4;
  summary.keyframe_boundary_count = 12;
  summary.imu_factor_count = 11;
  summary.relative_pose_factor_count = 0;
  summary.initialization = "stationary";
  summary.ate_rmse_m = 0.0858023641;

  const std::string text = uw::application::FormatReplayRunSummary(summary);
  EXPECT_NE(text.find("summary.estimator_mode=imu_preintegration\n"), std::string::npos) << text;
  EXPECT_NE(text.find("summary.solver_converged=true\n"), std::string::npos) << text;
  EXPECT_NE(text.find("summary.keyframe_boundary_count=12\n"), std::string::npos) << text;
  EXPECT_NE(text.find("summary.imu_factor_count=11\n"), std::string::npos) << text;
  EXPECT_NE(text.find("summary.relative_pose_factor_count=0\n"), std::string::npos) << text;
  EXPECT_NE(text.find("summary.initialization=stationary\n"), std::string::npos) << text;
  // Fixed-point, enough digits to compare a 0.15 m gate against without a
  // parser having to cope with scientific notation.
  EXPECT_NE(text.find("summary.ate_rmse_m=0.085802364\n"), std::string::npos) << text;
  // A key that is a prefix of nothing else, and every line self-terminating:
  // this is what lets the smoke test anchor on whole `key=` strings.
  EXPECT_EQ(text.back(), '\n');
}
