#include "application/replay_pipeline.hpp"

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/mcap_io.hpp"
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
            "  camera_right: 0.0\n");

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
