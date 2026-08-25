// Portable (no ROS2/HoloOcean headers) conversion from raw HoloOcean AI-D
// sensor readings, already unpacked from their real ROS2 message shapes by
// adapters/ros2/include/adapters/ros2_holoocean_realtime_gateway.hpp, into
// this platform's canonical uw::domain messages -- the realtime-gateway
// counterpart to include/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp,
// but targeting uw::runtime::CanonicalEvent/LiveEventSource directly instead
// of a poll-based measurement_api::SonarFrameProvider queue: the realtime
// gateway pushes straight into LiveEventSource on each ROS2 callback, so
// there is no queue for this layer to own.
//
// Unlike the offline/replay HoloOcean Python adapter
// (adapters/holoocean/uw_holoocean_adapter/camera_conversion.py), this does
// NOT apply the BGR(A)->RGB channel fix documented there -- by the time a
// frame reaches this C++ boundary it has already crossed Task 3's
// realtime_ros_session.py (uw_holoocean_adapter/ros_message_conversion.py's
// holoocean_camera_to_ros_image), which already performed that fix before
// publishing sensor_msgs/Image. RawHoloImage.rgb_pixel_data is expected to
// already be RGB8, row-major, no padding.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/domain.hpp"

namespace uw::adapters {

// One already-RGB8 camera frame, unpacked from a real sensor_msgs/Image.
struct RawHoloImage {
  int64_t capture_ns = 0;  // HoloOcean simulation time, nanoseconds
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<uint8_t> rgb_pixel_data;  // row-major RGB8, size == width*height*3
};

// Converts one already-RGB8 realtime camera frame into a uw::domain::ImageFrame.
// `sequence_id` is a per-sensor monotonically increasing counter the caller
// owns (the gateway node's own receive counter, not anything HoloOcean
// provides). `calibration_version` should mirror the same rig/manifest
// version string used elsewhere in this run (see RigCalibrationSnapshot).
// `receive_monotonic_ns` is the gateway's own local steady-clock receive
// time, nanoseconds since that clock's epoch.
uw::domain::ImageFrame ConvertHoloImage(const RawHoloImage& raw, const std::string& sensor_id,
                                         const std::string& sensor_frame, uint64_t sequence_id,
                                         const std::string& calibration_version,
                                         int64_t receive_monotonic_ns);

// One already-unpacked holoocean_interfaces/msg/ImagingSonar reading: flat
// row-major [num_ranges, num_beams] intensity array, float32 in [0, 1] (see
// holoocean_ros_bridge_sonar_frame_provider.hpp's header comment for the
// real-hardware-confirmed field mapping and mirror-flip this mirrors).
struct RawHoloSonar {
  int64_t capture_ns = 0;  // HoloOcean simulation time, nanoseconds (msg.timestamp)
  uint32_t num_ranges = 0;
  uint32_t num_beams = 0;
  std::vector<float> image_range;  // size == num_ranges * num_beams
};

// Sonar geometry/calibration this platform's canonical SonarFrame needs but
// holoocean_interfaces/msg/ImagingSonar does not itself carry -- sourced from
// ROS2 node parameters mirroring Task 1's scenario manifest
// (adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json), never
// hardcoded defaults (see HoloOceanLiveConversion.
// SonarUsesManifestCalibrationNotHardcodedDefaults).
struct HoloOceanSonarCalibration {
  float horizontal_fov_rad = 0.0f;
  float min_range_m = 0.0f;
  float max_range_m = 0.0f;
  float elevation_aperture_rad = 0.0f;
  double operating_frequency_hz = 0.0;
  float gain_db = 0.0f;
  uint32_t gain_mode = 0;
  float sound_speed_mps = 0.0f;
  float salinity_ppt = 0.0f;
  bool sound_speed_is_measured = false;
  std::string sensor_id = "holoocean_imaging_sonar";
  std::string sensor_frame = "sonar_link";
  std::string calibration_version;
};

// Converts one raw sonar reading into a uw::domain::SonarFrame. Applies the
// same mirror flip / ascending-bearing construction as
// HoloOceanRosBridgeSonarFrameProvider::PushImagingSonar (HoloOcean's raw
// column order runs opposite this platform's ascending-bearing convention)
// and fills the elevation_aperture/operating_frequency_hz/gain_metadata/
// sound_speed_assumption fields ValidateCanonicalEvent requires -- omitting
// those is a real, previously-hit failure mode (silently rejects every
// sonar event with "sonar range and field-of-view geometry must be finite
// and ordered"; see docs/superpowers/plans/2026-08-24-acoustic-optic-
// online-tracking.md Task 8 notes).
uw::domain::SonarFrame ConvertHoloSonar(const RawHoloSonar& raw,
                                         const HoloOceanSonarCalibration& calibration,
                                         uint64_t sequence_id, int64_t receive_monotonic_ns);

// One already-unpacked nav_msgs/Odometry reading, as published by Task 3's
// vehicle_state_to_odometry (noisy VehicleOrientation+IMUSensor+DepthSensor
// composite -- never the ground-truth PoseSensor/ScoringPose channel).
struct RawHoloVehicleState {
  int64_t capture_ns = 0;  // HoloOcean simulation time, nanoseconds
  double orientation_xyzw[4] = {0.0, 0.0, 0.0, 1.0};
  double angular_velocity_radps[3] = {0.0, 0.0, 0.0};
  // Raw pose.position.z from the Odometry message -- HoloOcean's own raw
  // world-frame z (negative underwater), NOT yet the positive-down depth_m
  // wire convention (see vehicle_state_to_odometry's docstring in
  // ros_message_conversion.py: it deliberately passes this through
  // un-negated, matching state_conversion.py's depth_sensor_to_evidence raw
  // input). ConvertHoloVehicleState below negates it into VehicleState's
  // documented positive-down depth_m -- omitting that negation is exactly
  // the real bug CLAUDE.md's "已经踩过的坑" records for the offline path.
  double raw_position_z_m = 0.0;
};

// Converts one raw vehicle-state reading into a uw::domain::VehicleState.
// Only attitude_valid/depth_valid are set true (this gateway has no signal
// for leak/power/link telemetry -- Task 3's realtime_ros_session.py does not
// publish any, so device_health_valid stays false rather than fabricating a
// value with false precision).
uw::domain::VehicleState ConvertHoloVehicleState(const RawHoloVehicleState& raw,
                                                  const std::string& sensor_id,
                                                  const std::string& sensor_frame,
                                                  uint64_t sequence_id,
                                                  const std::string& calibration_version,
                                                  int64_t receive_monotonic_ns);

}  // namespace uw::adapters
