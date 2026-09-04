// Stationary IMU initialization (PREP-B-01,
// docs/imu-preintegration-design-2026-09-03.md section 7): reads the IMU
// samples recorded before the first keyframe boundary and produces the
// initial velocity, gyro/accel biases, and gravity-aligned attitude for the
// anchor keyframe, together with the sigmas of the prior that will hold
// them.
//
// Why this exists at all: in the phase-1 sensor set (mono + sonar + IMU +
// depth, no DVL) there is no other legitimate source for the initial
// biases or the gravity direction. Taking them from /gt/state or from
// relative-pose evidence is exactly the leak PREP-B-01 closes off, so they
// have to be measured from a stretch of data where the vehicle is known to
// be still.
//
// Three things about the output are easy to get wrong and are therefore
// spelled out here:
//
// 1. STATIONARITY IS JUDGED ON THE WINDOW MEAN, NOT PER SAMPLE. At 200 Hz
//    with the rig's own accelerometer density (2.0e-3 m/s^2/sqrt(Hz)
//    discretises to ~0.028 m/s^2 per axis) roughly 8% of individual
//    samples fall outside the 0.05 m/s^2 gravity-magnitude gate purely
//    from white noise, while the 0.5 s mean sits two orders of magnitude
//    inside it. A per-sample gate would refuse to initialize a perfectly
//    good stationary recording.
//
// 2. ONLY THE ALONG-GRAVITY PART OF THE ACCELEROMETER BIAS IS OBSERVABLE
//    HERE. A stationary accelerometer cannot tell a horizontal bias from a
//    tilt: both rotate the measured specific force away from the body z
//    axis. This code resolves that ambiguity the standard way -- direction
//    goes to attitude, magnitude goes to bias -- so bias_accel comes out
//    parallel to the measured specific force. That is a modelling choice,
//    not an approximation error, and it is why the estimator must keep the
//    accel bias free rather than fixing it here.
//
// 3. YAW IS NOT OBSERVABLE FROM GRAVITY and stays at the canonical zero.
//    rotation_WB is the minimal rotation taking the measured specific
//    force onto world up, which has no component about the world z axis by
//    construction. PREP-B-02's heading factor is what gives yaw an absolute
//    reference.
//
// Failure is graded, not binary. A window that is too short or not actually
// still still yields a usable initialization -- zero velocity as an INITIAL
// VALUE under a deliberately wide prior, and the rig's own zero-mean bias
// prior -- flagged as kWideVelocityPrior so the run diagnostics can say so.
// What does NOT yield anything is a rig whose imu_noise cannot define a
// prior at all, or a malformed reading inside the window: those return
// nullopt and the caller must fail closed rather than invent a sigma.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "domain/domain.hpp"

namespace uw::frontends {

struct ImuStationaryInitializerParams {
  // Samples whose header.sensor_id differs are ignored; empty accepts all.
  std::string imu_sensor_id = "imu0";
  // rig frame_tree child frame giving base_link_T_imu_link. Readings are
  // rotated into base_link with it; the lever arm is irrelevant here
  // because a stationary body has no centripetal term to correct.
  std::string imu_frame = "imu_link";
  bool require_extrinsic = false;

  // How far back from the boundary to look. This bound is not cosmetic: a
  // real recording may manoeuvre for minutes before its first keyframe and
  // only then settle, and oscillatory motion averages toward zero, so an
  // unbounded window can pass the criterion below on data that is mostly
  // motion -- and would then report a window_duration_s of minutes, whose
  // sqrt shrinks the measured-bias standard error far below what the data
  // supports. Kept larger than min_stationary_duration_s so a producer that
  // supplies a little more than the minimum is not judged on an exactly
  // minimum-length window.
  double stationary_window_s = 1.0;
  // The stationary criterion of the design note, section 7.
  double min_stationary_duration_s = 0.5;
  double max_gyro_mean_norm_radps = 0.01;
  double max_accel_mean_norm_deviation_mps2 = 0.05;
  int min_samples = 2;

  // Velocity prior sigma when the criterion is NOT met: zero velocity is
  // then an initial value only and must not be allowed to fight the data.
  double wide_velocity_sigma_mps = 0.5;
  // Velocity prior sigma when it IS met. The vehicle is verified at rest,
  // so zero is a measurement rather than a guess; this is still set two
  // orders of magnitude above the white-noise floor of the window
  // (sigma_accel_c * sqrt(window) is ~1.4e-3 m/s at the rig's density) so
  // a slowly drifting mooring or a mild current is absorbed instead of
  // fought.
  double stationary_velocity_sigma_mps = 0.05;
};

struct ImuStationaryInitialization {
  enum class Mode {
    // The window met the stationary criterion: biases and gravity
    // direction are measured, velocity is known to be zero.
    kStationary,
    // It did not: velocity is zero only as an initial value, under a wide
    // prior, and the biases fall back to the rig's zero-mean prior.
    kWideVelocityPrior,
  };

  // world_R_body. Roll/pitch from the measured gravity direction, yaw at
  // the canonical zero (see the file comment). Identity in
  // kWideVelocityPrior mode, where nothing was measured.
  Eigen::Quaterniond rotation_WB = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
  // Prior sigmas laid out as the inertial parameter block is:
  // [v(3), bg(3), ba(3)]. Feed straight into InertialPriorResidual.
  Eigen::Matrix<double, 9, 1> sigma = Eigen::Matrix<double, 9, 1>::Zero();
  Mode mode = Mode::kWideVelocityPrior;

  // How many well-formed samples the window held, its length in seconds,
  // and the two statistics the criterion was judged on -- reported whether
  // or not it passed, so a run diagnostic can say by how much.
  int sample_count = 0;
  double window_duration_s = 0.0;
  double gyro_mean_norm_radps = 0.0;
  double accel_mean_norm_mps2 = 0.0;
  // Human-readable reason, empty on kStationary.
  std::string detail;
};

// `samples` may be the whole bag's IMU stream; only those in
// (window_end_time_s - params.stationary_window_s, window_end_time_s] are
// considered. The upper bound is inclusive because the first keyframe
// boundary is itself the last instant at which the vehicle is still known
// to be still -- dropping it would shorten every window by one sample. The
// lower bound is what keeps a long pre-boundary manoeuvre from averaging
// itself into looking stationary.
//
// Returns nullopt only when no prior can be defined at all: the rig's
// imu_noise white-noise densities or bias prior sigmas are non-finite or
// non-positive, a required extrinsic is missing, or a reading inside the
// window is malformed.
std::optional<ImuStationaryInitialization> InitializeFromStationaryWindow(
    const std::vector<uw::domain::ImuSample>& samples, double window_end_time_s,
    const uw::domain::RigCalibrationSnapshot& rig,
    const ImuStationaryInitializerParams& params = ImuStationaryInitializerParams());

}  // namespace uw::frontends
