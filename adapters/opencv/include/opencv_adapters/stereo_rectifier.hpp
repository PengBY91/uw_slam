// opencv_adapters: the ONLY place in this repo allowed to depend on OpenCV
// (tools/lint/check_layer_dependencies.py enforces this). This public header
// must never expose a cv:: type or an opencv2/... include — callers outside
// this adapter only see repo-native domain/measurement_api types.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "domain/domain.hpp"
#include "measurement_api/frontend.hpp"

namespace uw::opencv_adapters {

enum class RectificationCropPolicy { kFullCanvas, kCommonValidRoi };

struct StereoRectificationParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  std::string rectified_frame_suffix = "_rectified";
  double alpha = 0.0;
  RectificationCropPolicy crop_policy = RectificationCropPolicy::kFullCanvas;
};

struct RectifiedStereoBundle {
  uw::measurement_api::CameraFrameBundle images;
};

// Builds once per rig (raw_rig + params -> validated, immutable rectification
// plan), then reused across many frame pairs via Process(). Construction does
// all the fallible calibration work (input validation, OpenCV stereo
// calibration conversion) so per-frame Process() calls only do image I/O and
// cannot fail for calibration reasons.
class StereoRectificationContext {
 public:
  static std::optional<StereoRectificationContext> Create(
      const uw::domain::RigCalibrationSnapshot& raw_rig,
      const StereoRectificationParams& params,
      std::string* error);

  std::optional<RectifiedStereoBundle> Process(
      const uw::domain::ImageFrame& left,
      const uw::domain::ImageFrame& right,
      std::string* error) const;

  // A rig identical to raw_rig except the left/right camera calibration
  // entries are replaced by their rectified equivalents and two virtual
  // frame edges are appended for LeftRectifiedFrame()/RightRectifiedFrame().
  // Everything else (sonar/IMU/depth models, time offsets, other cameras)
  // is carried through unchanged.
  const uw::domain::RigCalibrationSnapshot& DerivedRig() const;
  const std::string& LeftRectifiedFrame() const;
  const std::string& RightRectifiedFrame() const;

 private:
  class Impl;
  explicit StereoRectificationContext(std::shared_ptr<const Impl> impl);
  std::shared_ptr<const Impl> impl_;
};

}  // namespace uw::opencv_adapters
