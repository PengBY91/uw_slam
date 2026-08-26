#include "opencv_adapters/stereo_rectifier.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::opencv_adapters {

namespace {

using uw::domain::CameraIntrinsics;
using uw::domain::FrameEdge;
using uw::domain::RigCalibrationSnapshot;
using uw::sensor_models::FindCamera;
using uw::sensor_models::Pose3;

// Not a cryptographic hash -- only used to make DerivedRig()'s calibration
// version change deterministically whenever the raw rig or rectification
// params change, matching the convention already used for
// RunManifest::calibration_hash (src/application/replay_pipeline.cpp).
std::string Fnv1aHex(const std::string& data) {
  uint64_t hash = 1469598103934665603ull;
  for (unsigned char c : data) {
    hash ^= c;
    hash *= 1099511628211ull;
  }
  std::ostringstream out;
  out << std::hex << hash;
  return out.str();
}

const FrameEdge* FindEdge(const RigCalibrationSnapshot& rig, const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return &edge;
  }
  return nullptr;
}

bool ValidIntrinsicsK(const CameraIntrinsics& intrinsics) {
  if (intrinsics.k_matrix_row_major_size() != 9) return false;
  for (double v : intrinsics.k_matrix_row_major()) {
    if (!std::isfinite(v)) return false;
  }
  const double fx = intrinsics.k_matrix_row_major(0);
  const double fy = intrinsics.k_matrix_row_major(4);
  return fx > 0.0 && fy > 0.0;
}

bool ValidDistortion(const CameraIntrinsics& intrinsics) {
  if (!intrinsics.distortion_model().empty() && intrinsics.distortion_model() != "plumb_bob") {
    return false;
  }
  static constexpr std::array<int, 6> kAllowedLengths = {0, 4, 5, 8, 12, 14};
  const int n = intrinsics.distortion_size();
  bool length_allowed = false;
  for (int allowed : kAllowedLengths) {
    if (allowed == n) {
      length_allowed = true;
      break;
    }
  }
  if (!length_allowed) return false;
  for (double v : intrinsics.distortion()) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

bool IsZeroDistortion(const CameraIntrinsics& intrinsics) {
  for (double v : intrinsics.distortion()) {
    if (v != 0.0) return false;
  }
  return true;
}

bool SameK(const CameraIntrinsics& a, const CameraIntrinsics& b) {
  for (int i = 0; i < 9; ++i) {
    if (std::abs(a.k_matrix_row_major(i) - b.k_matrix_row_major(i)) > 1e-9) return false;
  }
  return true;
}

// Validates that `edge` carries a finite, orthonormal-rotation 4x4 rigid
// transform and returns it; std::nullopt for a missing edge or anything that
// does not parse as a rigid transform.
std::optional<Eigen::Matrix4d> ValidRigidTransform(const FrameEdge* edge) {
  if (edge == nullptr) return std::nullopt;
  if (edge->transform().matrix_row_major_size() != 16) return std::nullopt;
  Eigen::Matrix4d m;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      const double v = edge->transform().matrix_row_major(row * 4 + col);
      if (!std::isfinite(v)) return std::nullopt;
      m(row, col) = v;
    }
  }
  const Eigen::Matrix3d rotation = m.topLeftCorner<3, 3>();
  if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    return std::nullopt;
  }
  if (rotation.determinant() < 0.0) return std::nullopt;
  return m;
}

Pose3 PoseFromMatrix(const Eigen::Matrix4d& m) {
  Pose3 pose;
  pose.translation = m.topRightCorner<3, 1>();
  pose.rotation = Eigen::Quaterniond(m.topLeftCorner<3, 3>()).normalized();
  return pose;
}

// Fixed hardware-mounting rotation relating a camera's body-convention LINK
// frame to its OPTICAL frame, as a zero-translation Pose3 -- see
// sensor_models/camera_model.hpp's OpticalFromBodyRotation() doc comment.
Pose3 LinkToOptical() {
  Pose3 pose;
  pose.rotation = Eigen::Quaterniond(uw::sensor_models::OpticalFromBodyRotation()).conjugate();
  return pose;
}

Pose3 OpticalToLink() {
  Pose3 pose;
  pose.rotation = Eigen::Quaterniond(uw::sensor_models::OpticalFromBodyRotation());
  return pose;
}

void SetDerivedCamera(RigCalibrationSnapshot& rig, const std::string& sensor_id, uint32_t width,
                       uint32_t height, double fx, double fy, double cx, double cy) {
  for (int i = 0; i < rig.cameras_size(); ++i) {
    if (rig.cameras(i).sensor_id().value() != sensor_id) continue;
    auto* camera = rig.mutable_cameras(i);
    camera->set_width(width);
    camera->set_height(height);
    camera->clear_k_matrix_row_major();
    for (double v : {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);
    camera->clear_distortion();
    camera->set_distortion_model("plumb_bob");
    return;
  }
}

void SetVirtualFrameEdge(RigCalibrationSnapshot& rig, const std::string& parent_frame,
                          const std::string& child_frame, const Pose3& transform) {
  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value(parent_frame);
  edge->mutable_child_frame()->set_value(child_frame);
  *edge->mutable_transform() = transform.ToProto();
}

std::string CropPolicyName(RectificationCropPolicy policy) {
  return policy == RectificationCropPolicy::kFullCanvas ? "full_canvas" : "common_valid_roi";
}

cv::Mat ToCvK(const CameraIntrinsics& intrinsics) {
  cv::Mat k(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      k.at<double>(row, col) = intrinsics.k_matrix_row_major(row * 3 + col);
    }
  }
  return k;
}

cv::Mat ToCvDistortion(const CameraIntrinsics& intrinsics) {
  if (intrinsics.distortion_size() == 0) return cv::Mat();
  cv::Mat d(1, intrinsics.distortion_size(), CV_64F);
  for (int i = 0; i < intrinsics.distortion_size(); ++i) d.at<double>(0, i) = intrinsics.distortion(i);
  return d;
}

cv::Mat ToCvRotation(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  cv::Mat m(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) m.at<double>(row, col) = r(row, col);
  }
  return m;
}

cv::Mat ToCvVector(const Eigen::Vector3d& t) {
  cv::Mat m(3, 1, CV_64F);
  m.at<double>(0) = t.x();
  m.at<double>(1) = t.y();
  m.at<double>(2) = t.z();
  return m;
}

Eigen::Matrix3d FromCvRotation(const cv::Mat& r) {
  Eigen::Matrix3d out;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) out(row, col) = r.at<double>(row, col);
  }
  return out;
}

}  // namespace

class StereoRectificationContext::Impl {
 public:
  bool identity_fast_path = false;
  uw::domain::RigCalibrationSnapshot derived_rig;
  std::string left_rectified_frame;
  std::string right_rectified_frame;
  uint32_t input_width = 0;
  uint32_t input_height = 0;
  uint32_t output_width = 0;
  uint32_t output_height = 0;
  // Only populated when !identity_fast_path. Destination-indexed remap
  // coordinates into the raw (distorted) source image -- see
  // sensor_models/camera_rectifier.cpp for the same backward-mapping
  // convention this mirrors, here delegated to cv::remap.
  cv::Mat map_left_x;
  cv::Mat map_left_y;
  cv::Mat map_right_x;
  cv::Mat map_right_y;
};

StereoRectificationContext::StereoRectificationContext(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}

std::optional<StereoRectificationContext> StereoRectificationContext::Create(
    const RigCalibrationSnapshot& raw_rig, const StereoRectificationParams& params,
    std::string* error) {
  auto fail = [&](const std::string& message) -> std::optional<StereoRectificationContext> {
    if (error != nullptr) *error = message;
    return std::nullopt;
  };

  if (params.left_sensor_id.empty() || params.left_frame.empty() ||
      params.right_sensor_id.empty() || params.right_frame.empty() ||
      params.rectified_frame_suffix.empty()) {
    return fail("stereo rectification params must not have empty identifiers");
  }
  if (params.left_sensor_id == params.right_sensor_id || params.left_frame == params.right_frame) {
    return fail("left and right sensor id/frame must be distinct");
  }
  if (!std::isfinite(params.alpha) || params.alpha < -1.0 || params.alpha > 1.0) {
    return fail("alpha must be finite and within [-1, 1]");
  }

  const CameraIntrinsics* left_intrinsics = FindCamera(raw_rig, params.left_sensor_id);
  const CameraIntrinsics* right_intrinsics = FindCamera(raw_rig, params.right_sensor_id);
  if (left_intrinsics == nullptr) return fail("missing left camera: " + params.left_sensor_id);
  if (right_intrinsics == nullptr) return fail("missing right camera: " + params.right_sensor_id);

  if (!ValidIntrinsicsK(*left_intrinsics)) return fail("invalid left camera K matrix");
  if (!ValidIntrinsicsK(*right_intrinsics)) return fail("invalid right camera K matrix");
  if (left_intrinsics->width() == 0 || left_intrinsics->height() == 0) {
    return fail("left camera has zero width/height");
  }
  if (right_intrinsics->width() == 0 || right_intrinsics->height() == 0) {
    return fail("right camera has zero width/height");
  }
  if (left_intrinsics->width() != right_intrinsics->width() ||
      left_intrinsics->height() != right_intrinsics->height()) {
    return fail("left and right camera image sizes must match");
  }
  if (!ValidDistortion(*left_intrinsics)) {
    return fail("left camera distortion model/coefficients are not supported");
  }
  if (!ValidDistortion(*right_intrinsics)) {
    return fail("right camera distortion model/coefficients are not supported");
  }

  const FrameEdge* left_edge = FindEdge(raw_rig, params.left_frame);
  const FrameEdge* right_edge = FindEdge(raw_rig, params.right_frame);
  const auto left_transform = ValidRigidTransform(left_edge);
  const auto right_transform = ValidRigidTransform(right_edge);
  if (!left_transform.has_value()) return fail("missing/invalid left frame edge: " + params.left_frame);
  if (!right_transform.has_value()) {
    return fail("missing/invalid right frame edge: " + params.right_frame);
  }

  const uint32_t input_width = left_intrinsics->width();
  const uint32_t input_height = left_intrinsics->height();
  const std::string left_rectified_frame = params.left_frame + params.rectified_frame_suffix;
  const std::string right_rectified_frame = params.right_frame + params.rectified_frame_suffix;

  const Pose3 b_T_left_link = PoseFromMatrix(*left_transform);
  const Pose3 b_T_right_link = PoseFromMatrix(*right_transform);
  const Pose3 link_to_optical = LinkToOptical();
  const Pose3 optical_to_link = OpticalToLink();
  const Pose3 b_T_left_optical = b_T_left_link * link_to_optical;
  const Pose3 b_T_right_optical = b_T_right_link * link_to_optical;

  // Identity fast path: skip OpenCV entirely when the raw rig is already a
  // rectified-form parallel pair (zero distortion, identical K, identical
  // camera_link orientation -- which implies identical optical orientation
  // too, since both cameras share the same hardware mounting rotation --
  // and a baseline purely along the optical x axis). This is the R1=R2=I
  // special case of the general formula below, computed directly to avoid
  // OpenCV overhead on the common synthetic-rig case.
  const bool zero_distortion = IsZeroDistortion(*left_intrinsics) && IsZeroDistortion(*right_intrinsics);
  const bool same_k = SameK(*left_intrinsics, *right_intrinsics);
  const bool same_orientation =
      left_transform->topLeftCorner<3, 3>().isApprox(right_transform->topLeftCorner<3, 3>(), 1e-9);
  const Eigen::Vector3d body_baseline =
      left_transform->topRightCorner<3, 1>() - right_transform->topRightCorner<3, 1>();
  const Eigen::Vector3d optical_baseline_probe =
      uw::sensor_models::OpticalFromBodyRotation() * body_baseline;
  const bool baseline_x_only = std::abs(optical_baseline_probe.y()) < 1e-9 &&
                                std::abs(optical_baseline_probe.z()) < 1e-9 &&
                                std::abs(optical_baseline_probe.x()) > 1e-9;
  const bool identity_fast_path = zero_distortion && same_k && same_orientation && baseline_x_only;

  auto impl = std::make_shared<Impl>();
  impl->input_width = input_width;
  impl->input_height = input_height;
  impl->left_rectified_frame = left_rectified_frame;
  impl->right_rectified_frame = right_rectified_frame;

  RigCalibrationSnapshot derived = raw_rig;

  if (identity_fast_path) {
    impl->identity_fast_path = true;
    impl->output_width = input_width;
    impl->output_height = input_height;

    SetDerivedCamera(derived, params.left_sensor_id, input_width, input_height,
                      left_intrinsics->k_matrix_row_major(0), left_intrinsics->k_matrix_row_major(4),
                      left_intrinsics->k_matrix_row_major(2), left_intrinsics->k_matrix_row_major(5));
    SetDerivedCamera(derived, params.right_sensor_id, input_width, input_height,
                      right_intrinsics->k_matrix_row_major(0), right_intrinsics->k_matrix_row_major(4),
                      right_intrinsics->k_matrix_row_major(2), right_intrinsics->k_matrix_row_major(5));
    SetVirtualFrameEdge(derived, left_edge->parent_frame().value(), left_rectified_frame,
                         b_T_left_optical * optical_to_link);
    SetVirtualFrameEdge(derived, right_edge->parent_frame().value(), right_rectified_frame,
                         b_T_right_optical * optical_to_link);
  } else {
    // General path: OpenCV computes the rectifying rotations/projections
    // from the true relative extrinsics between the two OPTICAL frames.
    const Pose3 c2_T_c1 = b_T_right_optical.Inverse() * b_T_left_optical;

    const cv::Mat k1 = ToCvK(*left_intrinsics);
    const cv::Mat d1 = ToCvDistortion(*left_intrinsics);
    const cv::Mat k2 = ToCvK(*right_intrinsics);
    const cv::Mat d2 = ToCvDistortion(*right_intrinsics);
    const cv::Size image_size(static_cast<int>(input_width), static_cast<int>(input_height));
    const cv::Mat r_cv = ToCvRotation(c2_T_c1.rotation);
    const cv::Mat t_cv = ToCvVector(c2_T_c1.translation);

    cv::Mat r1, r2, p1, p2, q;
    cv::Rect roi1, roi2;
    cv::stereoRectify(k1, d1, k2, d2, image_size, r_cv, t_cv, r1, r2, p1, p2, q,
                       cv::CALIB_ZERO_DISPARITY, params.alpha, image_size, &roi1, &roi2);

    cv::Mat map_left_x_full, map_left_y_full, map_right_x_full, map_right_y_full;
    cv::initUndistortRectifyMap(k1, d1, r1, p1, image_size, CV_32FC1, map_left_x_full, map_left_y_full);
    cv::initUndistortRectifyMap(k2, d2, r2, p2, image_size, CV_32FC1, map_right_x_full,
                                 map_right_y_full);

    cv::Mat p1_derived = p1.clone();
    cv::Mat p2_derived = p2.clone();
    cv::Rect output_rect(0, 0, image_size.width, image_size.height);
    if (params.crop_policy == RectificationCropPolicy::kCommonValidRoi) {
      const cv::Rect common = roi1 & roi2;
      if (common.width <= 0 || common.height <= 0) {
        return fail("stereo rectification produced an empty common valid ROI");
      }
      output_rect = common;
      p1_derived.at<double>(0, 2) -= common.x;
      p1_derived.at<double>(1, 2) -= common.y;
      p2_derived.at<double>(0, 2) -= common.x;
      p2_derived.at<double>(1, 2) -= common.y;
    }

    impl->identity_fast_path = false;
    impl->output_width = static_cast<uint32_t>(output_rect.width);
    impl->output_height = static_cast<uint32_t>(output_rect.height);
    impl->map_left_x = map_left_x_full(output_rect).clone();
    impl->map_left_y = map_left_y_full(output_rect).clone();
    impl->map_right_x = map_right_x_full(output_rect).clone();
    impl->map_right_y = map_right_y_full(output_rect).clone();

    const double p2_fx = p2_derived.at<double>(0, 0);
    if (p2_fx == 0.0 || !std::isfinite(p2_fx)) return fail("degenerate rectified P2 focal length");
    const double baseline_m = std::abs(p2_derived.at<double>(0, 3) / p2_fx);
    const double expected_baseline_m = c2_T_c1.translation.norm();
    if (!std::isfinite(baseline_m) || baseline_m <= 0.0) {
      return fail("derived stereo baseline is not finite/positive");
    }
    if (std::abs(baseline_m - expected_baseline_m) > 1e-4 * std::max(1.0, expected_baseline_m)) {
      return fail("derived stereo baseline does not match extrinsic translation");
    }

    SetDerivedCamera(derived, params.left_sensor_id, impl->output_width, impl->output_height,
                      p1_derived.at<double>(0, 0), p1_derived.at<double>(1, 1),
                      p1_derived.at<double>(0, 2), p1_derived.at<double>(1, 2));
    SetDerivedCamera(derived, params.right_sensor_id, impl->output_width, impl->output_height,
                      p2_derived.at<double>(0, 0), p2_derived.at<double>(1, 1),
                      p2_derived.at<double>(0, 2), p2_derived.at<double>(1, 2));

    const Eigen::Matrix3d r1_eigen = FromCvRotation(r1);
    const Eigen::Matrix3d r2_eigen = FromCvRotation(r2);
    Pose3 raw_optical_to_rectified_left;
    raw_optical_to_rectified_left.rotation = Eigen::Quaterniond(r1_eigen).conjugate();
    Pose3 raw_optical_to_rectified_right;
    raw_optical_to_rectified_right.rotation = Eigen::Quaterniond(r2_eigen).conjugate();

    const Pose3 b_T_left_rectified_optical = b_T_left_optical * raw_optical_to_rectified_left;
    const Pose3 b_T_right_rectified_optical = b_T_right_optical * raw_optical_to_rectified_right;

    SetVirtualFrameEdge(derived, left_edge->parent_frame().value(), left_rectified_frame,
                         b_T_left_rectified_optical * optical_to_link);
    SetVirtualFrameEdge(derived, right_edge->parent_frame().value(), right_rectified_frame,
                         b_T_right_rectified_optical * optical_to_link);
  }

  std::ostringstream digest_input;
  digest_input << raw_rig.SerializeAsString() << '|' << params.left_sensor_id << '|'
               << params.left_frame << '|' << params.right_sensor_id << '|' << params.right_frame
               << '|' << impl->output_width << '|' << impl->output_height << '|' << params.alpha
               << '|' << CropPolicyName(params.crop_policy) << '|' << params.rectified_frame_suffix;
  derived.mutable_calibration_version()->set_value(
      raw_rig.calibration_version().value() + "+opencv_rectified_v1_" + Fnv1aHex(digest_input.str()));

  impl->derived_rig = std::move(derived);
  return StereoRectificationContext(std::move(impl));
}

std::optional<RectifiedStereoBundle> StereoRectificationContext::Process(
    const uw::domain::ImageFrame& left, const uw::domain::ImageFrame& right,
    std::string* error) const {
  auto fail = [&](const std::string& message) -> std::optional<RectifiedStereoBundle> {
    if (error != nullptr) *error = message;
    return std::nullopt;
  };

  if (!uw::domain::ValidateImageFrame(left).ok()) return fail("left image frame failed validation");
  if (!uw::domain::ValidateImageFrame(right).ok()) return fail("right image frame failed validation");
  if (left.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      right.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8) {
    return fail("stereo rectification requires MONO8 input");
  }
  if (left.width() != impl_->input_width || left.height() != impl_->input_height ||
      right.width() != impl_->input_width || right.height() != impl_->input_height) {
    return fail("image size does not match the rectification context's calibrated size");
  }

  RectifiedStereoBundle bundle;

  if (impl_->identity_fast_path) {
    bundle.images.primary = left;
    bundle.images.primary.mutable_header()->mutable_sensor_frame()->set_value(
        impl_->left_rectified_frame);
    bundle.images.primary.set_is_rectified(true);

    uw::domain::ImageFrame secondary = right;
    secondary.mutable_header()->mutable_sensor_frame()->set_value(impl_->right_rectified_frame);
    secondary.set_is_rectified(true);
    bundle.images.secondary = std::move(secondary);
    return bundle;
  }

  const cv::Mat src_left(static_cast<int>(left.height()), static_cast<int>(left.width()), CV_8UC1,
                          const_cast<char*>(left.pixel_data().data()),
                          static_cast<std::size_t>(left.row_stride_bytes()));
  const cv::Mat src_right(static_cast<int>(right.height()), static_cast<int>(right.width()), CV_8UC1,
                           const_cast<char*>(right.pixel_data().data()),
                           static_cast<std::size_t>(right.row_stride_bytes()));

  cv::Mat dst_left;
  cv::Mat dst_right;
  cv::remap(src_left, dst_left, impl_->map_left_x, impl_->map_left_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::remap(src_right, dst_right, impl_->map_right_x, impl_->map_right_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));

  auto to_image_frame = [&](const uw::domain::ImageFrame& source, const cv::Mat& remapped,
                             const std::string& rectified_frame) {
    uw::domain::ImageFrame out = source;
    out.set_width(static_cast<uint32_t>(remapped.cols));
    out.set_height(static_cast<uint32_t>(remapped.rows));
    out.set_row_stride_bytes(static_cast<uint32_t>(remapped.step[0]));
    out.set_pixel_data(std::string(reinterpret_cast<const char*>(remapped.data),
                                    remapped.step[0] * static_cast<std::size_t>(remapped.rows)));
    out.mutable_header()->mutable_sensor_frame()->set_value(rectified_frame);
    out.set_is_rectified(true);
    return out;
  };

  bundle.images.primary = to_image_frame(left, dst_left, impl_->left_rectified_frame);
  bundle.images.secondary = to_image_frame(right, dst_right, impl_->right_rectified_frame);
  return bundle;
}

const RigCalibrationSnapshot& StereoRectificationContext::DerivedRig() const {
  return impl_->derived_rig;
}

const std::string& StereoRectificationContext::LeftRectifiedFrame() const {
  return impl_->left_rectified_frame;
}

const std::string& StereoRectificationContext::RightRectifiedFrame() const {
  return impl_->right_rectified_frame;
}

}  // namespace uw::opencv_adapters
