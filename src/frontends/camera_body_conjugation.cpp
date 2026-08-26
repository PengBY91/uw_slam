#include "frontends/camera_body_conjugation.hpp"

#include "sensor_models/camera_model.hpp"

namespace uw::frontends {

uw::sensor_models::Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                          const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return uw::sensor_models::Pose3::Identity();
}

uw::sensor_models::Pose3 BodyFromCameraOptical(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& camera_frame) {
  const auto camera_link_body_pose = FindRigEdgePose(rig, camera_frame);
  uw::sensor_models::Pose3 optical_to_body_rotation;
  optical_to_body_rotation.rotation = Eigen::Quaterniond(uw::sensor_models::OpticalFromBodyRotation()).inverse();
  return camera_link_body_pose * optical_to_body_rotation;
}

Eigen::Matrix<double, 6, 6> TransformCovarianceForConjugation(
    const uw::sensor_models::Pose3& original_pose, const uw::sensor_models::Pose3& conjugator,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  const Eigen::Matrix3d r_c = conjugator.rotation.toRotationMatrix();
  const Eigen::Vector3d w =
      original_pose.rotation * (conjugator.rotation.conjugate() * conjugator.translation);
  Eigen::Matrix3d w_hat;
  w_hat << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;

  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
  jacobian.block<3, 3>(0, 0) = r_c;
  jacobian.block<3, 3>(0, 3) = r_c * w_hat;
  jacobian.block<3, 3>(3, 3) = r_c;

  Eigen::Matrix<double, 6, 6> result = jacobian * covariance * jacobian.transpose();
  return 0.5 * (result + result.transpose());
}

}  // namespace uw::frontends
