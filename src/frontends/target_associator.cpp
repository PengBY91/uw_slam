#include "frontends/target_associator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::frontends {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMinVariance = 1e-12;
constexpr double kMaxMeasurementVariance = 1e12;
constexpr double kMaxTargetRangeM = 1e6;
constexpr long double kMaxExactlyRepresentableIntegerInDouble =
    9007199254740992.0L;  // 2^53

double WrapBearing(double angle) {
  angle = std::remainder(angle, kTwoPi);
  if (angle <= -kPi) angle += kTwoPi;
  return angle;
}

bool FinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

bool ValidParams(const TargetAssociatorParams& params) {
  return FinitePositive(params.max_corrected_time_delta_s) &&
         FinitePositive(params.max_bearing_mahalanobis_sq) &&
         FinitePositive(params.max_range_mahalanobis_sq) &&
         FinitePositive(params.max_motion_bearing_delta_rad) &&
         std::isfinite(params.max_motion_rate_rad_s) &&
         params.max_motion_rate_rad_s >= 0.0 &&
         FinitePositive(params.max_bearing_variance_rad2) &&
         FinitePositive(params.max_range_variance_m2);
}

bool ValidCovariance(const Eigen::Matrix2d& covariance) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-10) ||
      covariance.cwiseAbs().maxCoeff() > kMaxMeasurementVariance) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1e-10 &&
         covariance(0, 0) > 0.0;
}

std::optional<long double> StampSeconds(const uw::domain::Stamp& stamp) {
  if (stamp.seconds() < 0 || stamp.nanos() < 0 ||
      stamp.nanos() >= 1'000'000'000 ||
      static_cast<long double>(stamp.seconds()) >
          kMaxExactlyRepresentableIntegerInDouble) {
    return std::nullopt;
  }
  return static_cast<long double>(stamp.seconds()) +
         static_cast<long double>(stamp.nanos()) * 1.0e-9L;
}

std::optional<Eigen::Matrix4d> EdgeMatrix(const uw::domain::FrameEdge& edge) {
  if (edge.parent_frame().value().empty() || edge.child_frame().value().empty() ||
      edge.parent_frame().value() == edge.child_frame().value() ||
      edge.transform().matrix_row_major_size() != 16) {
    return std::nullopt;
  }
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = edge.transform().matrix_row_major(row * 4 + col);
    }
  }
  if (!matrix.allFinite() ||
      !matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-9)) {
    return std::nullopt;
  }
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if (!rotation.transpose().isApprox(rotation.inverse(), 1e-8) ||
      std::abs(rotation.determinant() - 1.0) > 1e-8) {
    return std::nullopt;
  }
  return matrix;
}

struct Neighbor {
  std::string frame;
  Eigen::Matrix4d from_current;
};

std::optional<uw::sensor_models::Pose3> BaseFromFrame(
    const uw::domain::RigCalibrationSnapshot& rig, const std::string& target_frame) {
  if (target_frame.empty()) return std::nullopt;
  if (target_frame == "base_link") return uw::sensor_models::Pose3::Identity();

  std::map<std::string, std::vector<Neighbor>> graph;
  for (const auto& edge : rig.frame_tree()) {
    const auto parent_from_child = EdgeMatrix(edge);
    if (!parent_from_child) return std::nullopt;
    const std::string parent = edge.parent_frame().value();
    const std::string child = edge.child_frame().value();
    // `from_current` maps the neighbor into the current frame.
    graph[parent].push_back({child, *parent_from_child});
    graph[child].push_back({parent, parent_from_child->inverse()});
  }
  for (auto& [frame, neighbors] : graph) {
    (void)frame;
    std::sort(neighbors.begin(), neighbors.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.frame < rhs.frame; });
  }

  std::queue<std::pair<std::string, Eigen::Matrix4d>> pending;
  std::set<std::string> visited{"base_link"};
  pending.push({"base_link", Eigen::Matrix4d::Identity()});
  while (!pending.empty()) {
    auto [current, base_from_current] = pending.front();
    pending.pop();
    const auto it = graph.find(current);
    if (it == graph.end()) continue;
    for (const auto& neighbor : it->second) {
      if (!visited.insert(neighbor.frame).second) continue;
      const Eigen::Matrix4d base_from_neighbor =
          base_from_current * neighbor.from_current;
      if (neighbor.frame == target_frame) {
        return uw::sensor_models::Pose3::FromProto([&] {
          uw::domain::Transform3D proto;
          for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
              proto.add_matrix_row_major(base_from_neighbor(row, col));
            }
          }
          return proto;
        }());
      }
      pending.push({neighbor.frame, base_from_neighbor});
    }
  }
  return std::nullopt;
}

bool ValidRigTree(const uw::domain::RigCalibrationSnapshot& rig) {
  std::map<std::string, std::string> parent_by_child;
  for (const auto& edge : rig.frame_tree()) {
    if (!EdgeMatrix(edge)) return false;
    const std::string parent = edge.parent_frame().value();
    const std::string child = edge.child_frame().value();
    if (child == "base_link" || !parent_by_child.emplace(child, parent).second) {
      return false;
    }
  }
  for (const auto& [frame, parent] : parent_by_child) {
    (void)parent;
    std::set<std::string> path;
    std::string current = frame;
    while (current != "base_link") {
      if (!path.insert(current).second) return false;
      const auto it = parent_by_child.find(current);
      if (it == parent_by_child.end()) return false;
      current = it->second;
    }
  }
  return true;
}

bool FrameMatchesSensor(const std::string& sensor_id, const std::string& frame,
                        bool visual) {
  // Calibration v1 has no explicit SensorId -> FrameId field. Enforce its
  // exact canonical naming contract instead of accepting prefixes. The
  // singleton sonar is the one legacy exception used by every checked-in
  // rig (sonar0 -> sonar_link).
  const std::string expected =
      !visual && sensor_id == "sonar0" ? "sonar_link" : sensor_id + "_link";
  return frame == expected;
}

bool SensorHasExclusiveRole(const uw::domain::RigCalibrationSnapshot& rig,
                            const SensorTargetDetection& input, bool visual) {
  const auto camera_count = std::count_if(
      rig.cameras().begin(), rig.cameras().end(), [&](const auto& camera) {
        return camera.sensor_id().value() == input.sensor_id;
      });
  const auto enabled_sonar_count = std::count_if(
      rig.sonar_beam_models().begin(), rig.sonar_beam_models().end(),
      [&](const auto& sonar) {
        return sonar.sensor_id().value() == input.sensor_id && sonar.sonar_enabled();
      });
  return visual ? camera_count == 1 && enabled_sonar_count == 0
                : enabled_sonar_count == 1 && camera_count == 0;
}

struct Projected {
  TargetMeasurement measurement;
  std::string id;
};

struct ProjectResult {
  std::optional<Projected> projected;
  AssociationReason reason = AssociationReason::kInvalidInput;
};

Eigen::Matrix2d DetectionCovariance(const uw::domain::TargetDetection& detection) {
  Eigen::Matrix2d covariance;
  covariance << detection.covariance_2x2_row_major(0),
      detection.covariance_2x2_row_major(1),
      detection.covariance_2x2_row_major(2),
      detection.covariance_2x2_row_major(3);
  return covariance;
}

template <typename Projection>
std::optional<std::pair<Eigen::Vector2d, Eigen::Matrix2d>> ProjectWithJacobian(
    double bearing, double range, const Eigen::Matrix2d& covariance,
    Projection projection) {
  const Eigen::Vector2d output = projection(bearing, range);
  if (!output.allFinite() || output.y() <= 0.0) return std::nullopt;
  constexpr double kBearingStep = 1e-6;
  const double range_step = std::max(1e-6, std::abs(range) * 1e-6);
  Eigen::Matrix2d jacobian;
  const Eigen::Vector2d bearing_plus = projection(bearing + kBearingStep, range);
  const Eigen::Vector2d bearing_minus = projection(bearing - kBearingStep, range);
  const Eigen::Vector2d range_plus = projection(bearing, range + range_step);
  const Eigen::Vector2d range_minus = projection(bearing, range - range_step);
  if (!bearing_plus.allFinite() || !bearing_minus.allFinite() ||
      !range_plus.allFinite() || !range_minus.allFinite()) {
    return std::nullopt;
  }
  jacobian(0, 0) = WrapBearing(bearing_plus.x() - bearing_minus.x()) /
                   (2.0 * kBearingStep);
  jacobian(1, 0) = (bearing_plus.y() - bearing_minus.y()) /
                   (2.0 * kBearingStep);
  jacobian(0, 1) = WrapBearing(range_plus.x() - range_minus.x()) /
                   (2.0 * range_step);
  jacobian(1, 1) = (range_plus.y() - range_minus.y()) /
                   (2.0 * range_step);
  Eigen::Matrix2d projected_covariance = jacobian * covariance * jacobian.transpose();
  projected_covariance = 0.5 * (projected_covariance + projected_covariance.transpose());
  if (!ValidCovariance(projected_covariance) ||
      projected_covariance(1, 1) <= 0.0) {
    return std::nullopt;
  }
  return std::make_pair(output, projected_covariance);
}

ProjectResult ProjectOne(const SensorTargetDetection& input, bool visual,
                         const uw::domain::RigCalibrationSnapshot& rig) {
  const auto& detection = input.detection;
  const auto expected_source = visual ? uw::domain::ASSIST_SOURCE_VISUAL
                                      : uw::domain::ASSIST_SOURCE_SONAR;
  if (input.sensor_id.empty() || input.sensor_frame.empty() ||
      input.calibration_version.empty() ||
      detection.source_observation().value().empty() ||
      !detection.has_capture_time() || detection.class_label().empty() ||
      detection.source() != expected_source || !std::isfinite(detection.confidence()) ||
      detection.confidence() < 0.0 || detection.confidence() > 1.0 ||
      !std::isfinite(detection.bearing_rad()) ||
      std::abs(detection.bearing_rad()) > kPi ||
      detection.covariance_2x2_row_major_size() != 4 ||
      (!visual && (!detection.has_range() || !FinitePositive(detection.range_m()))) ||
      (detection.has_range() &&
       (!FinitePositive(detection.range_m()) || detection.range_m() > kMaxTargetRangeM))) {
    return {};
  }
  if (!ValidRigTree(rig) || !SensorHasExclusiveRole(rig, input, visual)) return {};
  for (const auto& [name, value] : detection.quality_metrics()) {
    (void)name;
    if (!std::isfinite(value)) return {};
  }
  const Eigen::Matrix2d covariance = DetectionCovariance(detection);
  if (!ValidCovariance(covariance) ||
      (detection.has_range() && covariance(1, 1) <= 0.0)) {
    return {};
  }
  if (input.calibration_version != rig.calibration_version().value() ||
      rig.calibration_version().value().empty()) {
    return {std::nullopt, AssociationReason::kCalibrationMismatch};
  }
  const auto offset = rig.time_offset_seconds().find(input.sensor_id);
  const auto provenance = rig.time_offset_provenance().find(input.sensor_id);
  const auto capture_time = StampSeconds(detection.capture_time());
  if (!capture_time || offset == rig.time_offset_seconds().end() ||
      provenance == rig.time_offset_provenance().end() || provenance->second.empty() ||
      !std::isfinite(offset->second)) {
    return {};
  }
  const auto base_from_sensor = BaseFromFrame(rig, input.sensor_frame);
  if (!base_from_sensor) {
    return {std::nullopt, AssociationReason::kFrameUnresolved};
  }
  if (!FrameMatchesSensor(input.sensor_id, input.sensor_frame, visual)) return {};

  TargetMeasurement measurement;
  const long double corrected_time =
      *capture_time + static_cast<long double>(offset->second);
  if (corrected_time < 0.0L ||
      corrected_time > kMaxExactlyRepresentableIntegerInDouble) {
    return {};
  }
  measurement.corrected_time_s = static_cast<double>(corrected_time);
  measurement.class_label = detection.class_label();
  measurement.confidence = detection.confidence();
  measurement.sources = {detection.source()};
  measurement.observation_ids = {detection.source_observation()};

  if (visual && !detection.has_range()) {
    const auto base_ray = [&](double bearing) {
      const Eigen::Vector3d optical_ray(std::sin(bearing), 0.0, std::cos(bearing));
      const Eigen::Vector3d body_ray =
          uw::sensor_models::OpticalFromBodyRotation().transpose() * optical_ray;
      return base_from_sensor->rotation * body_ray;
    };
    const Eigen::Vector3d ray = base_ray(detection.bearing_rad());
    if (!ray.allFinite() || ray.head<2>().norm() <= 1e-12) return {};
    measurement.bearing_rad = WrapBearing(std::atan2(ray.y(), ray.x()));
    constexpr double kStep = 1e-6;
    const double derivative =
        WrapBearing(std::atan2(base_ray(detection.bearing_rad() + kStep).y(),
                               base_ray(detection.bearing_rad() + kStep).x()) -
                    std::atan2(base_ray(detection.bearing_rad() - kStep).y(),
                               base_ray(detection.bearing_rad() - kStep).x())) /
        (2.0 * kStep);
    measurement.covariance.setZero();
    measurement.covariance(0, 0) = derivative * derivative * covariance(0, 0);
    measurement.covariance(1, 1) = covariance(1, 1);
    if (!ValidCovariance(measurement.covariance)) return {};
  } else {
    const auto projection = [&](double bearing, double range) {
      Eigen::Vector3d point_sensor;
      if (visual) {
        const Eigen::Vector3d point_optical(range * std::sin(bearing), 0.0,
                                            range * std::cos(bearing));
        point_sensor = uw::sensor_models::OpticalFromBodyRotation().transpose() *
                       point_optical;
      } else {
        point_sensor = Eigen::Vector3d(range * std::cos(bearing),
                                       range * std::sin(bearing), 0.0);
      }
      const Eigen::Vector3d point_base = base_from_sensor->Apply(point_sensor);
      return Eigen::Vector2d(WrapBearing(std::atan2(point_base.y(), point_base.x())),
                             point_base.norm());
    };
    const auto projected = ProjectWithJacobian(detection.bearing_rad(), detection.range_m(),
                                                covariance, projection);
    if (!projected) return {};
    measurement.bearing_rad = projected->first.x();
    measurement.range_m = projected->first.y();
    measurement.covariance = projected->second;
  }
  if (!std::isfinite(measurement.corrected_time_s) ||
      !std::isfinite(measurement.bearing_rad)) {
    return {};
  }
  return {Projected{measurement, detection.source_observation().value()},
          AssociationReason::kAccepted};
}

bool GenericClass(const std::string& label) {
  return label == "target" || label == "sonar_target";
}

bool ClassesCompatible(const std::string& lhs, const std::string& rhs) {
  return lhs == rhs || GenericClass(lhs) || GenericClass(rhs);
}

template <typename T, typename Less>
void SortUnique(std::vector<T>* values, Less less) {
  std::sort(values->begin(), values->end(), less);
  values->erase(std::unique(values->begin(), values->end(),
                            [&](const auto& lhs, const auto& rhs) {
                              return !less(lhs, rhs) && !less(rhs, lhs);
                            }),
                values->end());
}

std::optional<TargetMeasurement> Fuse(const TargetMeasurement& visual,
                                      const TargetMeasurement& sonar) {
  TargetMeasurement fused;
  fused.corrected_time_s = std::max(visual.corrected_time_s, sonar.corrected_time_s);
  fused.class_label = GenericClass(visual.class_label) ? sonar.class_label : visual.class_label;
  fused.confidence = 1.0 - (1.0 - visual.confidence) * (1.0 - sonar.confidence);

  if (!sonar.range_m) return std::nullopt;
  if (visual.range_m) {
    const Eigen::Matrix2d innovation_covariance =
        visual.covariance + sonar.covariance;
    Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      return std::nullopt;
    }
    const Eigen::Matrix2d gain =
        visual.covariance * ldlt.solve(Eigen::Matrix2d::Identity());
    if (ldlt.info() != Eigen::Success || !gain.allFinite()) {
      return std::nullopt;
    }
    const Eigen::Vector2d innovation(
        WrapBearing(sonar.bearing_rad - visual.bearing_rad),
        *sonar.range_m - *visual.range_m);
    const Eigen::Vector2d state =
        Eigen::Vector2d(visual.bearing_rad, *visual.range_m) + gain * innovation;
    fused.bearing_rad = WrapBearing(state[0]);
    fused.range_m = state[1];
    fused.covariance = visual.covariance - gain * visual.covariance;
  } else {
    const double innovation_variance =
        sonar.covariance(0, 0) + visual.covariance(0, 0);
    if (!FinitePositive(innovation_variance)) return std::nullopt;
    const Eigen::Vector2d gain =
        sonar.covariance.col(0) / innovation_variance;
    const double innovation = WrapBearing(visual.bearing_rad - sonar.bearing_rad);
    const Eigen::Vector2d state =
        Eigen::Vector2d(sonar.bearing_rad, *sonar.range_m) + gain * innovation;
    fused.bearing_rad = WrapBearing(state[0]);
    fused.range_m = state[1];
    fused.covariance =
        sonar.covariance - gain * sonar.covariance.row(0);
  }
  fused.covariance = 0.5 * (fused.covariance + fused.covariance.transpose());
  if (!fused.range_m || !FinitePositive(*fused.range_m) ||
      !ValidCovariance(fused.covariance) || fused.covariance(1, 1) <= 0.0) {
    return std::nullopt;
  }
  fused.sources = visual.sources;
  fused.sources.insert(fused.sources.end(), sonar.sources.begin(), sonar.sources.end());
  SortUnique(&fused.sources, [](auto lhs, auto rhs) { return lhs < rhs; });
  fused.observation_ids = visual.observation_ids;
  fused.observation_ids.insert(fused.observation_ids.end(), sonar.observation_ids.begin(),
                               sonar.observation_ids.end());
  SortUnique(&fused.observation_ids, [](const auto& lhs, const auto& rhs) {
    return lhs.value() < rhs.value();
  });
  return fused;
}

std::string FirstId(const TargetMeasurement& measurement) {
  return measurement.observation_ids.empty() ? "" : measurement.observation_ids[0].value();
}

void SetBoundaryDecision(AssociationDiagnostic* diagnostic,
                         AssociationReason reason) {
  diagnostic->reason = reason;
  diagnostic->value = 0.0;
  diagnostic->threshold = 1.0;
  switch (reason) {
    case AssociationReason::kCalibrationMismatch:
      diagnostic->metric = AssociationMetric::kCalibrationMatch;
      break;
    case AssociationReason::kFrameUnresolved:
      diagnostic->metric = AssociationMetric::kFrameResolution;
      break;
    default:
      diagnostic->metric = AssociationMetric::kInputValidity;
      break;
  }
}

AssociationDiagnostic SingleSourceDecision(const Projected& projected,
                                           bool visual) {
  AssociationDiagnostic diagnostic;
  if (visual) {
    diagnostic.visual_observation_id = projected.id;
  } else {
    diagnostic.sonar_observation_id = projected.id;
  }
  diagnostic.accepted = true;
  diagnostic.reason = AssociationReason::kSingleSourceAccepted;
  diagnostic.metric = AssociationMetric::kInputValidity;
  diagnostic.value = 1.0;
  diagnostic.threshold = 1.0;
  return diagnostic;
}

bool DiagnosticLess(const AssociationDiagnostic& lhs,
                    const AssociationDiagnostic& rhs) {
  // Modality is an explicit primary key: visual boundary failures first,
  // then sonar boundary failures, then actual visual/sonar pair decisions.
  // This avoids the surprising empty-string ordering that would otherwise
  // place sonar-only diagnostics before visual-only diagnostics.
  const int lhs_kind = !lhs.visual_observation_id.empty() && lhs.sonar_observation_id.empty()
                           ? 0
                           : lhs.visual_observation_id.empty() ? 1 : 2;
  const int rhs_kind = !rhs.visual_observation_id.empty() && rhs.sonar_observation_id.empty()
                           ? 0
                           : rhs.visual_observation_id.empty() ? 1 : 2;
  return std::tie(lhs_kind, lhs.visual_observation_id, lhs.sonar_observation_id, lhs.reason) <
         std::tie(rhs_kind, rhs.visual_observation_id, rhs.sonar_observation_id, rhs.reason);
}

}  // namespace

TargetAssociator::TargetAssociator(TargetAssociatorParams params) : params_(params) {
  if (!ValidParams(params_)) throw std::invalid_argument("invalid TargetAssociatorParams");
}

TargetAssociationResult TargetAssociator::Associate(
    const std::vector<SensorTargetDetection>& visual,
    const std::vector<SensorTargetDetection>& sonar,
    const uw::domain::RigCalibrationSnapshot& rig) const {
  TargetAssociationResult result;

  std::map<std::string, std::size_t> observation_id_counts;
  const auto count_ids = [&](const auto& inputs) {
    for (const auto& input : inputs) {
      const std::string& id = input.detection.source_observation().value();
      if (!id.empty()) ++observation_id_counts[id];
    }
  };
  count_ids(visual);
  count_ids(sonar);
  const bool has_duplicate_id = std::any_of(
      observation_id_counts.begin(), observation_id_counts.end(),
      [](const auto& item) { return item.second > 1; });
  if (has_duplicate_id) {
    // Association provenance is a set of globally unique observation IDs.
    // Reject the whole batch atomically: retaining unrelated measurements
    // would make retry behavior and downstream tracker acceptance ambiguous.
    for (const auto& input : visual) {
      AssociationDiagnostic diagnostic;
      diagnostic.visual_observation_id =
          input.detection.source_observation().value();
      SetBoundaryDecision(&diagnostic, AssociationReason::kInvalidInput);
      result.diagnostics.push_back(std::move(diagnostic));
    }
    for (const auto& input : sonar) {
      AssociationDiagnostic diagnostic;
      diagnostic.sonar_observation_id =
          input.detection.source_observation().value();
      SetBoundaryDecision(&diagnostic, AssociationReason::kInvalidInput);
      result.diagnostics.push_back(std::move(diagnostic));
    }
    std::sort(result.diagnostics.begin(), result.diagnostics.end(), DiagnosticLess);
    return result;
  }

  std::vector<Projected> projected_visual;
  std::vector<Projected> projected_sonar;

  const auto project_inputs = [&](const auto& inputs, bool is_visual, auto* output) {
    for (const auto& input : inputs) {
      const ProjectResult projected = ProjectOne(input, is_visual, rig);
      if (projected.projected) {
        output->push_back(*projected.projected);
      } else {
        AssociationDiagnostic diagnostic;
        if (is_visual) {
          diagnostic.visual_observation_id = input.detection.source_observation().value();
        } else {
          diagnostic.sonar_observation_id = input.detection.source_observation().value();
        }
        SetBoundaryDecision(&diagnostic, projected.reason);
        result.diagnostics.push_back(std::move(diagnostic));
      }
    }
    std::sort(output->begin(), output->end(), [](const auto& lhs, const auto& rhs) {
      return lhs.id < rhs.id;
    });
  };
  project_inputs(visual, true, &projected_visual);
  project_inputs(sonar, false, &projected_sonar);

  struct Pair {
    std::size_t visual_index = 0;
    std::size_t sonar_index = 0;
    double cost = 0.0;
    double max_cost = 0.0;
    bool gated_in = false;
    std::optional<TargetMeasurement> fused;
    AssociationDiagnostic diagnostic;
  };
  std::vector<Pair> pairs;
  for (std::size_t vi = 0; vi < projected_visual.size(); ++vi) {
    for (std::size_t si = 0; si < projected_sonar.size(); ++si) {
      const auto& v = projected_visual[vi].measurement;
      const auto& s = projected_sonar[si].measurement;
      Pair pair;
      pair.visual_index = vi;
      pair.sonar_index = si;
      pair.diagnostic.visual_observation_id = projected_visual[vi].id;
      pair.diagnostic.sonar_observation_id = projected_sonar[si].id;
      const double time_delta = std::abs(v.corrected_time_s - s.corrected_time_s);
      const double bearing_delta = WrapBearing(v.bearing_rad - s.bearing_rad);
      const double bearing_variance = v.covariance(0, 0) + s.covariance(0, 0);

      if (time_delta > params_.max_corrected_time_delta_s) {
        pair.diagnostic.reason = AssociationReason::kCorrectedTimeDelta;
        pair.diagnostic.metric = AssociationMetric::kCorrectedTimeDeltaSeconds;
        pair.diagnostic.value = time_delta;
        pair.diagnostic.threshold = params_.max_corrected_time_delta_s;
      } else if (!ClassesCompatible(v.class_label, s.class_label)) {
        pair.diagnostic.reason = AssociationReason::kClassIncompatible;
        pair.diagnostic.metric = AssociationMetric::kCompatibility;
        pair.diagnostic.value = 0.0;
        pair.diagnostic.threshold = 1.0;
      } else {
        double worst_variance_ratio = 0.0;
        double worst_variance = 0.0;
        double worst_variance_threshold = 1.0;
        const auto consider_variance = [&](double variance, double threshold) {
          const double ratio = variance / threshold;
          if (ratio > worst_variance_ratio) {
            worst_variance_ratio = ratio;
            worst_variance = variance;
            worst_variance_threshold = threshold;
          }
        };
        consider_variance(v.covariance(0, 0),
                          params_.max_bearing_variance_rad2);
        consider_variance(s.covariance(0, 0),
                          params_.max_bearing_variance_rad2);
        if (v.range_m) {
          consider_variance(v.covariance(1, 1),
                            params_.max_range_variance_m2);
        }
        if (s.range_m) {
          consider_variance(s.covariance(1, 1),
                            params_.max_range_variance_m2);
        }
        if (worst_variance_ratio > 1.0) {
          pair.diagnostic.reason = AssociationReason::kUncertainty;
          pair.diagnostic.metric = AssociationMetric::kVariance;
          pair.diagnostic.value = worst_variance;
          pair.diagnostic.threshold = worst_variance_threshold;
        } else {
          bool statistical_gate_passed = false;
          if (v.range_m && s.range_m) {
            const Eigen::Vector2d residual(
                bearing_delta, *v.range_m - *s.range_m);
            const Eigen::Matrix2d innovation_covariance =
                v.covariance + s.covariance;
            Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_covariance);
            if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
              pair.diagnostic.reason = AssociationReason::kUncertainty;
              pair.diagnostic.metric = AssociationMetric::kInputValidity;
              pair.diagnostic.value = 0.0;
              pair.diagnostic.threshold = 1.0;
            } else {
              const Eigen::Vector2d whitened = ldlt.solve(residual);
              const double joint_mahalanobis = residual.dot(whitened);
              const double joint_threshold =
                  params_.max_bearing_mahalanobis_sq +
                  params_.max_range_mahalanobis_sq;
              if (ldlt.info() != Eigen::Success || !whitened.allFinite() ||
                  !std::isfinite(joint_mahalanobis) ||
                  joint_mahalanobis < 0.0) {
                pair.diagnostic.reason = AssociationReason::kUncertainty;
                pair.diagnostic.metric = AssociationMetric::kInputValidity;
                pair.diagnostic.value = 0.0;
                pair.diagnostic.threshold = 1.0;
              } else if (joint_mahalanobis > joint_threshold) {
                pair.diagnostic.reason = AssociationReason::kJointMahalanobis;
                pair.diagnostic.metric =
                    AssociationMetric::kJointMahalanobisSquared;
                pair.diagnostic.value = joint_mahalanobis;
                pair.diagnostic.threshold = joint_threshold;
              } else {
                pair.cost = joint_mahalanobis;
                pair.max_cost = joint_threshold + 1.0;
                statistical_gate_passed = true;
              }
            }
          } else {
            const double bearing_mahalanobis =
                bearing_delta * bearing_delta /
                std::max(kMinVariance, bearing_variance);
            if (bearing_mahalanobis >
                params_.max_bearing_mahalanobis_sq) {
              pair.diagnostic.reason = AssociationReason::kBearingMahalanobis;
              pair.diagnostic.metric =
                  AssociationMetric::kBearingMahalanobisSquared;
              pair.diagnostic.value = bearing_mahalanobis;
              pair.diagnostic.threshold =
                  params_.max_bearing_mahalanobis_sq;
            } else {
              pair.cost = bearing_mahalanobis;
              pair.max_cost = params_.max_bearing_mahalanobis_sq + 1.0;
              statistical_gate_passed = true;
            }
          }
          if (!statistical_gate_passed) {
            pairs.push_back(std::move(pair));
            continue;
          }
          const double motion_threshold = params_.max_motion_bearing_delta_rad +
                                          params_.max_motion_rate_rad_s * time_delta;
          if (std::abs(bearing_delta) > motion_threshold) {
            pair.diagnostic.reason = AssociationReason::kMotionContinuity;
            pair.diagnostic.metric =
                AssociationMetric::kMotionBearingDeltaRadians;
            pair.diagnostic.value = std::abs(bearing_delta);
            pair.diagnostic.threshold = motion_threshold;
          } else {
            pair.cost += time_delta * time_delta /
                         (params_.max_corrected_time_delta_s *
                          params_.max_corrected_time_delta_s);
            pair.cost += bearing_delta * bearing_delta /
                         (motion_threshold * motion_threshold);
            pair.max_cost += 1.0;
            pair.fused = Fuse(v, s);
            if (!pair.fused) {
              pair.diagnostic.reason = AssociationReason::kUncertainty;
              pair.diagnostic.metric = AssociationMetric::kInputValidity;
              pair.diagnostic.value = 0.0;
              pair.diagnostic.threshold = 1.0;
            } else {
              pair.gated_in = true;
            }
          }
        }
      }
      pairs.push_back(std::move(pair));
    }
  }

  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    if (pairs[i].gated_in) eligible.push_back(i);
  }
  std::sort(eligible.begin(), eligible.end(), [&](std::size_t lhs, std::size_t rhs) {
    return std::tie(pairs[lhs].cost, pairs[lhs].diagnostic.visual_observation_id,
                    pairs[lhs].diagnostic.sonar_observation_id) <
           std::tie(pairs[rhs].cost, pairs[rhs].diagnostic.visual_observation_id,
                    pairs[rhs].diagnostic.sonar_observation_id);
  });
  std::vector<bool> used_visual(projected_visual.size(), false);
  std::vector<bool> used_sonar(projected_sonar.size(), false);
  std::vector<double> winning_visual_cost(
      projected_visual.size(), std::numeric_limits<double>::infinity());
  std::vector<double> winning_sonar_cost(
      projected_sonar.size(), std::numeric_limits<double>::infinity());
  for (std::size_t index : eligible) {
    auto& pair = pairs[index];
    if (!used_visual[pair.visual_index] && !used_sonar[pair.sonar_index]) {
      used_visual[pair.visual_index] = true;
      used_sonar[pair.sonar_index] = true;
      pair.diagnostic.accepted = true;
      pair.diagnostic.reason = AssociationReason::kAccepted;
      pair.diagnostic.metric = AssociationMetric::kPairCost;
      pair.diagnostic.value = pair.cost;
      pair.diagnostic.threshold = pair.max_cost;
      winning_visual_cost[pair.visual_index] = pair.cost;
      winning_sonar_cost[pair.sonar_index] = pair.cost;
      result.measurements.push_back(*pair.fused);
    } else {
      pair.diagnostic.reason = AssociationReason::kPairConflict;
      pair.diagnostic.metric = AssociationMetric::kWinningPairCost;
      pair.diagnostic.value = pair.cost;
      pair.diagnostic.threshold = std::min(
          winning_visual_cost[pair.visual_index],
          winning_sonar_cost[pair.sonar_index]);
    }
  }
  for (const auto& pair : pairs) result.diagnostics.push_back(pair.diagnostic);
  for (std::size_t i = 0; i < projected_visual.size(); ++i) {
    if (!used_visual[i]) {
      result.measurements.push_back(projected_visual[i].measurement);
      result.diagnostics.push_back(
          SingleSourceDecision(projected_visual[i], true));
    }
  }
  for (std::size_t i = 0; i < projected_sonar.size(); ++i) {
    if (!used_sonar[i]) {
      result.measurements.push_back(projected_sonar[i].measurement);
      result.diagnostics.push_back(
          SingleSourceDecision(projected_sonar[i], false));
    }
  }
  std::sort(result.measurements.begin(), result.measurements.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::tie(lhs.corrected_time_s, lhs.bearing_rad) <
                         std::tie(rhs.corrected_time_s, rhs.bearing_rad) ||
                     (lhs.corrected_time_s == rhs.corrected_time_s &&
                      lhs.bearing_rad == rhs.bearing_rad && FirstId(lhs) < FirstId(rhs));
            });
  std::sort(result.diagnostics.begin(), result.diagnostics.end(), DiagnosticLess);
  return result;
}

}  // namespace uw::frontends
