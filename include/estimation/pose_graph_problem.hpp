#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "measurement_api/residual_block.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::estimation {

// The v1 graph's optimization variables are keyframe poses ONLY — no
// jointly-optimized 3D landmarks (matches
// holoocean-to-acoustic-optic-slam-pipeline section 10.4.5: "第一版图变量
//只包含关键帧位姿"). The sonar_range_factor's (include/factor_builders/sonar_range_residual.hpp)
// landmark cluster is supplied as fixed external context
// (FactorBuildContext), not as graph variables here.
class PoseGraphProblem {
 public:
  // Per-residual-block robust loss policy, attached at AddResidualBlock
  // (mirroring Ceres's own separation of LossFunction from CostFunction —
  // ResidualBlock::Evaluate() itself stays pure math, see
  // measurement_api/residual_block.hpp). kNone (the default) preserves
  // today's behavior exactly: GaussNewtonSolver::EvaluateAll only applies
  // the Huber reweighting for kHuber bindings, so every pre-existing
  // factor stays bit-identical. Deliberately proto-free (no
  // uw::domain::RobustPolicyHint dependency here) to match
  // ResidualBlock's own convention.
  enum class RobustPolicy { kNone, kHuber };

  // PREP-B-01 state extension, option A of
  // docs/imu-preintegration-design-2026-09-03.md section 6: a keyframe may
  // carry a SECOND, independent 9-parameter inertial state
  // [vx,vy,vz, bgx,bgy,bgz, bax,bay,baz] (world-frame velocity, body-frame
  // gyro/accel biases) alongside its 7-parameter pose. The two are separate
  // parameter blocks, not one widened block, so that:
  //   - every pre-existing residual keeps ParameterBlockSizes() == {7,...}
  //     and every pre-existing factor_builder is untouched;
  //   - a graph with no inertial states at all takes exactly the same code
  //     path with exactly the same column layout as before this existed
  //     (pose block k still owns columns 7k..7k+6), so the synthetic_smoke*
  //     ATE numbers cannot move.
  // Inertial states are added ONLY when an IMU frontend actually produced
  // evidence for that keyframe — an inertial block that no factor
  // constrains is singular (design note section 7).
  enum class ParameterKind { kPose, kInertial };

  // One entry of a residual block's parameter binding. Deliberately NOT
  // implicitly constructible from a string: the older
  // AddResidualBlock(block, std::vector<std::string>) overload stays
  // unambiguous that way.
  struct ParameterRef {
    ParameterKind kind = ParameterKind::kPose;
    std::string keyframe_id;
  };
  static ParameterRef PoseRef(std::string keyframe_id) {
    return ParameterRef{ParameterKind::kPose, std::move(keyframe_id)};
  }
  static ParameterRef InertialRef(std::string keyframe_id) {
    return ParameterRef{ParameterKind::kInertial, std::move(keyframe_id)};
  }

  // Inertial state layout on the wire between here and the residual.
  static constexpr int kInertialBlockDim = 9;
  struct InertialState {
    Eigen::Vector3d velocity_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();
  };

  // Registers the inertial state of an ALREADY-registered keyframe (no-op
  // if it already has one; throws std::out_of_range for an unknown
  // keyframe). `fixed` is independent of the pose's fixed flag: the anchor
  // keyframe's pose is normally fixed to remove gauge freedom while its
  // velocity/bias stay free (design note section 7 — do not pin v_0 = 0
  // unless a stationary-start detector says so; that is the inertial twin
  // of the z-anchor trap in CLAUDE.md).
  void AddInertialState(const std::string& keyframe_id, const InertialState& initial_state,
                        bool fixed = false);
  void SetInertialState(const std::string& keyframe_id, const InertialState& state);
  InertialState GetInertialState(const std::string& keyframe_id) const;
  bool HasInertialState(const std::string& keyframe_id) const;
  bool IsInertialFixed(const std::string& keyframe_id) const;
  const std::vector<std::string>& InertialStateOrder() const { return inertial_order_; }
  std::size_t NumInertialStates() const { return inertial_order_.size(); }

  // Registers a keyframe if not already present (no-op if it already
  // exists). `fixed` keyframes are excluded from the optimization —
  // used to remove gauge freedom by anchoring one keyframe per window.
  void AddKeyframe(const std::string& keyframe_id, uw::sensor_models::Pose3 initial_pose,
                    bool fixed = false);

  void SetKeyframePose(const std::string& keyframe_id, const uw::sensor_models::Pose3& pose);
  uw::sensor_models::Pose3 GetKeyframePose(const std::string& keyframe_id) const;
  bool HasKeyframe(const std::string& keyframe_id) const;
  bool IsFixed(const std::string& keyframe_id) const;

  // `involved_keyframes` order MUST match the residual block's
  // ParameterBlockSizes() order. This pose-only overload is what every
  // factor_builder written before PREP-B-01 uses; it forwards to the
  // ParameterRef overload with every entry tagged kPose.
  void AddResidualBlock(std::unique_ptr<uw::measurement_api::ResidualBlock> block,
                        std::vector<std::string> involved_keyframes,
                        RobustPolicy robust_policy = RobustPolicy::kNone);

  // Mixed pose/inertial binding, e.g. the IMU preintegration factor's
  // {pose_i, inertial_i, pose_j, inertial_j}. Throws std::out_of_range if
  // any referenced pose/inertial state is not registered.
  //
  // Deliberately a DIFFERENT NAME rather than an overload of
  // AddResidualBlock: a braced `{"kf0", "kf1"}` argument is ambiguous
  // between vector<string>'s initializer_list constructor and
  // vector<ParameterRef>'s (InputIt, InputIt) constructor (const char* is a
  // valid input iterator as far as overload resolution is concerned), which
  // would break every existing call site.
  void AddResidualBlockOnParameters(std::unique_ptr<uw::measurement_api::ResidualBlock> block,
                                    std::vector<ParameterRef> involved_parameters,
                                    RobustPolicy robust_policy = RobustPolicy::kNone);

  const std::vector<std::string>& KeyframeOrder() const { return order_; }
  std::size_t NumKeyframes() const { return order_.size(); }
  std::size_t NumResidualBlocks() const { return residual_blocks_.size(); }

  // Backend-agnostic solver accessors, replacing the earlier single
  // `friend class GaussNewtonSolver` — any solver (GaussNewtonSolver, a
  // future Ceres/GTSAM adapter) reads/mutates the graph through these two
  // methods instead of being individually friended. `params` points at the
  // same 7 contiguous doubles Pose3::ToParameterBlock()/FromParameterBlock()
  // use (tx,ty,tz,qx,qy,qz,qw); the pointer stays valid until the next call
  // that adds a keyframe (AddKeyframe may rehash keyframes_) or destroys the
  // problem, matching the lifetime a solver already needs it for (one
  // Solve() call).
  struct KeyframeParameterBlock {
    std::string keyframe_id;
    double* params;
    bool fixed;
  };
  // Order matches KeyframeOrder(). Poses only — kept as-is so callers that
  // predate inertial states (and only ever want poses) do not change.
  std::vector<KeyframeParameterBlock> MutableParameterBlocks();

  // Every optimizable block, poses first (in KeyframeOrder()) then inertial
  // states (in InertialStateOrder()). `size` is 7 for kPose and
  // kInertialBlockDim (9) for kInertial. Solvers use this one; the
  // poses-first ordering is what keeps a pose-only graph's column layout
  // byte-identical to the pre-PREP-B-01 layout.
  struct ParameterBlockView {
    ParameterRef ref;
    double* params;
    int size;
    bool fixed;
  };
  std::vector<ParameterBlockView> MutableAllParameterBlocks();

  // One residual block's binding to its involved parameter blocks, in the
  // same order as ResidualBlock::ParameterBlockSizes().
  // `involved_parameters` points at AddResidualBlock's stored vector, valid
  // for the same lifetime as `block`.
  struct ResidualBinding {
    uw::measurement_api::ResidualBlock* block;
    const std::vector<ParameterRef>* involved_parameters;
    RobustPolicy robust_policy;
  };
  std::vector<ResidualBinding> ResidualBindings() const;

 private:
  struct Keyframe {
    std::array<double, 7> params{0, 0, 0, 0, 0, 0, 1};
    bool fixed = false;
  };
  struct Inertial {
    std::array<double, kInertialBlockDim> params{0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool fixed = false;
  };
  struct Binding {
    std::unique_ptr<uw::measurement_api::ResidualBlock> block;
    std::vector<ParameterRef> involved_parameters;
    RobustPolicy robust_policy = RobustPolicy::kNone;
  };

  std::unordered_map<std::string, Keyframe> keyframes_;
  std::vector<std::string> order_;  // insertion order
  std::unordered_map<std::string, Inertial> inertial_states_;
  std::vector<std::string> inertial_order_;  // insertion order
  std::vector<Binding> residual_blocks_;
};

}  // namespace uw::estimation
