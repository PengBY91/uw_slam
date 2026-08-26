#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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
  // ParameterBlockSizes() order.
  void AddResidualBlock(std::unique_ptr<uw::measurement_api::ResidualBlock> block,
                        std::vector<std::string> involved_keyframes,
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
  // Order matches KeyframeOrder().
  std::vector<KeyframeParameterBlock> MutableParameterBlocks();

  // One residual block's binding to its involved keyframe ids, in the same
  // order as ResidualBlock::ParameterBlockSizes(). `involved_keyframes`
  // points at AddResidualBlock's stored vector, valid for the same lifetime
  // as `block`.
  struct ResidualBinding {
    uw::measurement_api::ResidualBlock* block;
    const std::vector<std::string>* involved_keyframes;
    RobustPolicy robust_policy;
  };
  std::vector<ResidualBinding> ResidualBindings() const;

 private:
  struct Keyframe {
    std::array<double, 7> params{0, 0, 0, 0, 0, 0, 1};
    bool fixed = false;
  };
  struct Binding {
    std::unique_ptr<uw::measurement_api::ResidualBlock> block;
    std::vector<std::string> involved_keyframes;
    RobustPolicy robust_policy = RobustPolicy::kNone;
  };

  std::unordered_map<std::string, Keyframe> keyframes_;
  std::vector<std::string> order_;  // insertion order
  std::vector<Binding> residual_blocks_;
};

}  // namespace uw::estimation
