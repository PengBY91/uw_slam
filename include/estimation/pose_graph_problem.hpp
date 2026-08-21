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
                        std::vector<std::string> involved_keyframes);

  const std::vector<std::string>& KeyframeOrder() const { return order_; }
  std::size_t NumKeyframes() const { return order_.size(); }
  std::size_t NumResidualBlocks() const { return residual_blocks_.size(); }

 private:
  friend class GaussNewtonSolver;

  struct Keyframe {
    std::array<double, 7> params{0, 0, 0, 0, 0, 0, 1};
    bool fixed = false;
  };
  struct Binding {
    std::unique_ptr<uw::measurement_api::ResidualBlock> block;
    std::vector<std::string> involved_keyframes;
  };

  std::unordered_map<std::string, Keyframe> keyframes_;
  std::vector<std::string> order_;  // insertion order
  std::vector<Binding> residual_blocks_;
};

}  // namespace uw::estimation
