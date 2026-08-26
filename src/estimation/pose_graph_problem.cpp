#include "estimation/pose_graph_problem.hpp"

#include <stdexcept>

namespace uw::estimation {

void PoseGraphProblem::AddKeyframe(const std::string& keyframe_id,
                                    uw::sensor_models::Pose3 initial_pose, bool fixed) {
  if (keyframes_.count(keyframe_id) > 0) return;
  Keyframe kf;
  kf.params = initial_pose.ToParameterBlock();
  kf.fixed = fixed;
  keyframes_.emplace(keyframe_id, kf);
  order_.push_back(keyframe_id);
}

void PoseGraphProblem::SetKeyframePose(const std::string& keyframe_id,
                                        const uw::sensor_models::Pose3& pose) {
  auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) throw std::out_of_range("unknown keyframe: " + keyframe_id);
  it->second.params = pose.ToParameterBlock();
}

uw::sensor_models::Pose3 PoseGraphProblem::GetKeyframePose(const std::string& keyframe_id) const {
  auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) throw std::out_of_range("unknown keyframe: " + keyframe_id);
  return uw::sensor_models::Pose3::FromParameterBlock(it->second.params.data());
}

bool PoseGraphProblem::HasKeyframe(const std::string& keyframe_id) const {
  return keyframes_.count(keyframe_id) > 0;
}

bool PoseGraphProblem::IsFixed(const std::string& keyframe_id) const {
  auto it = keyframes_.find(keyframe_id);
  return it != keyframes_.end() && it->second.fixed;
}

void PoseGraphProblem::AddResidualBlock(std::unique_ptr<uw::measurement_api::ResidualBlock> block,
                                         std::vector<std::string> involved_keyframes,
                                         RobustPolicy robust_policy) {
  for (const auto& id : involved_keyframes) {
    if (keyframes_.count(id) == 0) throw std::out_of_range("unknown keyframe: " + id);
  }
  residual_blocks_.push_back(Binding{std::move(block), std::move(involved_keyframes), robust_policy});
}

std::vector<PoseGraphProblem::KeyframeParameterBlock> PoseGraphProblem::MutableParameterBlocks() {
  std::vector<KeyframeParameterBlock> blocks;
  blocks.reserve(order_.size());
  for (const auto& id : order_) {
    auto& kf = keyframes_.at(id);
    blocks.push_back(KeyframeParameterBlock{id, kf.params.data(), kf.fixed});
  }
  return blocks;
}

std::vector<PoseGraphProblem::ResidualBinding> PoseGraphProblem::ResidualBindings() const {
  std::vector<ResidualBinding> bindings;
  bindings.reserve(residual_blocks_.size());
  for (const auto& binding : residual_blocks_) {
    bindings.push_back(ResidualBinding{binding.block.get(), &binding.involved_keyframes, binding.robust_policy});
  }
  return bindings;
}

}  // namespace uw::estimation
