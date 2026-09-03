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

namespace {
// [vx,vy,vz, bgx,bgy,bgz, bax,bay,baz] — the layout ImuPreintegrationResidual
// reads from its inertial parameter blocks. Kept in one place so the two
// files cannot drift apart silently.
void WriteInertial(const PoseGraphProblem::InertialState& state, double* params) {
  for (int i = 0; i < 3; ++i) params[i] = state.velocity_W(i);
  for (int i = 0; i < 3; ++i) params[3 + i] = state.bias_gyro(i);
  for (int i = 0; i < 3; ++i) params[6 + i] = state.bias_accel(i);
}
PoseGraphProblem::InertialState ReadInertial(const double* params) {
  PoseGraphProblem::InertialState state;
  state.velocity_W = Eigen::Vector3d(params[0], params[1], params[2]);
  state.bias_gyro = Eigen::Vector3d(params[3], params[4], params[5]);
  state.bias_accel = Eigen::Vector3d(params[6], params[7], params[8]);
  return state;
}
}  // namespace

void PoseGraphProblem::AddInertialState(const std::string& keyframe_id,
                                         const InertialState& initial_state, bool fixed) {
  if (keyframes_.count(keyframe_id) == 0) {
    throw std::out_of_range("unknown keyframe: " + keyframe_id);
  }
  if (inertial_states_.count(keyframe_id) > 0) return;
  Inertial inertial;
  WriteInertial(initial_state, inertial.params.data());
  inertial.fixed = fixed;
  inertial_states_.emplace(keyframe_id, inertial);
  inertial_order_.push_back(keyframe_id);
}

void PoseGraphProblem::SetInertialState(const std::string& keyframe_id,
                                         const InertialState& state) {
  auto it = inertial_states_.find(keyframe_id);
  if (it == inertial_states_.end()) {
    throw std::out_of_range("keyframe has no inertial state: " + keyframe_id);
  }
  WriteInertial(state, it->second.params.data());
}

PoseGraphProblem::InertialState PoseGraphProblem::GetInertialState(
    const std::string& keyframe_id) const {
  auto it = inertial_states_.find(keyframe_id);
  if (it == inertial_states_.end()) {
    throw std::out_of_range("keyframe has no inertial state: " + keyframe_id);
  }
  return ReadInertial(it->second.params.data());
}

bool PoseGraphProblem::HasInertialState(const std::string& keyframe_id) const {
  return inertial_states_.count(keyframe_id) > 0;
}

bool PoseGraphProblem::IsInertialFixed(const std::string& keyframe_id) const {
  auto it = inertial_states_.find(keyframe_id);
  return it != inertial_states_.end() && it->second.fixed;
}

void PoseGraphProblem::AddResidualBlock(std::unique_ptr<uw::measurement_api::ResidualBlock> block,
                                         std::vector<std::string> involved_keyframes,
                                         RobustPolicy robust_policy) {
  std::vector<ParameterRef> refs;
  refs.reserve(involved_keyframes.size());
  for (auto& id : involved_keyframes) refs.push_back(ParameterRef{ParameterKind::kPose, std::move(id)});
  AddResidualBlockOnParameters(std::move(block), std::move(refs), robust_policy);
}

void PoseGraphProblem::AddResidualBlockOnParameters(
    std::unique_ptr<uw::measurement_api::ResidualBlock> block,
    std::vector<ParameterRef> involved_parameters, RobustPolicy robust_policy) {
  for (const auto& ref : involved_parameters) {
    if (ref.kind == ParameterKind::kPose) {
      if (keyframes_.count(ref.keyframe_id) == 0) {
        throw std::out_of_range("unknown keyframe: " + ref.keyframe_id);
      }
    } else if (inertial_states_.count(ref.keyframe_id) == 0) {
      throw std::out_of_range("keyframe has no inertial state: " + ref.keyframe_id);
    }
  }
  residual_blocks_.push_back(
      Binding{std::move(block), std::move(involved_parameters), robust_policy});
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

std::vector<PoseGraphProblem::ParameterBlockView> PoseGraphProblem::MutableAllParameterBlocks() {
  std::vector<ParameterBlockView> blocks;
  blocks.reserve(order_.size() + inertial_order_.size());
  for (const auto& id : order_) {
    auto& kf = keyframes_.at(id);
    blocks.push_back(ParameterBlockView{ParameterRef{ParameterKind::kPose, id}, kf.params.data(),
                                         7, kf.fixed});
  }
  for (const auto& id : inertial_order_) {
    auto& inertial = inertial_states_.at(id);
    blocks.push_back(ParameterBlockView{ParameterRef{ParameterKind::kInertial, id},
                                         inertial.params.data(), kInertialBlockDim,
                                         inertial.fixed});
  }
  return blocks;
}

std::vector<PoseGraphProblem::ResidualBinding> PoseGraphProblem::ResidualBindings() const {
  std::vector<ResidualBinding> bindings;
  bindings.reserve(residual_blocks_.size());
  for (const auto& binding : residual_blocks_) {
    bindings.push_back(
        ResidualBinding{binding.block.get(), &binding.involved_parameters, binding.robust_policy});
  }
  return bindings;
}

}  // namespace uw::estimation
