// 把有序的 CanonicalEvent 流（来自任意 EventSource——今天是 MCAP 回放，以后可能是
// 实时 SDK 源）攒成一份扁平的、身份已校验过的 ReplayInputData，供 RunReplayPipeline
// 后续的求解 / 建图 / 评测逻辑直接消费。
//
// 它是 PipelineInputPort 的一个实现，本身不做任何算法，只干两件事：**分门别类地攒**
// 和 **校验身份**。
//
// 关键设计（见 docs/archive/superpowers/plans/2026-08-24-live-replay-unified-ingress.md
// Task 4）：这里替换掉了 RunReplayPipeline 早期那套"拿 capture_time 除以 0.2s 反推
// keyframe id"的做法。现在每一个身份都直接来自线上字段
// （ObservationHeader.observation_id、MeasurementEvidence.source_observations），
// **绝不从时间戳推导**——时间推身份在丢帧、抖动、变帧率下都会悄悄错位。
#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"

namespace uw::application {

struct ReplayInputData {
  std::vector<uw::domain::ImageFrame> images;
  std::vector<uw::domain::SonarFrame> sonar_frames;
  std::vector<uw::domain::ImuSample> imu_samples;
  std::vector<uw::domain::DvlSample> dvl_samples;
  std::vector<uw::domain::VehicleState> vehicle_states;
  std::vector<uw::domain::KeyframeBoundary> keyframe_boundaries;
  std::vector<uw::domain::MeasurementEvidence> evidence;
  std::vector<uw::domain::StateSnapshot> reference_states;
};

// 坏身份绝不静默丢弃（完成标准 #3）：每一条被拒的记录都恰好让这里的某一个计数器
// +1，并追加一条人能读懂的说明。这样调用方既可以用 HasErrors() 做门禁判断，也能把
// 具体问题打出来定位——只计数不说明，或者只打日志不计数，都不够。
struct ReplayInputDiagnostics {
  uint64_t empty_observation_id_count = 0;
  uint64_t duplicate_observation_count = 0;
  uint64_t empty_keyframe_id_count = 0;
  uint64_t duplicate_keyframe_id_count = 0;
  uint64_t non_increasing_keyframe_capture_time_count = 0;
  uint64_t dangling_evidence_reference_count = 0;
  std::vector<std::string> messages;

  bool HasErrors() const {
    return empty_observation_id_count > 0 || duplicate_observation_count > 0 ||
          empty_keyframe_id_count > 0 || duplicate_keyframe_id_count > 0 ||
          non_increasing_keyframe_capture_time_count > 0 ||
          dangling_evidence_reference_count > 0;
  }
};

class ReplayInputAccumulator final : public PipelineInputPort {
 public:
  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnImuSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnDvlSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnVehicleState(const uw::runtime::CanonicalEvent& event) override;
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) override;
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool OnReferenceState(const uw::runtime::CanonicalEvent& event) override;
  // 目前没有任何生产者会发 /health 和 /evidence/map（见 canonical_topics.hpp），
  // ReplayInputData 也刻意没有给它们留字段。所以这两个是"收下并忽略"，等真的出现
  // 消费者再说——按计划里的停止条件：不为假想的需求先把结构体撑大。
  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) override;
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) override;

  // 拿本次跑到的全部原始观测身份，去校验每条已累积证据的 source_observations
  // （即检查"证据引用的原始观测确实存在"）。
  //
  // 之所以推迟到 Flush 而不是每条事件当场查：按 log_time_ns 排序时，一条证据完全
  // 可能先于它所引用的原始观测到达，当场查会误报。
  bool Flush() override;

  const ReplayInputData& Data() const { return data_; }
  const ReplayInputDiagnostics& Diagnostics() const { return diagnostics_; }

  // 与 Data().evidence 按下标一一对应：每条证据到达时所带的
  // CanonicalEvent::log_time_ns。
  //
  // 它存在的唯一理由是 MeasurementEvidence 不像 ImageFrame/SonarFrame 那样在线上
  // 自带 header/时间戳。RunReplayPipeline 在"相机 capture_time 和真值都覆盖不到某个
  // keyframe"时，拿它当最后兜底的时间戳（见那边 capture_time_by_keyframe 的优先级
  // 分层注释）。
  const std::vector<uint64_t>& EvidenceLogTimeNs() const { return evidence_log_time_ns_; }

 private:
  // observation_id 合法则返回 true（并把这个身份登记为"已知"）；为空则返回 false
  // 并追加一条诊断。
  //
  // 对于已经见过的 (sensor_id, observation_id) 组合，默认拒绝，除非传了
  // allow_duplicate_identity：
  //   - SonarFrame 传 true。因为 synth_bag_gen / HoloOcean 的声呐模型会在同一个
  //     物理观测身份下合理地发出多个 ping（量程内每个目标一条），这是建模方式，
  //     不是数据错误（对应 replay_pipeline.cpp 里那条 v1 规则："每个 keyframe 只
  //     保留见到的第一帧声呐"）。
  //   - ImageFrame / ImuSample / DvlSample / VehicleState 传 false。这几类出现重复
  //     就是真正的数据完整性问题，不是可接受的建模选择。
  bool ValidateRawIdentity(const std::string& sensor_id, const std::string& observation_id,
                           const std::string& kind_label, bool allow_duplicate_identity);

  ReplayInputData data_;
  ReplayInputDiagnostics diagnostics_;
  std::unordered_set<std::string> seen_raw_identities_;    // 键是 "sensor_id\x1Fobservation_id"
  std::unordered_set<std::string> known_observation_ids_;  // 所有原始观测类型的并集
  std::unordered_set<std::string> seen_keyframe_ids_;
  std::vector<uint64_t> evidence_log_time_ns_;              // 与 data_.evidence 平行
};

}  // namespace uw::application
