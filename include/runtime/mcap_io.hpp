// Thin, protobuf-aware wrapper around the MCAP C++ SDK. MCAP is this
// platform's canonical bag/recording format (locked decision, see the
// architecture refinement this repo implements): multi-schema, C++/Python
// SDKs, not ROS-locked. This wrapper exists so callers (apps/synth_bag_gen.cpp,
// apps/replay_demo.cpp, adapters/holoocean's future C++
// consumers) never touch mcap::McapWriter/McapReader schema/channel
// bookkeeping directly — they just read/write typed protobuf messages by
// topic.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <mcap/mcap.hpp>

namespace uw::runtime {

// Serializes the transitive closure of .proto file dependencies for
// `descriptor` into a google.protobuf.FileDescriptorSet — this is what
// makes an MCAP "protobuf"-encoded schema self-describing (viewable in
// tools like Foxglove Studio) rather than just a bare type-name label.
std::string BuildFileDescriptorSet(const google::protobuf::Descriptor* descriptor);

class McapProtobufWriter {
 public:
  bool Open(const std::string& path);
  void Close();

  template <typename T>
  bool WriteMessage(const std::string& topic, uint64_t log_time_ns, const T& message) {
    const mcap::ChannelId channel_id = EnsureChannel(topic, T::descriptor());
    std::string payload;
    if (!message.SerializeToString(&payload)) return false;

    mcap::Message msg;
    msg.channelId = channel_id;
    msg.sequence = sequence_by_channel_[channel_id]++;
    msg.logTime = log_time_ns;
    msg.publishTime = log_time_ns;
    msg.dataSize = payload.size();
    msg.data = reinterpret_cast<const std::byte*>(payload.data());
    return writer_.write(msg).ok();
  }

 private:
  mcap::ChannelId EnsureChannel(const std::string& topic, const google::protobuf::Descriptor* descriptor);

  mcap::McapWriter writer_;
  bool open_ = false;
  std::unordered_map<std::string, mcap::SchemaId> schema_id_by_type_name_;
  std::unordered_map<std::string, mcap::ChannelId> channel_id_by_topic_;
  std::unordered_map<mcap::ChannelId, uint32_t> sequence_by_channel_;
  mcap::SchemaId next_schema_id_ = 1;
  mcap::ChannelId next_channel_id_ = 1;
};

// Reads every message on `topic`, deserializing into T and invoking
// `callback(log_time_ns, message)` in file order. Returns false if the file
// could not be opened. Messages that fail to parse as T are skipped (this
// can happen if `topic` is reused across schema versions — not treated as
// a hard error in v1, but callers reading evaluation-critical data should
// check message counts against expectations).
template <typename T>
bool ReadMcapMessages(const std::string& path, const std::string& topic,
                      const std::function<void(uint64_t log_time_ns, const T& message)>& callback) {
  mcap::McapReader reader;
  if (!reader.open(path).ok()) return false;

  for (const auto& view : reader.readMessages()) {
    if (view.channel == nullptr || view.channel->topic != topic) continue;
    T message;
    if (!message.ParseFromArray(view.message.data, static_cast<int>(view.message.dataSize))) continue;
    callback(view.message.logTime, message);
  }
  reader.close();
  return true;
}

}  // namespace uw::runtime
