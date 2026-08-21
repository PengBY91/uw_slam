#include "runtime/mcap_io.hpp"

#include <queue>
#include <unordered_set>

#include <google/protobuf/descriptor.pb.h>

namespace uw::runtime {

std::string BuildFileDescriptorSet(const google::protobuf::Descriptor* descriptor) {
  google::protobuf::FileDescriptorSet fd_set;
  std::unordered_set<std::string> visited;
  std::queue<const google::protobuf::FileDescriptor*> to_visit;
  to_visit.push(descriptor->file());

  while (!to_visit.empty()) {
    const auto* file = to_visit.front();
    to_visit.pop();
    if (!visited.insert(std::string(file->name())).second) continue;

    file->CopyTo(fd_set.add_file());
    for (int i = 0; i < file->dependency_count(); ++i) {
      to_visit.push(file->dependency(i));
    }
  }

  std::string bytes;
  const bool ok = fd_set.SerializeToString(&bytes);
  (void)ok;  // best-effort; an empty descriptor set just means a less self-describing schema
  return bytes;
}

bool McapProtobufWriter::Open(const std::string& path) {
  mcap::McapWriterOptions options("");
  const auto status = writer_.open(path, options);
  open_ = status.ok();
  return open_;
}

void McapProtobufWriter::Close() {
  if (open_) {
    writer_.close();
    open_ = false;
  }
}

mcap::ChannelId McapProtobufWriter::EnsureChannel(const std::string& topic,
                                                   const google::protobuf::Descriptor* descriptor) {
  auto channel_it = channel_id_by_topic_.find(topic);
  if (channel_it != channel_id_by_topic_.end()) return channel_it->second;

  const std::string type_name(descriptor->full_name());
  mcap::SchemaId schema_id;
  auto schema_it = schema_id_by_type_name_.find(type_name);
  if (schema_it != schema_id_by_type_name_.end()) {
    schema_id = schema_it->second;
  } else {
    mcap::Schema schema(type_name, "protobuf", BuildFileDescriptorSet(descriptor));
    writer_.addSchema(schema);
    schema_id = schema.id;
    schema_id_by_type_name_[type_name] = schema_id;
  }

  mcap::Channel channel(topic, "protobuf", schema_id);
  writer_.addChannel(channel);
  channel_id_by_topic_[topic] = channel.id;
  sequence_by_channel_[channel.id] = 0;
  return channel.id;
}

}  // namespace uw::runtime
