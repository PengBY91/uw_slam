#pragma once

#include <string>

namespace uw::application {

struct ReplayOptions {
  std::string bag_path;
  std::string experiment_path;
  std::string out_prefix = "/tmp/replay_demo";
  int max_iterations = -1;
  bool align_ate = false;
};

int RunReplayPipeline(const ReplayOptions& options,
                      const std::string& git_commit);

}  // namespace uw::application
