#include <cstdlib>
#include <iostream>
#include <string>

#include "application/replay_pipeline.hpp"

#ifndef UW_GIT_COMMIT
#define UW_GIT_COMMIT "unknown"
#endif

int main(int argc, char** argv) {
  uw::application::ReplayOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << argument << "\n";
        std::exit(1);
      }
      return argv[++i];
    };

    if (argument == "--bag") {
      options.bag_path = next();
    } else if (argument == "--experiment") {
      options.experiment_path = next();
    } else if (argument == "--out") {
      options.out_prefix = next();
    } else if (argument == "--max-iterations") {
      options.max_iterations = std::stoi(next());
    } else if (argument == "--align-ate") {
      options.align_ate = true;
    } else {
      std::cerr << "unknown argument: " << argument << "\n";
      return 1;
    }
  }

  if (options.bag_path.empty()) {
    std::cerr << "usage: replay_demo --bag <path.mcap> [--experiment <path.yaml>] "
                 "[--out <prefix>] [--max-iterations N] [--align-ate]\n";
    return 1;
  }
  return uw::application::RunReplayPipeline(options, UW_GIT_COMMIT);
}
