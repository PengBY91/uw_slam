// Optical-only baseline harness (plan 2 of the acoustic-optic series):
// StereoOpticalDepthFrontend against a synthetic bag from
// apps/synth_stereo_gen.cpp, scored with uw::evaluation::ComputeDepthMetrics
// against /gt/depth. Does NOT fuse sonar and does NOT touch
// apps/replay_demo's pose-graph loop — see plan 2's scope boundary.
#include <iostream>
#include <optional>
#include <string>

#include "domain/domain.hpp"
#include "evaluation/depth_metrics.hpp"
#include "frontends/stereo_optical_depth_frontend.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_io.hpp"

int main(int argc, char** argv) {
  std::string bag_path;
  std::string experiment_path = "configs/experiment/synthetic_smoke.yaml";
  double max_rmse_m = 0.05;
  double min_coverage = 0.9;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--bag" && i + 1 < argc) {
      bag_path = next();
    } else if (arg == "--experiment" && i + 1 < argc) {
      experiment_path = next();
    } else if (arg == "--max-rmse-m" && i + 1 < argc) {
      max_rmse_m = std::stod(next());
    } else if (arg == "--min-coverage" && i + 1 < argc) {
      min_coverage = std::stod(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }
  if (bag_path.empty()) {
    std::cerr << "--bag is required\n";
    return 1;
  }

  const auto experiment = uw::runtime::LoadExperimentConfig(experiment_path);

  std::optional<uw::domain::ImageFrame> left;
  std::optional<uw::domain::ImageFrame> right;
  std::optional<uw::domain::OpticalDepthPriorMeasurement> gt;

  uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
      bag_path, "/raw/camera/left",
      [&](uint64_t, const uw::domain::ImageFrame& msg) { left = msg; });
  uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
      bag_path, "/raw/camera/right",
      [&](uint64_t, const uw::domain::ImageFrame& msg) { right = msg; });
  uw::runtime::ReadMcapMessages<uw::domain::MeasurementEvidence>(
      bag_path, "/gt/depth", [&](uint64_t, const uw::domain::MeasurementEvidence& msg) {
        if (uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(msg)) {
          gt = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(msg);
        }
      });

  if (!left.has_value() || !right.has_value() || !gt.has_value()) {
    std::cerr << "bag is missing /raw/camera/left, /raw/camera/right, or /gt/depth\n";
    return 1;
  }

  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = *left;
  bundle.secondary = right;

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.matcher.window_radius = 3;
  params.matcher.min_disparity = 1;
  params.matcher.max_disparity = 32;
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  const auto evidence = frontend.Process(bundle, experiment.rig);
  if (!evidence.has_value()) {
    std::cerr << "StereoOpticalDepthFrontend rejected the bundle\n";
    return 1;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);

  uw::evaluation::DepthGrid estimated_grid;
  estimated_grid.width = prior.width();
  estimated_grid.height = prior.height();
  estimated_grid.depth_m.assign(prior.depth_m().begin(), prior.depth_m().end());
  estimated_grid.valid_mask.assign(prior.valid_mask().begin(), prior.valid_mask().end());

  uw::evaluation::DepthGrid gt_grid;
  gt_grid.width = gt->width();
  gt_grid.height = gt->height();
  gt_grid.depth_m.assign(gt->depth_m().begin(), gt->depth_m().end());
  gt_grid.valid_mask.assign(gt->valid_mask().begin(), gt->valid_mask().end());

  const auto metrics = uw::evaluation::ComputeDepthMetrics(estimated_grid, gt_grid);
  std::cout << "rmse_m=" << metrics.rmse_m << " mae_m=" << metrics.mae_m
            << " coverage=" << metrics.valid_coverage_fraction
            << " compared_pixels=" << metrics.num_compared_pixels << "\n";

  if (metrics.rmse_m > max_rmse_m) {
    std::cerr << "FAIL: rmse_m " << metrics.rmse_m << " exceeds max_rmse_m " << max_rmse_m << "\n";
    return 1;
  }
  if (metrics.valid_coverage_fraction < min_coverage) {
    std::cerr << "FAIL: coverage " << metrics.valid_coverage_fraction << " below min_coverage "
              << min_coverage << "\n";
    return 1;
  }
  std::cout << "OK\n";
  return 0;
}
