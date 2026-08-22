// First end-to-end wiring of plans 1-4's real components (no fabricated
// MeasurementEvidence): AcousticOpticSynchronizer -> StereoOpticalDepthFrontend
// -> SonarCfarFrontend (pre-existing) -> AcousticOpticDepthFusionFrontend,
// run over the design spec's 9-scenario matrix (section 10). See this
// plan's header comment for the explicit scope cuts (no MCAP round trip,
// two ablation slices not three, reduced gate set).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "acoustic_optic_scenarios.hpp"
#include "evaluation/depth_metrics.hpp"
#include "evaluation/fusion_metrics.hpp"
#include "frontends/acoustic_optic_depth_fusion_frontend.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "frontends/stereo_optical_depth_frontend.hpp"
#include "runtime/acoustic_optic_synchronizer.hpp"
#include "runtime/config.hpp"

namespace {

struct ScenarioAggregate {
  int trials = 0;
  int sync_rejected = 0;
  int accepted = 0;
  int ambiguous = 0;
  int conflict = 0;
  int rejected_geometric = 0;
  int reason_no_candidate = 0;
  int reason_scale = 0;
  int reason_calibration = 0;
  int reason_posterior_invalid = 0;
  int reason_variance_not_improved = 0;
  int reason_cross_modal_conflict = 0;
  int false_fusions = 0;
  double sum_optical_full_rmse = 0.0;
  double sum_fused_full_rmse = 0.0;
  int full_rmse_samples = 0;
  double sum_optical_covered_rmse = 0.0;
  double sum_fused_covered_rmse = 0.0;
  int covered_rmse_samples = 0;
  std::vector<double> latencies_ms;
};

uw::evaluation::DepthGrid GridFromOptical(const uw::domain::OpticalDepthPriorMeasurement& prior) {
  uw::evaluation::DepthGrid grid;
  grid.width = prior.width();
  grid.height = prior.height();
  grid.depth_m.assign(prior.depth_m().begin(), prior.depth_m().end());
  grid.valid_mask.assign(prior.valid_mask().begin(), prior.valid_mask().end());
  return grid;
}

uw::evaluation::DepthGrid GridFromFused(const uw::domain::FusedDepthMeasurement& fused) {
  uw::evaluation::DepthGrid grid;
  grid.width = fused.width();
  grid.height = fused.height();
  grid.depth_m.assign(fused.depth_m().begin(), fused.depth_m().end());
  grid.valid_mask.assign(fused.valid_mask().begin(), fused.valid_mask().end());
  return grid;
}

// GT grid matching scenarios.cpp's actual scene: gt_target_depth_m inside
// the patch, gt_background_depth_m everywhere else.
uw::evaluation::DepthGrid TwoRegionGt(const uw::scenario_matrix::SyntheticTrial& trial) {
  uw::evaluation::DepthGrid grid;
  grid.width = trial.width;
  grid.height = trial.height;
  grid.depth_m.assign(static_cast<std::size_t>(trial.width) * trial.height, 0.0f);
  grid.valid_mask.assign(static_cast<std::size_t>(trial.width) * trial.height, 1);
  for (uint32_t v = 0; v < trial.height; ++v) {
    for (uint32_t u = 0; u < trial.width; ++u) {
      const bool inside = std::abs(static_cast<int>(u) - trial.patch_center_u) <= trial.patch_half_size &&
                         std::abs(static_cast<int>(v) - trial.patch_center_v) <= trial.patch_half_size;
      grid.depth_m[static_cast<std::size_t>(v) * trial.width + u] =
          static_cast<float>(inside ? trial.gt_target_depth_m : trial.gt_background_depth_m);
    }
  }
  return grid;
}

// Restricts `gt` to the pixels listed in `indices` (the sonar-covered
// region slice — design spec section 12's second reporting slice).
uw::evaluation::DepthGrid RestrictToIndices(const uw::evaluation::DepthGrid& gt,
                                            const std::vector<uint32_t>& indices) {
  uw::evaluation::DepthGrid restricted;
  restricted.width = gt.width;
  restricted.height = gt.height;
  restricted.depth_m = gt.depth_m;
  restricted.valid_mask.assign(gt.valid_mask.size(), 0);
  for (uint32_t idx : indices) {
    if (idx < restricted.valid_mask.size()) restricted.valid_mask[idx] = gt.valid_mask[idx];
  }
  return restricted;
}

}  // namespace

int main(int argc, char** argv) {
  std::string experiment_path = "configs/experiment/synthetic_smoke.yaml";
  uint64_t base_seed = 20260820;
  int trials_per_scenario = 20;
  double max_false_fusion_rate = 0.05;
  int min_accepted_for_gate = 5;
  // <0 = disabled. Current measured P95 (278-308ms, see docs/uw-slam-
  // production-readiness-and-roadmap-2026-08-21.md 2.3) already exceeds the
  // architecture's 200ms aspirational target, so defaulting this on would
  // fail every scenario today for a reason unrelated to correctness — left
  // opt-in until the latency work in that roadmap's P3 lands.
  double max_p95_latency_ms = -1.0;
  // <0 = disabled. Fraction by which fused_covered_rmse must be <= (1 -
  // fraction) * optical_covered_rmse to pass, checked only for scenarios
  // with covered_rmse_samples > 0. Left opt-in for the same reason as the
  // minimum-coverage gate above stays unwired from ctest (docs/uw-slam-
  // production-readiness-and-roadmap-2026-08-21.md 2.3): most scenarios
  // currently produce 0 accepted associations because of a real associator
  // scoring bug, so covered_rmse_samples is 0 for them and this gate would
  // have nothing to check, not a genuine pass/fail signal.
  double min_fusion_improvement_fraction = -1.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--experiment" && i + 1 < argc) {
      experiment_path = next();
    } else if (arg == "--seed" && i + 1 < argc) {
      base_seed = std::stoull(next());
    } else if (arg == "--trials-per-scenario" && i + 1 < argc) {
      trials_per_scenario = std::stoi(next());
    } else if (arg == "--max-p95-latency-ms" && i + 1 < argc) {
      max_p95_latency_ms = std::stod(next());
    } else if (arg == "--min-fusion-improvement-fraction" && i + 1 < argc) {
      min_fusion_improvement_fraction = std::stod(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  const auto experiment = uw::runtime::LoadExperimentConfig(experiment_path);
  const auto& true_rig = experiment.rig;

  bool any_gate_failed = false;
  std::cout << std::fixed << std::setprecision(4);

  for (std::size_t si = 0; si < uw::scenario_matrix::AllScenarios().size(); ++si) {
    const auto& spec = uw::scenario_matrix::AllScenarios()[si];
    ScenarioAggregate agg;

    for (int ti = 0; ti < trials_per_scenario; ++ti) {
      ++agg.trials;
      const uint64_t trial_seed = base_seed * 1000003ULL + static_cast<uint64_t>(si) * 1009 + ti;
      const auto trial = uw::scenario_matrix::BuildTrial(spec.kind, true_rig, trial_seed);

      uw::runtime::SynchronizerParams sync_params;
      const auto sync_bundle = uw::runtime::SynchronizeAcousticOptic(
          trial.left, std::optional<uw::domain::ImageFrame>(trial.right),
          trial.sonar.value_or(uw::domain::SonarFrame{}), trial.pipeline_rig, sync_params);
      // A missing sonar frame (dropout) is not a sync failure; only treat
      // a failed sync as a rejection when a sonar frame actually exists.
      if (trial.sonar.has_value() && !sync_bundle.has_value()) {
        ++agg.sync_rejected;
        continue;
      }

      const auto start = std::chrono::steady_clock::now();

      uw::frontends::StereoOpticalDepthFrontendParams stereo_params;
      uw::frontends::StereoOpticalDepthFrontend stereo(stereo_params);
      uw::measurement_api::CameraFrameBundle bundle;
      bundle.primary = trial.left;
      bundle.secondary = trial.right;
      const auto optical_evidence = stereo.Process(bundle, trial.pipeline_rig);
      if (!optical_evidence.has_value()) {
        ++agg.rejected_geometric;
        continue;
      }

      uw::domain::HypothesisSet sonar_hypotheses;
      if (trial.sonar.has_value()) {
        uw::frontends::SonarCfarFrontendParams cfar_params;
        uw::frontends::SonarCfarFrontend cfar(cfar_params);
        sonar_hypotheses = cfar.ProcessSonarFrame(*trial.sonar);
      }
      // Set SCENARIO_DEBUG=1 to print, for each scenario's first trial, the
      // analytic sonar reading vs. what SonarCfarFrontend actually
      // detected — useful for separating "scene geometry is wrong" from
      // "CFAR detection/quantization drifted" when a scenario's numbers
      // look off (this distinction found a real scene-construction bug
      // during this plan's own development — see MakeStereoPair's header
      // comment in scenarios.cpp).
      if (getenv("SCENARIO_DEBUG") != nullptr && ti == 0) {
        std::cerr << "[debug] " << spec.name << " gt_target_depth_m=" << trial.gt_target_depth_m
                  << " expected sonar range=" << trial.expected_sonar_range_m
                  << " bearing=" << trial.expected_sonar_bearing_rad << "\n";
        if (sonar_hypotheses.candidates_size() > 0 &&
            uw::domain::HasPayload<uw::domain::SonarRangeBearing>(sonar_hypotheses.candidates(0))) {
          const auto& top = uw::domain::GetPayload<uw::domain::SonarRangeBearing>(sonar_hypotheses.candidates(0));
          std::cerr << "[debug] " << spec.name << " detected sonar range=" << top.range_m()
                    << " bearing=" << top.bearing_rad() << "\n";
        } else {
          std::cerr << "[debug] " << spec.name << " no sonar detection this trial\n";
        }
      }

      uw::frontends::AcousticOpticDepthFusionParams fusion_params;
      uw::frontends::AcousticOpticDepthFusionFrontend fusion(fusion_params);
      const double time_delta = sync_bundle.has_value() ? sync_bundle->max_pairwise_time_delta_s : 0.0;
      const auto fused_result = fusion.Fuse(sonar_hypotheses, *optical_evidence, trial.pipeline_rig, time_delta);

      const auto end = std::chrono::steady_clock::now();
      agg.latencies_ms.push_back(std::chrono::duration<double, std::milli>(end - start).count());

      if (!fused_result.has_value()) {
        ++agg.rejected_geometric;
        continue;
      }
      const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_result->fused_evidence);
      const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*optical_evidence);

      const auto gt = TwoRegionGt(trial);
      const auto optical_grid = GridFromOptical(prior);
      const auto fused_grid = GridFromFused(fused);

      const auto optical_full = uw::evaluation::ComputeDepthMetrics(optical_grid, gt);
      const auto fused_full = uw::evaluation::ComputeDepthMetrics(fused_grid, gt);
      if (optical_full.num_compared_pixels > 0) {
        agg.sum_optical_full_rmse += optical_full.rmse_m;
        agg.sum_fused_full_rmse += fused_full.rmse_m;
        ++agg.full_rmse_samples;
      }

      if (fused.associations_size() > 0) {
        const auto& record = fused.associations(0);
        std::vector<uint32_t> covered(record.candidate_pixel_indices().begin(),
                                      record.candidate_pixel_indices().end());
        if (!covered.empty()) {
          const auto gt_covered = RestrictToIndices(gt, covered);
          const auto optical_covered = uw::evaluation::ComputeDepthMetrics(optical_grid, gt_covered);
          const auto fused_covered = uw::evaluation::ComputeDepthMetrics(fused_grid, gt_covered);
          if (optical_covered.num_compared_pixels > 0) {
            agg.sum_optical_covered_rmse += optical_covered.rmse_m;
            agg.sum_fused_covered_rmse += fused_covered.rmse_m;
            ++agg.covered_rmse_samples;
          }
        }

        switch (record.status()) {
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED: {
            ++agg.accepted;
            const double estimated = fused.depth_m(static_cast<int>(record.selected_pixel_index()));
            const auto ff = uw::evaluation::EvaluateFalseFusion(estimated, trial.gt_target_depth_m);
            if (ff.is_false_fusion) ++agg.false_fusions;
            break;
          }
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS:
            ++agg.ambiguous;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT:
            ++agg.conflict;
            ++agg.reason_cross_modal_conflict;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED:
            ++agg.rejected_geometric;
            switch (record.reason()) {
              case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE:
                ++agg.reason_no_candidate;
                break;
              case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE:
                ++agg.reason_scale;
                break;
              case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION:
                ++agg.reason_calibration;
                break;
              case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_POSTERIOR_INVALID:
                ++agg.reason_posterior_invalid;
                break;
              case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED:
                ++agg.reason_variance_not_improved;
                break;
              default:
                break;
            }
            break;
          default:
            break;
        }
      }
    }

    const double false_fusion_rate = agg.accepted > 0 ? static_cast<double>(agg.false_fusions) / agg.accepted : 0.0;
    std::sort(agg.latencies_ms.begin(), agg.latencies_ms.end());
    const double p95_latency =
        agg.latencies_ms.empty() ? 0.0
                                 : agg.latencies_ms[static_cast<std::size_t>(0.95 * (agg.latencies_ms.size() - 1))];

    std::cout << "scenario=" << spec.name << " trials=" << agg.trials << " sync_rejected=" << agg.sync_rejected
              << " accepted=" << agg.accepted << " ambiguous=" << agg.ambiguous << " conflict=" << agg.conflict
              << " rejected=" << agg.rejected_geometric << " [no_candidate=" << agg.reason_no_candidate
              << " scale=" << agg.reason_scale << " calibration=" << agg.reason_calibration
              << " posterior_invalid=" << agg.reason_posterior_invalid
              << " variance_not_improved=" << agg.reason_variance_not_improved
              << " cross_modal_conflict=" << agg.reason_cross_modal_conflict << "]"
              << " false_fusion_rate=" << false_fusion_rate
              << " optical_full_rmse="
              << (agg.full_rmse_samples > 0 ? agg.sum_optical_full_rmse / agg.full_rmse_samples : 0.0)
              << " fused_full_rmse="
              << (agg.full_rmse_samples > 0 ? agg.sum_fused_full_rmse / agg.full_rmse_samples : 0.0)
              << " optical_covered_rmse="
              << (agg.covered_rmse_samples > 0 ? agg.sum_optical_covered_rmse / agg.covered_rmse_samples : 0.0)
              << " fused_covered_rmse="
              << (agg.covered_rmse_samples > 0 ? agg.sum_fused_covered_rmse / agg.covered_rmse_samples : 0.0)
              << " p95_latency_ms=" << p95_latency << "\n";

    if (agg.accepted >= min_accepted_for_gate && false_fusion_rate > max_false_fusion_rate) {
      std::cerr << "GATE FAIL: " << spec.name << " false_fusion_rate " << false_fusion_rate << " exceeds "
                << max_false_fusion_rate << " (accepted=" << agg.accepted << ")\n";
      any_gate_failed = true;
    }

    // Minimum effective coverage gate (docs/uw-slam-production-readiness-
    // and-roadmap-2026-08-21.md 2.3/5.5): the false-fusion gate above is
    // silently skipped whenever accepted < min_accepted_for_gate, which
    // previously let a scenario with 0/N accepted (no candidates at all —
    // e.g. turbid_sonar_visible) print and exit 0 unnoticed. kSonarDropout
    // and kOpticalInvalidRegion are deliberately built (see
    // acoustic_optic_scenarios.cpp) so that fusion has nothing valid to
    // accept — 0 accepted is their correct, expected outcome, not a
    // regression, so they're excluded here. kTimeOffsetFault
    // (acoustic_optic_scenarios.cpp injects a full 1s capture-time offset,
    // meant to be caught by the synchronizer's time gate before the
    // associator ever runs) and kExtrinsicPerturbation (injects a 1.0m
    // sonar-extrinsic error, explicitly documented there as "chosen to
    // unambiguously exceed the associator's default range/bearing gates so
    // this scenario demonstrates fail-closed rejection") are likewise
    // deliberate fail-closed fault-injection scenarios, not associator
    // bugs — see the investigation in this roadmap doc's P0 section for how
    // this was told apart from the real bug that used to also produce 0/N
    // here (clean_textured/elevation_stress, now fixed in
    // acoustic_optic_associator.cpp's depth-agreement check).
    if (spec.kind != uw::scenario_matrix::ScenarioKind::kSonarDropout &&
        spec.kind != uw::scenario_matrix::ScenarioKind::kOpticalInvalidRegion &&
        spec.kind != uw::scenario_matrix::ScenarioKind::kTimeOffsetFault &&
        spec.kind != uw::scenario_matrix::ScenarioKind::kExtrinsicPerturbation && agg.accepted == 0) {
      std::cerr << "GATE FAIL: " << spec.name << " had 0/" << agg.trials
                << " accepted associations — no evidence the fusion pipeline produced any valid "
                   "output in this scenario\n";
      any_gate_failed = true;
    }

    if (max_p95_latency_ms >= 0.0 && p95_latency > max_p95_latency_ms) {
      std::cerr << "GATE FAIL: " << spec.name << " p95_latency_ms " << p95_latency << " exceeds "
                << max_p95_latency_ms << "\n";
      any_gate_failed = true;
    }

    if (min_fusion_improvement_fraction >= 0.0 && agg.covered_rmse_samples > 0) {
      const double optical_covered_rmse = agg.sum_optical_covered_rmse / agg.covered_rmse_samples;
      const double fused_covered_rmse = agg.sum_fused_covered_rmse / agg.covered_rmse_samples;
      const double required_max_rmse = optical_covered_rmse * (1.0 - min_fusion_improvement_fraction);
      if (fused_covered_rmse > required_max_rmse) {
        std::cerr << "GATE FAIL: " << spec.name << " fused_covered_rmse " << fused_covered_rmse
                  << " does not improve on optical_covered_rmse " << optical_covered_rmse << " by at least "
                  << (min_fusion_improvement_fraction * 100.0) << "% (required <= " << required_max_rmse << ")\n";
        any_gate_failed = true;
      }
    }
  }

  return any_gate_failed ? 1 : 0;
}
