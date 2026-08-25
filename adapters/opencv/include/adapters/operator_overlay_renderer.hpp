// OpenCV is isolated behind this repo-native interface. This public header
// intentionally exposes no cv:: types or OpenCV headers.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "domain/domain.hpp"

namespace uw::opencv_adapters {

// Renders a headless operator-assistance overlay: the pilot RGB frame as
// the main panel, a labeled polar sonar side panel when a sonar frame is
// available and well-formed, per-track labels/cues, a top health banner
// when the assist state is not fully healthy, and a path-offset arrow.
// Never calls imshow/waitKey and never publishes anywhere -- this only
// produces a new RGB8 ImageFrame; ROS2 publishing and the on-screen
// display belong to the HoloOcean online-closed-loop plan. A malformed
// sonar frame degrades gracefully (panel just omitted) rather than
// failing the whole render -- matches this task's "non-blocking" framing.
class OperatorOverlayRenderer {
 public:
  std::optional<uw::domain::ImageFrame> Render(
      const uw::domain::ImageFrame& pilot_rgb,
      const std::optional<uw::domain::SonarFrame>& latest_sonar,
      const uw::domain::OperatorAssistState& state);

  // Read-only record of the exact per-track and health-banner label
  // strings drawn by the last Render() call -- track labels first, in
  // target_tracks() order, then the health banner if the state was not
  // fully healthy. Lets a headless test check operator-visible semantics,
  // not just that some pixels changed.
  const std::vector<std::string>& LastLabelsForTest() const { return last_labels_; }

 private:
  std::vector<std::string> last_labels_;
};

}  // namespace uw::opencv_adapters
