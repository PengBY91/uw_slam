#include "operator_overlay_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace uw::opencv_adapters {

namespace {

constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
constexpr double kFontScale = 0.45;
constexpr int kFontThickness = 1;
constexpr int kLineHeightPx = 18;
constexpr int kBannerHeightPx = 24;
constexpr int kSonarPanelMaxSidePx = 320;

const cv::Scalar kColorAcousticOptic(0, 200, 0);    // green
const cv::Scalar kColorVisual(0, 220, 220);         // cyan (R,G,B channel order)
const cv::Scalar kColorSonar(255, 176, 0);          // amber
const cv::Scalar kColorStale(220, 30, 30);          // red
const cv::Scalar kColorUnknown(160, 160, 160);       // gray
const cv::Scalar kColorText(255, 255, 255);
const cv::Scalar kColorBannerSuspect(200, 150, 0);
const cv::Scalar kColorBannerUnavailable(200, 30, 30);
const cv::Scalar kColorBannerRecovering(0, 140, 220);
const cv::Scalar kColorBackground(10, 10, 30);

bool IsSupportedColorEncoding(uw::domain::ImageFrame::ImageEncoding encoding) {
  return encoding == uw::domain::ImageFrame::IMAGE_ENCODING_RGB8 ||
         encoding == uw::domain::ImageFrame::IMAGE_ENCODING_BGR8;
}

// track_id/class_label duplicate the constant from target_tracker.cpp's
// GenericClass -- a placeholder label the frontends emit when they have no
// real class model yet (see frontends/target_tracker.cpp). Kept local
// rather than exposing frontends internals to this application-agnostic
// adapter.
bool GenericClass(const std::string& label) {
  return label == "target" || label == "sonar_target";
}

std::string FormatFixed(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

struct SourceStyle {
  std::string word;
  cv::Scalar color;
};

SourceStyle StyleForTrack(const uw::domain::TargetTrack& track) {
  const bool has_visual = std::find(track.sources().begin(), track.sources().end(),
                                    uw::domain::ASSIST_SOURCE_VISUAL) != track.sources().end();
  const bool has_sonar = std::find(track.sources().begin(), track.sources().end(),
                                   uw::domain::ASSIST_SOURCE_SONAR) != track.sources().end();
  SourceStyle style;
  if (has_visual && has_sonar) {
    style = {"ACOUSTIC_OPTIC", kColorAcousticOptic};
  } else if (has_visual) {
    style = {"VISUAL", kColorVisual};
  } else if (has_sonar) {
    style = {"SONAR", kColorSonar};
  } else {
    style = {"UNKNOWN", kColorUnknown};
  }
  if (track.status() == uw::domain::TARGET_TRACK_STATUS_STALE ||
      track.status() == uw::domain::TARGET_TRACK_STATUS_DEGRADED) {
    style.color = kColorStale;
  }
  return style;
}

std::string LabelForTrack(const uw::domain::TargetTrack& track, const SourceStyle& style) {
  std::vector<std::string> tokens;
  tokens.push_back(track.track_id().value());
  tokens.push_back(style.word);
  if (!GenericClass(track.class_label())) tokens.push_back(track.class_label());
  if (track.has_range_m()) tokens.push_back(FormatFixed(track.range_m(), 1) + "m");
  tokens.push_back("c" + FormatFixed(track.class_confidence(), 2));
  const double age_ms = std::max(
      0.0, (uw::domain::ToSeconds(track.publish_time()) - uw::domain::ToSeconds(track.last_capture_time())) *
               1000.0);
  tokens.push_back(std::to_string(static_cast<long long>(std::llround(age_ms))) + "ms");

  std::string label;
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) label += " ";
    label += tokens[i];
  }
  return label;
}

bool WellFormedSonarFrame(const uw::domain::SonarFrame& frame) {
  const auto ranges = frame.num_ranges();
  const auto beams = frame.num_beams();
  // IsAzimuthAscending both checks finiteness and ascending order (see
  // src/domain/domain.cpp) -- RenderSonarPanel's std::lower_bound lookup
  // below assumes ascending azimuth_angles, the exact precondition that
  // helper exists to guard.
  return ranges > 0 && beams > 0 &&
        frame.intensity_tensor().size() == static_cast<std::size_t>(ranges) * beams &&
        frame.azimuth_angles_size() == static_cast<int>(beams) &&
        uw::domain::IsAzimuthAscending(frame) &&
        std::isfinite(frame.range_resolution()) && frame.range_resolution() > 0.0f &&
        std::isfinite(frame.min_range());
}

// Renders the sonar frame as a forward-looking polar fan (apex at the
// bottom-center, bearing 0 pointing up) into a square panel. Every output
// pixel is mapped back to (range, bearing) and looked up in the raw
// row-major [range, beam] intensity tensor -- geometrically real, not a
// raw reshape of the tensor.
cv::Mat RenderSonarPanel(const uw::domain::SonarFrame& frame, int side_px) {
  cv::Mat panel(side_px, side_px, CV_8UC3, kColorBackground);
  if (!WellFormedSonarFrame(frame)) return panel;

  const int num_ranges = static_cast<int>(frame.num_ranges());
  const int num_beams = static_cast<int>(frame.num_beams());
  const double min_range = frame.min_range();
  const double max_range =
      std::max(static_cast<double>(min_range) + 1e-6,
               min_range + static_cast<double>(frame.range_resolution()) * num_ranges);
  const double min_bearing = frame.azimuth_angles(0);
  const double max_bearing = frame.azimuth_angles(num_beams - 1);
  const auto& pixels = frame.intensity_tensor();

  const double apex_x = side_px / 2.0;
  const double apex_y = side_px - 1.0;
  const double range_scale = max_range / static_cast<double>(side_px - 1);

  for (int py = 0; py < side_px; ++py) {
    for (int px = 0; px < side_px; ++px) {
      const double dx = px - apex_x;
      const double dy = apex_y - py;
      if (dy < 0.0) continue;
      const double range = std::sqrt(dx * dx + dy * dy) * range_scale;
      const double bearing = std::atan2(dx, dy);
      if (range < min_range || range > max_range || bearing < min_bearing || bearing > max_bearing) {
        continue;
      }
      int range_idx = static_cast<int>((range - min_range) / frame.range_resolution());
      range_idx = std::clamp(range_idx, 0, num_ranges - 1);
      const auto beam_it = std::lower_bound(frame.azimuth_angles().begin(),
                                            frame.azimuth_angles().end(), bearing);
      int beam_idx = static_cast<int>(beam_it - frame.azimuth_angles().begin());
      beam_idx = std::clamp(beam_idx, 0, num_beams - 1);
      const std::size_t tensor_index = static_cast<std::size_t>(range_idx) * num_beams + beam_idx;
      const auto intensity = static_cast<uint8_t>(pixels[tensor_index]);
      panel.at<cv::Vec3b>(py, px) = cv::Vec3b(intensity, intensity, intensity);
    }
  }
  cv::putText(panel, "SONAR", cv::Point(6, 16), kFont, kFontScale, kColorSonar, kFontThickness,
             cv::LINE_AA);
  return panel;
}

cv::Mat ColorView(const uw::domain::ImageFrame& image) {
  return cv::Mat(static_cast<int>(image.height()), static_cast<int>(image.width()), CV_8UC3,
                const_cast<char*>(image.pixel_data().data()), image.row_stride_bytes());
}

}  // namespace

std::optional<uw::domain::ImageFrame> OperatorOverlayRenderer::Render(
    const uw::domain::ImageFrame& pilot_rgb, const std::optional<uw::domain::SonarFrame>& latest_sonar,
    const uw::domain::OperatorAssistState& state) {
  last_labels_.clear();

  // ValidateImageFrame (src/domain/domain.cpp) checks dimensions, stride
  // vs. width*bytes-per-pixel, and exact payload size -- reused here
  // rather than re-deriving those bounds, so ColorView's cv::Mat below
  // never gets a too-small stride or buffer to read past the end of.
  if (!uw::domain::ValidateImageFrame(pilot_rgb).ok() || !IsSupportedColorEncoding(pilot_rgb.encoding()) ||
      pilot_rgb.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      pilot_rgb.height() > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }

  cv::Mat canvas = ColorView(pilot_rgb).clone();
  if (pilot_rgb.encoding() == uw::domain::ImageFrame::IMAGE_ENCODING_BGR8) {
    cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
  }

  // Track labels and cues.
  int line_y = kBannerHeightPx + kLineHeightPx;
  for (const auto& track : state.target_tracks().tracks()) {
    const auto style = StyleForTrack(track);
    const std::string label = LabelForTrack(track, style);
    last_labels_.push_back(label);
    cv::putText(canvas, label, cv::Point(6, line_y), kFont, kFontScale, style.color, kFontThickness,
               cv::LINE_AA);
    // Bearing cue: a short tick along the top edge, linearly mapping a
    // +/-60 degree forward field of view across the canvas width.
    constexpr double kAssumedHalfFovRad = 1.0472;  // 60 degrees
    const double clamped_bearing =
        std::clamp(track.bearing_rad(), -kAssumedHalfFovRad, kAssumedHalfFovRad);
    const double u = 0.5 + clamped_bearing / (2.0 * kAssumedHalfFovRad);
    const int tick_x = std::clamp(static_cast<int>(u * canvas.cols), 0, canvas.cols - 1);
    cv::line(canvas, cv::Point(tick_x, 0), cv::Point(tick_x, 8), style.color, 2, cv::LINE_AA);
    line_y += kLineHeightPx;
  }

  // Health banner.
  const auto& health = state.system_health();
  if (health.status() != uw::domain::HealthReport::STATUS_HEALTHY &&
      health.status() != uw::domain::HealthReport::STATUS_UNSPECIFIED) {
    std::string word;
    cv::Scalar color = kColorBannerSuspect;
    switch (health.status()) {
      case uw::domain::HealthReport::STATUS_SUSPECT:
        word = "DEGRADED";
        color = kColorBannerSuspect;
        break;
      case uw::domain::HealthReport::STATUS_UNAVAILABLE:
        word = "UNAVAILABLE";
        color = kColorBannerUnavailable;
        break;
      case uw::domain::HealthReport::STATUS_RECOVERING:
        word = "RECOVERING";
        color = kColorBannerRecovering;
        break;
      default:
        word = "DEGRADED";
        break;
    }
    std::string banner_label = word;
    if (!health.reason_code().empty()) banner_label += " " + health.reason_code();
    last_labels_.push_back(banner_label);

    cv::rectangle(canvas, cv::Point(0, 0), cv::Point(canvas.cols - 1, kBannerHeightPx - 1), color,
                 cv::FILLED);
    cv::putText(canvas, banner_label, cv::Point(6, kBannerHeightPx - 6), kFont, kFontScale, kColorText,
               kFontThickness, cv::LINE_AA);
  }

  // Path-offset arrow, bottom-center.
  if (state.has_path_lateral_offset()) {
    const int cx = canvas.cols / 2;
    const int base_y = canvas.rows - 12;
    const double magnitude_px =
        std::clamp(state.path_lateral_offset_m() * 40.0, -80.0, 80.0);
    const cv::Point tip(std::clamp(cx + static_cast<int>(magnitude_px), 0, canvas.cols - 1), base_y);
    cv::arrowedLine(canvas, cv::Point(cx, base_y), tip, kColorText, 2, cv::LINE_AA, 0, 0.3);
  }

  // Sonar side panel, appended to the right at a size matched to the main
  // panel's height (capped, since a raw sensor image can be much taller
  // than a sensible overlay).
  if (latest_sonar.has_value() && WellFormedSonarFrame(*latest_sonar)) {
    const int side_px = std::min(canvas.rows, kSonarPanelMaxSidePx);
    cv::Mat panel = RenderSonarPanel(*latest_sonar, side_px);
    if (panel.rows != canvas.rows) {
      cv::Mat resized;
      cv::resize(panel, resized, cv::Size(panel.cols, canvas.rows));
      panel = resized;
    }
    cv::Mat combined;
    cv::hconcat(canvas, panel, combined);
    canvas = combined;
  }

  uw::domain::ImageFrame out = pilot_rgb;
  out.set_width(static_cast<uint32_t>(canvas.cols));
  out.set_height(static_cast<uint32_t>(canvas.rows));
  out.set_row_stride_bytes(static_cast<uint32_t>(canvas.step[0]));
  out.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  out.set_pixel_data(std::string(reinterpret_cast<const char*>(canvas.data),
                                 canvas.step[0] * static_cast<std::size_t>(canvas.rows)));
  return out;
}

}  // namespace uw::opencv_adapters
