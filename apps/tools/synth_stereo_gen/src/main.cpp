// Generates a single synthetic stereo frame pair + a ground-truth depth
// grid for optical-baseline evaluation (plan 2 of the acoustic-optic
// series). Deliberately independent of apps/tools/synth_bag_gen: this is a
// single static-scene generator for the optical baseline, not a
// trajectory/pose-graph bag — see that tool's own header for why THIS repo
// needs synthetic generators at all (no HoloOcean/ROS2 on this dev
// machine).
//
// Topics written:
//   /raw/camera/left    uw.domain.ImageFrame
//   /raw/camera/right   uw.domain.ImageFrame
//   /gt/depth            uw.domain.MeasurementEvidence (OpticalDepthPriorMeasurement,
//                         producer_type="ground_truth" — reuses plan 1's metric depth-grid
//                         contract instead of inventing a parallel GT schema; ground truth is
//                         exactly a perfect metric depth prior)
#include <cstdint>
#include <iostream>
#include <string>

#include "uw/domain/domain.hpp"
#include "uw/runtime/mcap_io.hpp"

namespace {

uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 131 + v * 67 + 19) % 256); }

uw::domain::ImageFrame MakeImage(const std::string& frame, uint32_t width, uint32_t height, int shift) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  image.mutable_header()->set_provenance("synth_stereo_gen_v1");
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      pixels[static_cast<std::size_t>(v) * width + u] =
          static_cast<char>(Texture(static_cast<int>(u) + shift, static_cast<int>(v)));
    }
  }
  image.set_pixel_data(pixels);
  image.set_is_rectified(true);
  return image;
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_path = "/tmp/synthetic_stereo.mcap";
  uint32_t width = 640;
  uint32_t height = 480;
  int true_disparity_px = 8;
  double depth_m = 6.3;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--out" && i + 1 < argc) {
      out_path = next();
    } else if (arg == "--width" && i + 1 < argc) {
      width = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--height" && i + 1 < argc) {
      height = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--disparity-px" && i + 1 < argc) {
      true_disparity_px = std::stoi(next());
    } else if (arg == "--depth-m" && i + 1 < argc) {
      depth_m = std::stod(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(out_path)) {
    std::cerr << "failed to open " << out_path << " for writing\n";
    return 1;
  }

  const auto left = MakeImage("camera_left_link", width, height, 0);
  const auto right = MakeImage("camera_right_link", width, height, true_disparity_px);
  writer.WriteMessage("/raw/camera/left", 0, left);
  writer.WriteMessage("/raw/camera/right", 0, right);

  uw::domain::OpticalDepthPriorMeasurement gt;
  *gt.mutable_reference_camera_frame() = left.header().sensor_frame();
  gt.set_width(width);
  gt.set_height(height);
  gt.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  gt.set_producer_type("ground_truth");
  std::string valid_mask(static_cast<std::size_t>(width) * height, 1);
  for (uint32_t i = 0; i < width * height; ++i) {
    gt.add_depth_m(static_cast<float>(depth_m));
    gt.add_variance_m2(1e-6f);
  }
  gt.set_valid_mask(valid_mask);

  uw::domain::EvidenceId gt_id;
  gt_id.set_value("gt_depth_0");
  auto gt_evidence = uw::domain::MakeEvidence(gt_id, {}, gt, 0.0, "synth_stereo_gen_v1");
  writer.WriteMessage("/gt/depth", 0, gt_evidence);

  writer.Close();
  std::cout << "wrote " << width << "x" << height << " stereo pair (disparity=" << true_disparity_px
            << "px, depth=" << depth_m << "m) to " << out_path << "\n";
  return 0;
}
