#include "adapters/holoocean_live_conversion.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

using uw::adapters::ConvertHoloImage;
using uw::adapters::ConvertHoloSonar;
using uw::adapters::ConvertHoloVehicleState;
using uw::adapters::HoloOceanSonarCalibration;
using uw::adapters::RawHoloImage;
using uw::adapters::RawHoloSonar;
using uw::adapters::RawHoloVehicleState;

namespace {

RawHoloImage MakeRawHoloImage(int64_t capture_ns, uint32_t width, uint32_t height) {
  RawHoloImage raw;
  raw.capture_ns = capture_ns;
  raw.width = width;
  raw.height = height;
  raw.rgb_pixel_data.assign(static_cast<std::size_t>(width) * height * 3, 42);
  return raw;
}

// num_beams matches adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json's
// ImagingSonar AzimuthBins (768) exactly -- num_beams/num_ranges are a
// property of the actual received frame, not the calibration struct, so
// TestSonarCalibration30m below only supplies range/FOV/sound-speed
// calibration, not dimensions.
RawHoloSonar MakeRawHoloSonar() {
  RawHoloSonar raw;
  raw.capture_ns = 3'000'000'000;
  raw.num_ranges = 4;
  raw.num_beams = 768;
  raw.image_range.assign(static_cast<std::size_t>(raw.num_ranges) * raw.num_beams, 0.2f);
  raw.image_range[2 * raw.num_beams + (raw.num_beams - 1)] = 0.9f;
  return raw;
}

// Matches adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json's
// ImagingSonar sensor exactly (RangeMax 30.0, AzimuthBins 768, WaterSpeedSound
// 1480, uw_metadata.sonar_model.operating_frequency_hz 1200000.0) -- proving
// this comes from a manifest-shaped calibration struct, not a hardcoded
// default the way record_session.py's _DEFAULT_SONAR_* constants are.
HoloOceanSonarCalibration TestSonarCalibration30m() {
  HoloOceanSonarCalibration calibration;
  calibration.horizontal_fov_rad = 2.4434609528f;  // 140 deg
  calibration.min_range_m = 0.30f;
  calibration.max_range_m = 30.0f;
  calibration.elevation_aperture_rad = 0.34906585f;  // 20 deg
  calibration.operating_frequency_hz = 1'200'000.0;
  calibration.sound_speed_mps = 1480.0f;
  calibration.salinity_ppt = 35.0f;
  calibration.sound_speed_is_measured = false;
  calibration.gain_db = 0.0f;
  calibration.gain_mode = 0;
  return calibration;
}

}  // namespace

TEST(HoloOceanLiveConversion, PopulatesSequenceCalibrationAndReceiveClock) {
  RawHoloImage raw = MakeRawHoloImage(/*capture_ns=*/1'000'000'000, 1280, 720);
  const auto frame = ConvertHoloImage(raw, "camera_left", "camera_left_link", 7, "aid_sim_v1",
                                       /*receive_monotonic_ns=*/2'000'000'000);
  EXPECT_EQ(frame.header().sequence_id().value(), 7u);
  EXPECT_EQ(frame.header().calibration_version().value(), "aid_sim_v1");
  EXPECT_EQ(frame.header().receive_clock_domain(), uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  EXPECT_EQ(frame.header().clock_domain(), uw::domain::CLOCK_DOMAIN_SIMULATION);
  EXPECT_EQ(frame.header().sensor_id().value(), "camera_left");
  EXPECT_EQ(frame.header().sensor_frame().value(), "camera_left_link");
  EXPECT_EQ(frame.header().capture_time().seconds(), 1);
  EXPECT_EQ(frame.header().receive_time().seconds(), 2);
  EXPECT_EQ(frame.width(), 1280u);
  EXPECT_EQ(frame.height(), 720u);
  EXPECT_EQ(frame.row_stride_bytes(), 1280u * 3u);
  EXPECT_EQ(frame.encoding(), uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  EXPECT_EQ(frame.pixel_data().size(), 1280u * 720u * 3u);
}

TEST(HoloOceanLiveConversion, SonarUsesManifestCalibrationNotHardcodedDefaults) {
  const auto frame = ConvertHoloSonar(MakeRawHoloSonar(), TestSonarCalibration30m(), 4, 9);
  EXPECT_FLOAT_EQ(frame.max_range(), 30.0f);
  EXPECT_EQ(frame.num_beams(), 768u);
  EXPECT_FLOAT_EQ(frame.sound_speed_assumption().speed_of_sound_mps(), 1480.0f);
  EXPECT_DOUBLE_EQ(frame.operating_frequency_hz(), 1'200'000.0);
}

TEST(HoloOceanLiveConversion, SonarPopulatesHeaderAndFiniteGeometryFields) {
  const auto frame = ConvertHoloSonar(MakeRawHoloSonar(), TestSonarCalibration30m(),
                                       /*sequence_id=*/4, /*receive_monotonic_ns=*/9);
  EXPECT_EQ(frame.header().sequence_id().value(), 4u);
  EXPECT_EQ(frame.header().receive_clock_domain(), uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  EXPECT_EQ(frame.header().clock_domain(), uw::domain::CLOCK_DOMAIN_SIMULATION);
  EXPECT_EQ(frame.header().validity(), uw::domain::ObservationHeader::VALIDITY_OK);
  EXPECT_EQ(frame.num_ranges(), 4u);
  EXPECT_TRUE(uw::domain::IsAzimuthAscending(frame));
  EXPECT_TRUE(std::isfinite(frame.elevation_aperture()));
  EXPECT_TRUE(std::isfinite(frame.horizontal_fov()));
  EXPECT_TRUE(std::isfinite(static_cast<float>(frame.operating_frequency_hz())));
}

TEST(HoloOceanLiveConversion, SonarMirrorsBeamOrderLikeTheExistingRosBridgeProvider) {
  const auto frame = ConvertHoloSonar(MakeRawHoloSonar(), TestSonarCalibration30m(), 1, 0);
  ASSERT_EQ(frame.intensity_tensor().size(), 4u * 768u);
  // Row 2's bright cell was at the last (pre-mirror) column -- the mirror
  // flip must land it in the first column of that row.
  const std::string& bytes = frame.intensity_tensor();
  const std::size_t row2_first = 2 * 768;
  EXPECT_GT(static_cast<unsigned char>(bytes[row2_first]), 200);
}

TEST(HoloOceanLiveConversion, VehicleStateNegatesRawZIntoPositiveDownDepth) {
  RawHoloVehicleState raw;
  raw.capture_ns = 5'000'000'000;
  raw.orientation_xyzw[3] = 1.0;
  raw.raw_position_z_m = -2.0;  // HoloOcean's raw world-frame z, underwater

  const auto state =
      ConvertHoloVehicleState(raw, "rov-state", "state_link", 3, "aid_sim_v1", 0);
  EXPECT_DOUBLE_EQ(state.depth_m(), 2.0);
  EXPECT_TRUE(state.attitude_valid());
  EXPECT_TRUE(state.depth_valid());
  EXPECT_FALSE(state.device_health_valid());
  ASSERT_EQ(state.orientation_xyzw_size(), 4);
  EXPECT_DOUBLE_EQ(state.orientation_xyzw(3), 1.0);
  ASSERT_EQ(state.angular_velocity_radps_size(), 3);
}
