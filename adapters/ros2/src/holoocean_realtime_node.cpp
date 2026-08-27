#include "adapters/ros2_holoocean_realtime_gateway.hpp"

#include <chrono>
#include <utility>

#include "sensor_models/geometry.hpp"

namespace uw::adapters {
namespace {

int64_t SteadyNowNs() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count < 0 ? 0 : count;
}

RawHoloImage ToRawHoloImage(const sensor_msgs::msg::Image& msg) {
  RawHoloImage raw;
  raw.capture_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1'000'000'000 + msg.header.stamp.nanosec;
  raw.width = msg.width;
  raw.height = msg.height;
  // Task 3's holoocean_camera_to_ros_image already performs the BGR(A)->RGB
  // fix and publishes rgb8 -- copy through as-is (see
  // holoocean_live_conversion.hpp's ConvertHoloImage doc comment).
  raw.rgb_pixel_data.assign(msg.data.begin(), msg.data.end());
  return raw;
}

RawHoloSonar ToRawHoloSonar(const holoocean_interfaces::msg::ImagingSonar& msg) {
  RawHoloSonar raw;
  raw.capture_ns = msg.timestamp;
  raw.num_ranges = static_cast<uint32_t>(msg.bins_range);
  raw.num_beams = static_cast<uint32_t>(msg.bins_azimuth);
  raw.image_range.assign(msg.image_range.begin(), msg.image_range.end());
  return raw;
}

RawHoloVehicleState ToRawHoloVehicleState(const nav_msgs::msg::Odometry& msg) {
  RawHoloVehicleState raw;
  raw.capture_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1'000'000'000 + msg.header.stamp.nanosec;
  raw.orientation_xyzw[0] = msg.pose.pose.orientation.x;
  raw.orientation_xyzw[1] = msg.pose.pose.orientation.y;
  raw.orientation_xyzw[2] = msg.pose.pose.orientation.z;
  raw.orientation_xyzw[3] = msg.pose.pose.orientation.w;
  raw.angular_velocity_radps[0] = msg.twist.twist.angular.x;
  raw.angular_velocity_radps[1] = msg.twist.twist.angular.y;
  raw.angular_velocity_radps[2] = msg.twist.twist.angular.z;
  raw.raw_position_z_m = msg.pose.pose.position.z;
  return raw;
}

sensor_msgs::msg::Image ToRosImage(const uw::domain::ImageFrame& frame) {
  sensor_msgs::msg::Image msg;
  msg.header.stamp.sec = static_cast<int32_t>(frame.header().capture_time().seconds());
  msg.header.stamp.nanosec = static_cast<uint32_t>(frame.header().capture_time().nanos());
  msg.header.frame_id = frame.header().sensor_frame().value();
  msg.height = frame.height();
  msg.width = frame.width();
  msg.encoding = "rgb8";
  msg.is_bigendian = 0;
  msg.step = frame.row_stride_bytes();
  msg.data.assign(frame.pixel_data().begin(), frame.pixel_data().end());
  return msg;
}

// Parameter defaults mirror Task 1's manifest
// (adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json's ImagingSonar
// sensor: Azimuth 140deg, RangeMin/Max 0.30/30.0m, WaterSpeedSound 1480) so
// an unconfigured launch already matches the BlueROV/AI-D/SV1213 profile;
// a real launch file overriding these to mirror a different manifest is
// the intended integration point (declare_parameter, same pattern
// Ros2HoloOceanSonarBridge already uses for input_topic).
HoloOceanRealtimeGatewayOptions ReadOptionsFromParameters(rclcpp::Node& node) {
  HoloOceanRealtimeGatewayOptions options;
  options.agent_name = node.declare_parameter<std::string>("agent_name", options.agent_name);
  options.calibration_version =
      node.declare_parameter<std::string>("calibration_version", options.calibration_version);
  options.rig_config_path =
      node.declare_parameter<std::string>("rig_config_path", options.rig_config_path);
  options.platform_config_path =
      node.declare_parameter<std::string>("platform_config_path", options.platform_config_path);
  auto& sonar = options.sonar_calibration;
  sonar.horizontal_fov_rad = static_cast<float>(
      node.declare_parameter<double>("sonar_horizontal_fov_rad", 2.4434609528));  // 140 deg
  sonar.min_range_m = static_cast<float>(node.declare_parameter<double>("sonar_min_range_m", 0.30));
  sonar.max_range_m = static_cast<float>(node.declare_parameter<double>("sonar_max_range_m", 30.0));
  sonar.elevation_aperture_rad = static_cast<float>(
      node.declare_parameter<double>("sonar_elevation_aperture_rad", 0.34906585));  // 20 deg
  sonar.operating_frequency_hz =
      node.declare_parameter<double>("sonar_operating_frequency_hz", 1'200'000.0);
  sonar.sound_speed_mps = static_cast<float>(node.declare_parameter<double>("sonar_sound_speed_mps", 1480.0));
  sonar.salinity_ppt = static_cast<float>(node.declare_parameter<double>("sonar_salinity_ppt", 35.0));
  sonar.sound_speed_is_measured = node.declare_parameter<bool>("sonar_sound_speed_is_measured", false);
  sonar.sensor_id = node.declare_parameter<std::string>("sonar_sensor_id", sonar.sensor_id);
  sonar.sensor_frame = node.declare_parameter<std::string>("sonar_sensor_frame", sonar.sensor_frame);
  sonar.calibration_version = options.calibration_version;
  return options;
}

}  // namespace

uw::domain::RigCalibrationSnapshot BuildIdentityRig(const HoloOceanRealtimeGatewayOptions& options) {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value(options.calibration_version);
  const std::string left_id = "camera_left";
  const std::string right_id = "camera_right";
  const std::string sonar_id = options.sonar_calibration.sensor_id;
  const std::string state_id = "rov-state";
  auto add_identity_edge = [&](const std::string& child) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    *edge->mutable_transform() = uw::sensor_models::Pose3::Identity().ToProto();
  };
  for (const std::string& sensor : {left_id, right_id}) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(sensor);
    camera->set_width(1280);
    camera->set_height(720);
    for (double v : {900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0}) {
      camera->add_k_matrix_row_major(v);
    }
    add_identity_edge(sensor + "_link");
  }
  add_identity_edge(sonar_id + "_link");
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value(sonar_id);
  sonar->set_sonar_enabled(true);
  rig.add_vehicle_state_sensors()->set_value(state_id);
  for (const std::string& sensor : {left_id, right_id, sonar_id, state_id}) {
    (*rig.mutable_time_offset_seconds())[sensor] = 0.0;
    (*rig.mutable_time_offset_provenance())[sensor] = "assumed:holoocean_realtime_node";
  }
  return rig;
}

HoloOceanRealtimeGatewayNode::HoloOceanRealtimeGatewayNode()
    : rclcpp::Node("uw_holoocean_realtime_gateway") {
  options_ = ReadOptionsFromParameters(*this);
  overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("/uw/hmi/overlay", rclcpp::QoS(1));
  status_pub_ = create_publisher<std_msgs::msg::String>("/uw/hmi/status", rclcpp::QoS(1));

  sink_ = MakeOnlineAssistRealtimeSink(
      *this, HoloOceanRealtimeSinkConfig{options_.rig_config_path, BuildIdentityRig(options_),
                                         options_.platform_config_path});

  const auto qos = rclcpp::SensorDataQoS();
  const std::string prefix = "/holoocean/" + options_.agent_name + "/";
  // Exactly the four algorithm-input topics + the independent PilotCamera
  // presentation topic -- never /uw/sim/ground_truth, and never
  // /uw/pilot/thrusters (Task 3's realtime_ros_session.py owns that).
  left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      prefix + "LeftCamera", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg) { OnLeftCamera(msg); });
  right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      prefix + "RightCamera", qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { OnRightCamera(msg); });
  pilot_sub_ = create_subscription<sensor_msgs::msg::Image>(
      prefix + "PilotCamera", qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { OnPilotCamera(msg); });
  sonar_sub_ = create_subscription<holoocean_interfaces::msg::ImagingSonar>(
      prefix + "ImagingSonar", qos,
      [this](const holoocean_interfaces::msg::ImagingSonar::SharedPtr msg) { OnSonar(msg); });
  state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      prefix + "VehicleState", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OnVehicleState(msg); });
}

int64_t HoloOceanRealtimeGatewayNode::NowMonotonicNs() const { return SteadyNowNs(); }

void HoloOceanRealtimeGatewayNode::PublishOverlay(uw::domain::ImageFrame frame) {
  overlay_pub_->publish(ToRosImage(frame));
}

void HoloOceanRealtimeGatewayNode::PublishStatus(std::string json_status) {
  std_msgs::msg::String msg;
  msg.data = std::move(json_status);
  status_pub_->publish(msg);
}

void HoloOceanRealtimeGatewayNode::OnLeftCamera(const sensor_msgs::msg::Image::SharedPtr msg) {
  sink_->OnLeftCamera(ConvertHoloImage(ToRawHoloImage(*msg), "camera_left", "camera_left_link",
                                       ++left_sequence_, options_.calibration_version, NowMonotonicNs()));
}

void HoloOceanRealtimeGatewayNode::OnRightCamera(const sensor_msgs::msg::Image::SharedPtr msg) {
  sink_->OnRightCamera(ConvertHoloImage(ToRawHoloImage(*msg), "camera_right", "camera_right_link",
                                        ++right_sequence_, options_.calibration_version, NowMonotonicNs()));
}

void HoloOceanRealtimeGatewayNode::OnPilotCamera(const sensor_msgs::msg::Image::SharedPtr msg) {
  sink_->OnPilotCamera(ConvertHoloImage(ToRawHoloImage(*msg), "camera_pilot", "camera_pilot_link",
                                        ++pilot_sequence_, options_.calibration_version, NowMonotonicNs()));
}

void HoloOceanRealtimeGatewayNode::OnSonar(const holoocean_interfaces::msg::ImagingSonar::SharedPtr msg) {
  sink_->OnSonar(ConvertHoloSonar(ToRawHoloSonar(*msg), options_.sonar_calibration, ++sonar_sequence_,
                                  NowMonotonicNs()));
}

void HoloOceanRealtimeGatewayNode::OnVehicleState(const nav_msgs::msg::Odometry::SharedPtr msg) {
  sink_->OnVehicleState(ConvertHoloVehicleState(ToRawHoloVehicleState(*msg), "rov-state", "state_link",
                                                ++state_sequence_, options_.calibration_version,
                                                NowMonotonicNs()));
}

}  // namespace uw::adapters

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uw::adapters::HoloOceanRealtimeGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
