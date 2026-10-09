#include "dlrobot_robot/orientation_filter.hpp"
#include "dlrobot_robot/protocol.hpp"

#include <serial/serial.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace dlrobot_robot
{
namespace
{

constexpr std::array<double, 36> kMovingPoseCovariance = {
  1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 1e3};

constexpr std::array<double, 36> kStoppedPoseCovariance = {
  1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
  0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 1e-9};

constexpr std::array<double, 36> kMovingTwistCovariance = {
  1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 1e3};

constexpr std::array<double, 36> kStoppedTwistCovariance = {
  1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
  0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 1e-9};

bool IsStopped(const Telemetry & telemetry)
{
  return telemetry.velocity_x == 0.0F &&
         telemetry.velocity_y == 0.0F &&
         telemetry.velocity_z == 0.0F;
}

}  // namespace

class DlrobotRobotNode final : public rclcpp::Node
{
public:
  DlrobotRobotNode()
  : Node("dlrobot_robot"),
    serial_port_name_(declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0")),
    serial_baud_rate_(declare_parameter<int>("serial_baud_rate", 115200)),
    odom_frame_id_(declare_parameter<std::string>("odom_frame_id", "odom_combined")),
    robot_frame_id_(declare_parameter<std::string>("robot_frame_id", "base_footprint")),
    gyro_frame_id_(declare_parameter<std::string>("gyro_frame_id", "imu_link"))
  {
    voltage_publisher_ = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", 20);
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 100,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        SendCommand(message->twist);
      });

    OpenSerialPort();
    last_telemetry_time_ = now();
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      [this]() {
        ReadSerialPort();
      });

    RCLCPP_INFO(
      get_logger(), "Serial port %s opened at %d baud",
      serial_port_name_.c_str(), serial_baud_rate_);
  }

  ~DlrobotRobotNode() override
  {
    if (!serial_port_.isOpen()) {
      return;
    }

    try {
      const auto stop_frame = EncodeVelocityCommand(0.0, 0.0, 0.0);
      serial_port_.write(stop_frame.data(), stop_frame.size());
      serial_port_.close();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to stop and close serial port: %s", error.what());
    }
  }

private:
  void OpenSerialPort()
  {
    if (serial_baud_rate_ <= 0) {
      throw std::invalid_argument("serial_baud_rate must be positive");
    }

    serial_port_.setPort(serial_port_name_);
    serial_port_.setBaudrate(static_cast<std::uint32_t>(serial_baud_rate_));
    auto timeout = serial::Timeout::simpleTimeout(2000);
    serial_port_.setTimeout(timeout);
    serial_port_.open();
    if (!serial_port_.isOpen()) {
      throw std::runtime_error("serial port did not open: " + serial_port_name_);
    }
  }

  void SendCommand(const geometry_msgs::msg::Twist & command)
  {
    try {
      const auto frame = EncodeVelocityCommand(
        command.linear.x, command.linear.y, command.angular.z);
      const auto written = serial_port_.write(frame.data(), frame.size());
      if (written != frame.size()) {
        throw std::runtime_error("incomplete serial command write");
      }
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "Serial command failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  void ReadSerialPort()
  {
    try {
      std::array<std::uint8_t, 256> bytes{};
      while (true) {
        const auto available = serial_port_.available();
        if (available == 0) {
          break;
        }
        const auto count = std::min(available, bytes.size());
        const auto read_count = serial_port_.read(bytes.data(), count);
        for (std::size_t index = 0; index < read_count; ++index) {
          const auto telemetry = parser_.Push(bytes[index]);
          if (telemetry.has_value()) {
            PublishTelemetry(*telemetry);
          }
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "Serial receive failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  void PublishTelemetry(const Telemetry & telemetry)
  {
    const auto stamp = now();
    const double sampling_time = (stamp - last_telemetry_time_).seconds();
    last_telemetry_time_ = stamp;

    position_x_ +=
      (telemetry.velocity_x * std::cos(position_yaw_) -
      telemetry.velocity_y * std::sin(position_yaw_)) * sampling_time;
    position_y_ +=
      (telemetry.velocity_x * std::sin(position_yaw_) +
      telemetry.velocity_y * std::cos(position_yaw_)) * sampling_time;
    position_yaw_ += telemetry.velocity_z * sampling_time;

    const auto orientation = orientation_filter_.Update(
      telemetry.angular_velocity_x,
      telemetry.angular_velocity_y,
      telemetry.angular_velocity_z,
      telemetry.acceleration_x,
      telemetry.acceleration_y,
      telemetry.acceleration_z);

    PublishOdometry(telemetry, stamp);
    PublishImu(telemetry, orientation, stamp);
    PublishVoltage(telemetry.voltage);
  }

  void PublishOdometry(const Telemetry & telemetry, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = robot_frame_id_;
    odometry.pose.pose.position.x = position_x_;
    odometry.pose.pose.position.y = position_y_;
    odometry.pose.pose.orientation.z = std::sin(position_yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(position_yaw_ * 0.5);
    odometry.twist.twist.linear.x = telemetry.velocity_x;
    odometry.twist.twist.linear.y = telemetry.velocity_y;
    odometry.twist.twist.angular.z = telemetry.velocity_z;

    if (IsStopped(telemetry)) {
      odometry.pose.covariance = kStoppedPoseCovariance;
      odometry.twist.covariance = kStoppedTwistCovariance;
    } else {
      odometry.pose.covariance = kMovingPoseCovariance;
      odometry.twist.covariance = kMovingTwistCovariance;
    }
    odom_publisher_->publish(odometry);
  }

  void PublishImu(
    const Telemetry & telemetry,
    const Quaternion & orientation,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = gyro_frame_id_;
    imu.orientation.x = orientation.x;
    imu.orientation.y = orientation.y;
    imu.orientation.z = orientation.z;
    imu.orientation.w = orientation.w;
    imu.orientation_covariance[0] = 1e6;
    imu.orientation_covariance[4] = 1e6;
    imu.orientation_covariance[8] = 1e-6;
    imu.angular_velocity.x = telemetry.angular_velocity_x;
    imu.angular_velocity.y = telemetry.angular_velocity_y;
    imu.angular_velocity.z = telemetry.angular_velocity_z;
    imu.angular_velocity_covariance[0] = 1e6;
    imu.angular_velocity_covariance[4] = 1e6;
    imu.angular_velocity_covariance[8] = 1e-6;
    imu.linear_acceleration.x = telemetry.acceleration_x;
    imu.linear_acceleration.y = telemetry.acceleration_y;
    imu.linear_acceleration.z = telemetry.acceleration_z;
    imu_publisher_->publish(imu);
  }

  void PublishVoltage(float voltage)
  {
    ++voltage_publish_counter_;
    if (voltage_publish_counter_ < 12) {
      return;
    }

    voltage_publish_counter_ = 0;
    std_msgs::msg::Float32 message;
    message.data = voltage;
    voltage_publisher_->publish(message);
  }

  serial::Serial serial_port_;
  TelemetryParser parser_;
  OrientationFilter orientation_filter_;

  const std::string serial_port_name_;
  const int serial_baud_rate_;
  const std::string odom_frame_id_;
  const std::string robot_frame_id_;
  const std::string gyro_frame_id_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr serial_timer_;

  rclcpp::Time last_telemetry_time_;
  double position_x_{0.0};
  double position_y_{0.0};
  double position_yaw_{0.0};
  std::size_t voltage_publish_counter_{0};
};

}  // namespace dlrobot_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dlrobot_robot::DlrobotRobotNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dlrobot_robot"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
