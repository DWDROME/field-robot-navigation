#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}
}  // namespace

class PathFollowerGeneric final : public rclcpp::Node
{
public:
  PathFollowerGeneric()
  : Node("path_follower_generic")
  {
    declare_parameters();
    load_parameters();
    validate_parameters();

    const auto reliable_depth_10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto reliable_depth_5 = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, reliable_depth_10,
      std::bind(&PathFollowerGeneric::odom_handler, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, reliable_depth_5,
      std::bind(&PathFollowerGeneric::path_handler, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, sensor_qos,
      std::bind(&PathFollowerGeneric::joystick_handler, this, std::placeholders::_1));
    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_, reliable_depth_5,
      std::bind(&PathFollowerGeneric::speed_handler, this, std::placeholders::_1));
    stop_sub_ = create_subscription<std_msgs::msg::Int8>(
      stop_topic_, reliable_depth_5,
      std::bind(&PathFollowerGeneric::stop_handler, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    if (autonomy_mode_) {
      joy_speed_ = std::clamp(autonomy_speed_ / max_speed_, 0.0, 1.0);
    }

    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathFollowerGeneric::control_step, this));

    RCLCPP_INFO(
      get_logger(), "following %s -> %s using odometry %s",
      path_topic_.c_str(), cmd_vel_topic_.c_str(), odometry_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter("sensor_offset_x", 0.0);
    declare_parameter("sensor_offset_y", 0.0);
    declare_parameter("publish_skip_count", 0);
    declare_parameter("two_way_drive", true);
    declare_parameter("look_ahead_distance", 0.5);
    declare_parameter("yaw_rate_gain", 7.5);
    declare_parameter("stop_yaw_rate_gain", 7.5);
    declare_parameter("max_yaw_rate_deg_s", 45.0);
    declare_parameter("max_speed_m_s", 1.0);
    declare_parameter("max_accel_m_s2", 1.0);
    declare_parameter("switch_time_threshold_s", 1.0);
    declare_parameter("direction_difference_threshold_rad", 0.1);
    declare_parameter("stop_distance_threshold_m", 0.2);
    declare_parameter("slow_down_distance_threshold_m", 1.0);
    declare_parameter("use_inclination_rate_to_slow", false);
    declare_parameter("inclination_rate_threshold_deg_s", 120.0);
    declare_parameter("slow_rate_1", 0.25);
    declare_parameter("slow_rate_2", 0.5);
    declare_parameter("slow_time_1_s", 2.0);
    declare_parameter("slow_time_2_s", 2.0);
    declare_parameter("use_inclination_to_stop", false);
    declare_parameter("inclination_threshold_deg", 45.0);
    declare_parameter("stop_time_s", 5.0);
    declare_parameter("no_rotation_at_stop", false);
    declare_parameter("no_rotation_at_goal", true);
    declare_parameter("autonomy_mode", false);
    declare_parameter("autonomy_speed_m_s", 1.0);
    declare_parameter("joy_to_speed_delay_s", 2.0);
    declare_parameter("odometry_timeout_s", 0.5);
    declare_parameter("update_rate_hz", 100.0);
    declare_parameter("odometry_topic", std::string("/localization/odometry"));
    declare_parameter("path_topic", std::string("/planning/local_path"));
    declare_parameter("joy_topic", std::string("/joy"));
    declare_parameter("speed_topic", std::string("/speed"));
    declare_parameter("stop_topic", std::string("/stop"));
    declare_parameter("cmd_vel_topic", std::string("/cmd_vel/nav"));
    declare_parameter("output_frame", std::string("base_link"));
  }

  void load_parameters()
  {
    sensor_offset_x_ = get_parameter("sensor_offset_x").as_double();
    sensor_offset_y_ = get_parameter("sensor_offset_y").as_double();
    publish_skip_count_ = get_parameter("publish_skip_count").as_int();
    two_way_drive_ = get_parameter("two_way_drive").as_bool();
    look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
    yaw_rate_gain_ = get_parameter("yaw_rate_gain").as_double();
    stop_yaw_rate_gain_ = get_parameter("stop_yaw_rate_gain").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate_deg_s").as_double() * kPi / 180.0;
    max_speed_ = get_parameter("max_speed_m_s").as_double();
    max_accel_ = get_parameter("max_accel_m_s2").as_double();
    switch_time_threshold_ = get_parameter("switch_time_threshold_s").as_double();
    direction_difference_threshold_ =
      get_parameter("direction_difference_threshold_rad").as_double();
    stop_distance_threshold_ = get_parameter("stop_distance_threshold_m").as_double();
    slow_down_distance_threshold_ =
      get_parameter("slow_down_distance_threshold_m").as_double();
    use_inclination_rate_to_slow_ =
      get_parameter("use_inclination_rate_to_slow").as_bool();
    inclination_rate_threshold_ =
      get_parameter("inclination_rate_threshold_deg_s").as_double() * kPi / 180.0;
    slow_rate_1_ = get_parameter("slow_rate_1").as_double();
    slow_rate_2_ = get_parameter("slow_rate_2").as_double();
    slow_time_1_ = get_parameter("slow_time_1_s").as_double();
    slow_time_2_ = get_parameter("slow_time_2_s").as_double();
    use_inclination_to_stop_ = get_parameter("use_inclination_to_stop").as_bool();
    inclination_threshold_ =
      get_parameter("inclination_threshold_deg").as_double() * kPi / 180.0;
    stop_time_ = get_parameter("stop_time_s").as_double();
    no_rotation_at_stop_ = get_parameter("no_rotation_at_stop").as_bool();
    no_rotation_at_goal_ = get_parameter("no_rotation_at_goal").as_bool();
    autonomy_mode_ = get_parameter("autonomy_mode").as_bool();
    autonomy_speed_ = get_parameter("autonomy_speed_m_s").as_double();
    joy_to_speed_delay_ = get_parameter("joy_to_speed_delay_s").as_double();
    odometry_timeout_ = get_parameter("odometry_timeout_s").as_double();
    update_rate_hz_ = get_parameter("update_rate_hz").as_double();
    odometry_topic_ = get_parameter("odometry_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    joy_topic_ = get_parameter("joy_topic").as_string();
    speed_topic_ = get_parameter("speed_topic").as_string();
    stop_topic_ = get_parameter("stop_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
  }

  void validate_parameters() const
  {
    if (publish_skip_count_ < 0 || look_ahead_distance_ <= 0.0 || max_yaw_rate_ <= 0.0 ||
      max_speed_ <= 0.0 || max_accel_ <= 0.0 || switch_time_threshold_ < 0.0 ||
      direction_difference_threshold_ < 0.0 || stop_distance_threshold_ < 0.0 ||
      slow_down_distance_threshold_ <= 0.0 || slow_rate_1_ < 0.0 || slow_rate_1_ > 1.0 ||
      slow_rate_2_ < 0.0 || slow_rate_2_ > 1.0 || slow_time_1_ < 0.0 ||
      slow_time_2_ < 0.0 || stop_time_ < 0.0 || joy_to_speed_delay_ < 0.0 ||
      odometry_timeout_ <= 0.0 || update_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("invalid path follower numeric parameter");
    }
    if (odometry_topic_.empty() || path_topic_.empty() || joy_topic_.empty() ||
      speed_topic_.empty() || stop_topic_.empty() || cmd_vel_topic_.empty() ||
      output_frame_.empty())
    {
      throw std::invalid_argument("path follower topics and output_frame must be non-empty");
    }
  }

  void odom_handler(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_stamp_ = msg->header.stamp;
    last_odom_receipt_ = now();
    have_odom_ = true;

    const auto & orientation = msg->pose.pose.orientation;
    tf2::Quaternion quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    vehicle_roll_ = roll;
    vehicle_pitch_ = pitch;
    vehicle_yaw_ = yaw;
    vehicle_x_ =
      msg->pose.pose.position.x - std::cos(yaw) * sensor_offset_x_ +
      std::sin(yaw) * sensor_offset_y_;
    vehicle_y_ =
      msg->pose.pose.position.y - std::sin(yaw) * sensor_offset_x_ -
      std::cos(yaw) * sensor_offset_y_;
    vehicle_z_ = msg->pose.pose.position.z;

    const double odom_time = rclcpp::Time(msg->header.stamp).seconds();
    if (use_inclination_to_stop_ &&
      (std::abs(roll) > inclination_threshold_ ||
      std::abs(pitch) > inclination_threshold_))
    {
      stop_init_time_ = odom_time;
    }
    if (use_inclination_rate_to_slow_ &&
      (std::abs(msg->twist.twist.angular.x) > inclination_rate_threshold_ ||
      std::abs(msg->twist.twist.angular.y) > inclination_rate_threshold_))
    {
      slow_init_time_ = odom_time;
    }
  }

  void path_handler(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      path_.poses.clear();
      path_initialized_ = false;
      vehicle_speed_ = 0.0;
      vehicle_yaw_rate_ = 0.0;
      publish_command(true);
      RCLCPP_WARN(get_logger(), "received an empty path; publishing a stop command");
      return;
    }

    path_ = *msg;
    vehicle_x_recorded_ = vehicle_x_;
    vehicle_y_recorded_ = vehicle_y_;
    vehicle_z_recorded_ = vehicle_z_;
    vehicle_roll_recorded_ = vehicle_roll_;
    vehicle_pitch_recorded_ = vehicle_pitch_;
    vehicle_yaw_recorded_ = vehicle_yaw_;
    path_point_id_ = 0;
    path_initialized_ = true;
    nav_forward_ = true;
    switch_time_ = now().seconds();
  }

  void joystick_handler(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() <= 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignoring Joy message with fewer than five axes");
      return;
    }

    joy_time_ = now().seconds();
    joy_speed_raw_ = std::hypot(msg->axes[3], msg->axes[4]);
    joy_speed_ = std::min(joy_speed_raw_, 1.0);
    if (msg->axes[4] == 0.0F) {
      joy_speed_ = 0.0;
    }
    joy_yaw_ = msg->axes[3];
    if (joy_speed_ == 0.0 && no_rotation_at_stop_) {
      joy_yaw_ = 0.0;
    }
    if (msg->axes[4] < 0.0F && !two_way_drive_) {
      joy_speed_ = 0.0;
      joy_yaw_ = 0.0;
    }

    if (msg->axes.size() > 2) {
      autonomy_mode_ = msg->axes[2] <= -0.1F;
    }
  }

  void speed_handler(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double speed_time = now().seconds();
    if (autonomy_mode_ && speed_time - joy_time_ > joy_to_speed_delay_ &&
      joy_speed_raw_ == 0.0)
    {
      joy_speed_ = std::clamp(static_cast<double>(msg->data) / max_speed_, 0.0, 1.0);
    }
  }

  void stop_handler(const std_msgs::msg::Int8::SharedPtr msg)
  {
    safety_stop_ = msg->data;
  }

  void control_step()
  {
    if (!path_initialized_) {
      return;
    }
    if (!have_odom_ || (now() - last_odom_receipt_).seconds() > odometry_timeout_) {
      vehicle_speed_ = 0.0;
      vehicle_yaw_rate_ = 0.0;
      publish_command(true);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "odometry is missing or stale; publishing a stop command");
      return;
    }

    const double vehicle_x_relative =
      std::cos(vehicle_yaw_recorded_) * (vehicle_x_ - vehicle_x_recorded_) +
      std::sin(vehicle_yaw_recorded_) * (vehicle_y_ - vehicle_y_recorded_);
    const double vehicle_y_relative =
      -std::sin(vehicle_yaw_recorded_) * (vehicle_x_ - vehicle_x_recorded_) +
      std::cos(vehicle_yaw_recorded_) * (vehicle_y_ - vehicle_y_recorded_);

    const std::size_t path_size = path_.poses.size();
    const double end_dx = path_.poses.back().pose.position.x - vehicle_x_relative;
    const double end_dy = path_.poses.back().pose.position.y - vehicle_y_relative;
    const double end_distance = std::hypot(end_dx, end_dy);

    double dx = 0.0;
    double dy = 0.0;
    double distance = 0.0;
    while (path_point_id_ + 1 < path_size) {
      dx = path_.poses[path_point_id_].pose.position.x - vehicle_x_relative;
      dy = path_.poses[path_point_id_].pose.position.y - vehicle_y_relative;
      distance = std::hypot(dx, dy);
      if (distance >= look_ahead_distance_) {
        break;
      }
      ++path_point_id_;
    }

    dx = path_.poses[path_point_id_].pose.position.x - vehicle_x_relative;
    dy = path_.poses[path_point_id_].pose.position.y - vehicle_y_relative;
    distance = std::hypot(dx, dy);
    const double path_direction = std::atan2(dy, dx);
    double direction_difference =
      normalize_angle(vehicle_yaw_ - vehicle_yaw_recorded_ - path_direction);

    const double current_time = now().seconds();
    if (two_way_drive_) {
      if (std::abs(direction_difference) > kPi / 2.0 && nav_forward_ &&
        current_time - switch_time_ > switch_time_threshold_)
      {
        nav_forward_ = false;
        switch_time_ = current_time;
      } else if (std::abs(direction_difference) < kPi / 2.0 && !nav_forward_ &&
        current_time - switch_time_ > switch_time_threshold_)
      {
        nav_forward_ = true;
        switch_time_ = current_time;
      }
    }

    double target_speed = max_speed_ * joy_speed_;
    if (!nav_forward_) {
      direction_difference = normalize_angle(direction_difference + kPi);
      target_speed *= -1.0;
    }

    const double acceleration_per_step = max_accel_ / update_rate_hz_;
    vehicle_yaw_rate_ =
      -(std::abs(vehicle_speed_) < 2.0 * acceleration_per_step ?
      stop_yaw_rate_gain_ : yaw_rate_gain_) * direction_difference;
    vehicle_yaw_rate_ = std::clamp(vehicle_yaw_rate_, -max_yaw_rate_, max_yaw_rate_);

    if (target_speed == 0.0 && !autonomy_mode_) {
      vehicle_yaw_rate_ = max_yaw_rate_ * joy_yaw_;
    } else if (path_size <= 1 ||
      (distance < stop_distance_threshold_ && no_rotation_at_goal_))
    {
      vehicle_yaw_rate_ = 0.0;
    }

    if (path_size <= 1) {
      target_speed = 0.0;
    } else if (end_distance / slow_down_distance_threshold_ < joy_speed_) {
      target_speed *= end_distance / slow_down_distance_threshold_;
    }

    double limited_target_speed = target_speed;
    const double odom_time = rclcpp::Time(odom_stamp_).seconds();
    if (slow_init_time_ > 0.0 && odom_time < slow_init_time_ + slow_time_1_) {
      limited_target_speed *= slow_rate_1_;
    } else if (
      slow_init_time_ > 0.0 &&
      odom_time < slow_init_time_ + slow_time_1_ + slow_time_2_)
    {
      limited_target_speed *= slow_rate_2_;
    }

    if (std::abs(direction_difference) < direction_difference_threshold_ &&
      distance > stop_distance_threshold_)
    {
      approach(vehicle_speed_, limited_target_speed, acceleration_per_step);
    } else {
      approach(vehicle_speed_, 0.0, acceleration_per_step);
    }

    if (stop_init_time_ > 0.0 && odom_time < stop_init_time_ + stop_time_) {
      vehicle_speed_ = 0.0;
      vehicle_yaw_rate_ = 0.0;
    }
    if (safety_stop_ >= 1) {
      vehicle_speed_ = 0.0;
    }
    if (safety_stop_ >= 2) {
      vehicle_yaw_rate_ = 0.0;
    }

    publish_command(false);
  }

  static void approach(double & value, double target, double increment)
  {
    if (value < target) {
      value = std::min(value + increment, target);
    } else if (value > target) {
      value = std::max(value - increment, target);
    }
  }

  void publish_command(bool force)
  {
    if (!force) {
      if (publish_skip_remaining_ > 0) {
        --publish_skip_remaining_;
        return;
      }
      publish_skip_remaining_ = publish_skip_count_;
    }

    geometry_msgs::msg::TwistStamped command;
    if (have_odom_) {
      command.header.stamp = odom_stamp_;
    } else {
      command.header.stamp = now();
    }
    command.header.frame_id = output_frame_;
    command.twist.linear.x =
      std::abs(vehicle_speed_) <= max_accel_ / update_rate_hz_ ? 0.0 : vehicle_speed_;
    command.twist.angular.z = vehicle_yaw_rate_;
    cmd_vel_pub_->publish(command);
  }

  double sensor_offset_x_{0.0};
  double sensor_offset_y_{0.0};
  int64_t publish_skip_count_{0};
  int64_t publish_skip_remaining_{0};
  bool two_way_drive_{true};
  double look_ahead_distance_{0.5};
  double yaw_rate_gain_{7.5};
  double stop_yaw_rate_gain_{7.5};
  double max_yaw_rate_{0.0};
  double max_speed_{1.0};
  double max_accel_{1.0};
  double switch_time_threshold_{1.0};
  double direction_difference_threshold_{0.1};
  double stop_distance_threshold_{0.2};
  double slow_down_distance_threshold_{1.0};
  bool use_inclination_rate_to_slow_{false};
  double inclination_rate_threshold_{0.0};
  double slow_rate_1_{0.25};
  double slow_rate_2_{0.5};
  double slow_time_1_{2.0};
  double slow_time_2_{2.0};
  bool use_inclination_to_stop_{false};
  double inclination_threshold_{0.0};
  double stop_time_{5.0};
  bool no_rotation_at_stop_{false};
  bool no_rotation_at_goal_{true};
  bool autonomy_mode_{false};
  double autonomy_speed_{1.0};
  double joy_to_speed_delay_{2.0};
  double odometry_timeout_{0.5};
  double update_rate_hz_{100.0};
  std::string odometry_topic_;
  std::string path_topic_;
  std::string joy_topic_;
  std::string speed_topic_;
  std::string stop_topic_;
  std::string cmd_vel_topic_;
  std::string output_frame_;

  double joy_speed_{0.0};
  double joy_speed_raw_{0.0};
  double joy_yaw_{0.0};
  int safety_stop_{0};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double vehicle_z_{0.0};
  double vehicle_roll_{0.0};
  double vehicle_pitch_{0.0};
  double vehicle_yaw_{0.0};
  double vehicle_x_recorded_{0.0};
  double vehicle_y_recorded_{0.0};
  double vehicle_z_recorded_{0.0};
  double vehicle_roll_recorded_{0.0};
  double vehicle_pitch_recorded_{0.0};
  double vehicle_yaw_recorded_{0.0};
  double vehicle_yaw_rate_{0.0};
  double vehicle_speed_{0.0};
  double joy_time_{0.0};
  double slow_init_time_{0.0};
  double stop_init_time_{0.0};
  std::size_t path_point_id_{0};
  bool path_initialized_{false};
  bool nav_forward_{true};
  bool have_odom_{false};
  double switch_time_{0.0};
  builtin_interfaces::msg::Time odom_stamp_;
  rclcpp::Time last_odom_receipt_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PathFollowerGeneric>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("path_follower_generic"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
