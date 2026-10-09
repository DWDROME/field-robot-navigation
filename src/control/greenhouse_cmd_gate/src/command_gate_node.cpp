#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "greenhouse_cmd_gate/command_gate_core.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"

namespace greenhouse_cmd_gate
{
namespace
{

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

std::int64_t seconds_to_nanoseconds(const double seconds)
{
  constexpr double kNanosecondsPerSecond = 1'000'000'000.0;
  constexpr double kMaximumSeconds =
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
    kNanosecondsPerSecond;
  if (
    !std::isfinite(seconds) ||
    seconds <= 0.0 ||
    seconds > kMaximumSeconds ||
    seconds * kNanosecondsPerSecond < 1.0)
  {
    throw std::invalid_argument("duration parameter must be finite and positive");
  }
  return static_cast<std::int64_t>(seconds * kNanosecondsPerSecond);
}

std::int64_t nonnegative_seconds_to_nanoseconds(const double seconds)
{
  constexpr double kNanosecondsPerSecond = 1'000'000'000.0;
  constexpr double kMaximumSeconds =
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
    kNanosecondsPerSecond;
  if (
    !std::isfinite(seconds) ||
    seconds < 0.0 ||
    seconds > kMaximumSeconds)
  {
    throw std::invalid_argument(
            "duration parameter must be finite and nonnegative");
  }
  return static_cast<std::int64_t>(seconds * kNanosecondsPerSecond);
}

void require_absolute_topic(const std::string & name, const std::string & value)
{
  if (value.empty() || value.front() != '/') {
    throw std::invalid_argument(name + " must be an absolute ROS topic");
  }
}

diagnostic_msgs::msg::KeyValue key_value(
  std::string key,
  std::string value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = std::move(key);
  pair.value = std::move(value);
  return pair;
}

std::string reliability_name(const rmw_qos_reliability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "best_effort";
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
      return "unknown";
    default:
      return "invalid";
  }
}

std::string durability_name(const rmw_qos_durability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "volatile";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "transient_local";
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
      return "unknown";
    default:
      return "invalid";
  }
}

std::string history_name(const rmw_qos_history_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      return "keep_last";
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return "keep_all";
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      return "unknown";
    default:
      return "invalid";
  }
}

}  // namespace

class CommandGateNode final : public rclcpp::Node
{
public:
  CommandGateNode()
  : Node("greenhouse_cmd_gate"),
    source_topic_(required_string_parameter("source_topic")),
    output_topic_(required_string_parameter("output_topic")),
    output_frame_(required_string_parameter("output_frame")),
    qualification_scope_(required_string_parameter("qualification_scope")),
    publish_rate_hz_(required_double_parameter("publish_rate_hz")),
    limits_(load_limits()),
    core_(limits_)
  {
    require_absolute_topic("source_topic", source_topic_);
    require_absolute_topic("output_topic", output_topic_);
    if (source_topic_ == output_topic_) {
      throw std::invalid_argument("source_topic and output_topic must differ");
    }
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();
    output_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, command_qos);
    source_subscription_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
      source_topic_,
      command_qos,
      std::bind(&CommandGateNode::on_command, this, std::placeholders::_1));
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = rclcpp::create_timer(
      get_node_base_interface(),
      get_node_timers_interface(),
      get_clock(),
      period,
      std::bind(&CommandGateNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "gate source=%s output=%s scope=%s auto_recover_after_watchdog=%s",
      source_topic_.c_str(),
      output_topic_.c_str(),
      qualification_scope_.c_str(),
      limits_.auto_recover_after_watchdog ? "true" : "false");
  }

private:
  std::string required_string_parameter(const std::string & name)
  {
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
    const auto parameter = get_parameter(name);
    if (
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameter.as_string().empty())
    {
      throw std::invalid_argument(name + " is required");
    }
    return parameter.as_string();
  }

  double required_double_parameter(const std::string & name)
  {
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
    const auto parameter = get_parameter(name);
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      throw std::invalid_argument(name + " is required");
    }
    return parameter.as_double();
  }

  bool required_bool_parameter(const std::string & name)
  {
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_BOOL);
    const auto parameter = get_parameter(name);
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      throw std::invalid_argument(name + " is required");
    }
    return parameter.as_bool();
  }

  GateLimits load_limits()
  {
    return {
      required_double_parameter("max_linear_velocity"),
      required_double_parameter("max_angular_velocity"),
      required_double_parameter("max_linear_acceleration"),
      required_double_parameter("max_angular_acceleration"),
      seconds_to_nanoseconds(
        required_double_parameter("watchdog_timeout_sec")),
      seconds_to_nanoseconds(
        required_double_parameter("max_command_age_sec")),
      nonnegative_seconds_to_nanoseconds(
        required_double_parameter("future_tolerance_sec")),
      required_bool_parameter("auto_recover_after_watchdog"),
    };
  }

  void on_command(
    const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    source_owner_count_ = count_publishers(source_topic_);
    if (source_owner_count_ > 1U) {
      core_.reject_duplicate_source();
      return;
    }
    const auto now_ns = get_clock()->now().nanoseconds();
    const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    core_.accept(
      {
        message->twist.linear.x,
        message->twist.angular.z,
        stamp_ns,
      },
      now_ns);
  }

  void on_timer()
  {
    source_owner_count_ = count_publishers(source_topic_);
    if (source_owner_count_ > 1U) {
      core_.reject_duplicate_source();
    }

    const auto now = get_clock()->now();
    const auto output = core_.update(now.nanoseconds());
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now;
    command.header.frame_id = output_frame_;
    command.twist.linear.x = output.linear_velocity;
    command.twist.angular.z = output.angular_velocity;
    output_publisher_->publish(command);
    publish_diagnostics(now, output.state);
  }

  void publish_diagnostics(
    const rclcpp::Time & now,
    const GateState state)
  {
    const auto actual_qos =
      output_publisher_->get_actual_qos().get_rmw_qos_profile();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "greenhouse_cmd_gate";
    status.hardware_id = "command_gate";
    status.message = std::string(to_string(state));
    status.level =
      state == GateState::kActive ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    if (
      state == GateState::kInvalidCommand ||
      state == GateState::kDuplicateSource ||
      state == GateState::kClockRegression ||
      state == GateState::kWatchdogLatched)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    status.values = {
      key_value("state", std::string(to_string(state))),
      key_value("source_topic", source_topic_),
      key_value("output_topic", output_topic_),
      key_value("source_owner_count", std::to_string(source_owner_count_)),
      key_value(
        "watchdog_latched",
        core_.watchdog_latched() ? "true" : "false"),
      key_value("qualification_scope", qualification_scope_),
      key_value("estop_interface_configured", "false"),
      key_value(
        "output_qos_reliability",
        reliability_name(actual_qos.reliability)),
      key_value(
        "output_qos_durability",
        durability_name(actual_qos.durability)),
      key_value("output_qos_history", history_name(actual_qos.history)),
      key_value("output_qos_depth", std::to_string(actual_qos.depth)),
    };

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now;
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  const std::string source_topic_;
  const std::string output_topic_;
  const std::string output_frame_;
  const std::string qualification_scope_;
  const double publish_rate_hz_;
  const GateLimits limits_;
  CommandGateCore core_;
  std::size_t source_owner_count_{0U};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    output_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    source_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace greenhouse_cmd_gate

int main(int argc, char ** argv)
{
  greenhouse_cmd_gate::stop_requested = 0;
  rclcpp::init(
    argc,
    argv,
    rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);
  if (
    std::signal(SIGINT, greenhouse_cmd_gate::request_stop) == SIG_ERR ||
    std::signal(SIGTERM, greenhouse_cmd_gate::request_stop) == SIG_ERR)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("greenhouse_cmd_gate"),
      "failed to install process signal handlers");
    rclcpp::shutdown();
    return 2;
  }

  try {
    const auto node =
      std::make_shared<greenhouse_cmd_gate::CommandGateNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (greenhouse_cmd_gate::stop_requested == 0) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("greenhouse_cmd_gate"),
      "configuration/runtime failure: %s",
      error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
