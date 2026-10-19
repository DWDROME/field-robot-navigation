#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace greenhouse_cmd_gate
{

class CommandSinkNode final : public rclcpp::Node
{
public:
  CommandSinkNode()
  : Node("greenhouse_sim_cmd_sink"),
    input_topic_(required_topic_parameter("input_topic")),
    output_topic_(required_topic_parameter("output_topic"))
  {
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must differ");
    }
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();
    output_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, command_qos);
    input_subscription_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_,
      command_qos,
      std::bind(&CommandSinkNode::on_command, this, std::placeholders::_1));
  }

private:
  std::string required_topic_parameter(const std::string & name)
  {
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
    const auto parameter = get_parameter(name);
    if (
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameter.as_string().empty() ||
      parameter.as_string().front() != '/')
    {
      throw std::invalid_argument(name + " must be an absolute ROS topic");
    }
    return parameter.as_string();
  }

  void on_command(
    const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    const auto & linear = message->twist.linear;
    const auto & angular = message->twist.angular;
    if (
      !std::isfinite(linear.x) ||
      !std::isfinite(linear.y) ||
      !std::isfinite(linear.z) ||
      !std::isfinite(angular.x) ||
      !std::isfinite(angular.y) ||
      !std::isfinite(angular.z))
    {
      RCLCPP_ERROR(get_logger(), "refusing to forward non-finite command");
      return;
    }
    output_publisher_->publish(*message);
  }

  const std::string input_topic_;
  const std::string output_topic_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    output_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    input_subscription_;
};

}  // namespace greenhouse_cmd_gate

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<greenhouse_cmd_gate::CommandSinkNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("greenhouse_sim_cmd_sink"),
      "configuration/runtime failure: %s",
      error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
