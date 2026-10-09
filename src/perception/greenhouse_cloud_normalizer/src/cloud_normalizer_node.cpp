#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "greenhouse_cloud_normalizer/cloud_transformer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace greenhouse_cloud_normalizer
{

class CloudNormalizerNode final : public rclcpp::Node
{
public:
  CloudNormalizerNode()
  : Node("greenhouse_cloud_normalizer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sensors/lidar/raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/registered_cloud");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.1);

    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (output_topic_.empty()) {
      throw std::invalid_argument("output_topic must not be empty");
    }
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    if (transform_timeout_s_ < 0.0) {
      throw std::invalid_argument("transform_timeout_s must be non-negative");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    cloud_transformer_ = std::make_unique<CloudTransformer>(*tf_buffer_);

    const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(5);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, cloud_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, cloud_qos,
      std::bind(&CloudNormalizerNode::receive, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Transforming %s into frame %s on %s",
      input_topic_.c_str(), target_frame_.c_str(), output_topic_.c_str());
  }

private:
  void receive(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    try {
      publisher_->publish(
        cloud_transformer_->transform(
          *message, target_frame_, tf2::durationFromSec(transform_timeout_s_)));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 transform %s -> %s unavailable: %s",
        message->header.frame_id.c_str(), target_frame_.c_str(), error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 transformation rejected: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_s_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<CloudTransformer> cloud_transformer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace greenhouse_cloud_normalizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<greenhouse_cloud_normalizer::CloudNormalizerNode>());
  rclcpp::shutdown();
  return 0;
}
