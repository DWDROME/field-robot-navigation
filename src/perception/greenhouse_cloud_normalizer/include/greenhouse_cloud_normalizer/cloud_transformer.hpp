#ifndef GREENHOUSE_CLOUD_NORMALIZER__CLOUD_TRANSFORMER_HPP_
#define GREENHOUSE_CLOUD_NORMALIZER__CLOUD_TRANSFORMER_HPP_

#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer_interface.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace greenhouse_cloud_normalizer
{

class CloudTransformer
{
public:
  explicit CloudTransformer(const tf2_ros::BufferInterface & buffer)
  : buffer_(buffer) {}

  sensor_msgs::msg::PointCloud2 transform(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::string & target_frame,
    const tf2::Duration timeout) const
  {
    if (input.header.frame_id.empty()) {
      throw std::invalid_argument("input PointCloud2 frame_id must not be empty");
    }
    if (target_frame.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }

    sensor_msgs::msg::PointCloud2 output;
    buffer_.transform(input, output, target_frame, timeout);
    return output;
  }

private:
  const tf2_ros::BufferInterface & buffer_;
};

}  // namespace greenhouse_cloud_normalizer

#endif  // GREENHOUSE_CLOUD_NORMALIZER__CLOUD_TRANSFORMER_HPP_
