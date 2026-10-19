#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "greenhouse_cloud_normalizer/cloud_transformer.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 456;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  *x = 1.0F;
  *y = 2.0F;
  *z = 3.0F;
  return cloud;
}

geometry_msgs::msg::TransformStamped make_static_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "livox_frame";
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = -2.0;
  transform.transform.translation.z = 0.5;
  transform.transform.rotation.w = 1.0;
  return transform;
}

TEST(CloudTransformer, applies_tf2_transform_at_the_message_frame)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  ASSERT_TRUE(buffer.setTransform(make_static_transform(), "test_authority", true));
  greenhouse_cloud_normalizer::CloudTransformer transformer(buffer);

  const auto output = transformer.transform(
    make_cloud("livox_frame"), "map", tf2::durationFromSec(0.0));

  EXPECT_EQ(output.header.frame_id, "map");
  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(output, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(output, "z");
  EXPECT_FLOAT_EQ(*x, 11.0F);
  EXPECT_FLOAT_EQ(*y, 0.0F);
  EXPECT_FLOAT_EQ(*z, 3.5F);
}

TEST(CloudTransformer, rejects_an_empty_source_frame)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  greenhouse_cloud_normalizer::CloudTransformer transformer(buffer);

  EXPECT_THROW(
    transformer.transform(make_cloud(""), "map", tf2::durationFromSec(0.0)),
    std::invalid_argument);
}

TEST(CloudTransformer, exposes_missing_tf_as_a_tf2_exception)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  greenhouse_cloud_normalizer::CloudTransformer transformer(buffer);

  EXPECT_THROW(
    transformer.transform(make_cloud("livox_frame"), "map", tf2::durationFromSec(0.0)),
    tf2::TransformException);
}

}  // namespace
