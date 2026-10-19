// SPDX-License-Identifier: Apache-2.0
#include <greenhouse_cloud_normalizer/cloud_transformer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <cmath>
#include <chrono>

class TerrainGrid final : public rclcpp::Node {
 public:
  TerrainGrid():Node("terrain_grid"),buffer_(get_clock()),listener_(buffer_,this,true),transformer_(buffer_) {
    resolution_=declare_parameter("resolution",0.2); size_=declare_parameter("size",100);
    obstacle_=declare_parameter("obstacle_height",0.15); drop_=declare_parameter("drop_height",0.3);
    timeout_=declare_parameter("timeout",0.7); frame_=declare_parameter<std::string>("frame","odom");
    if(size_<10||size_>1000||!std::isfinite(resolution_)||resolution_<=0||!std::isfinite(obstacle_)||obstacle_<=0||
       !std::isfinite(drop_)||drop_<=0||!std::isfinite(timeout_)||timeout_<=0||frame_.empty())
      throw std::invalid_argument("terrain grid configuration");
    grid_=create_publisher<nav_msgs::msg::OccupancyGrid>("/perception/terrain_grid",rclcpp::QoS(1).reliable().transient_local());
    ready_=create_publisher<std_msgs::msg::Bool>("/perception/terrain_ready",10);
    odom_=create_subscription<nav_msgs::msg::Odometry>("/localization/odometry",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        if(m->header.frame_id!=frame_||m->child_frame_id!="base_link"||!std::isfinite(m->pose.pose.position.x)||
           !std::isfinite(m->pose.pose.position.y)||!fresh(m->header.stamp)) { has_odom_=false; return; }
        position_=m->pose.pose.position; odom_stamp_=m->header.stamp; odom_receipt_=std::chrono::steady_clock::now(); has_odom_=true;
      });
    terrain_=create_subscription<sensor_msgs::msg::PointCloud2>("/perception/terrain_map",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){receive(*m);});
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
      std_msgs::msg::Bool status; const auto t=std::chrono::steady_clock::now();
      status.data=has_grid_&&has_odom_&&fresh(grid_stamp_)&&fresh(odom_stamp_)&&
        std::chrono::duration<double>(t-grid_receipt_).count()<=timeout_&&
        std::chrono::duration<double>(t-odom_receipt_).count()<=timeout_;
      ready_->publish(status);
    });
  }
 private:
  bool fresh(const builtin_interfaces::msg::Time& t) const {
    const double age=(now()-rclcpp::Time(t)).seconds(); return age>=-0.1&&age<=timeout_&&rclcpp::Time(t).nanoseconds()>0;
  }
  void receive(const sensor_msgs::msg::PointCloud2& input) {
    has_grid_=false;
    if(!has_odom_||!fresh(input.header.stamp)||!fresh(odom_stamp_)||input.data.size()>64*1024*1024) return;
    try {
      const auto transformed=transformer_.transform(input,frame_,tf2::durationFromSec(0.05));
      pcl::PointCloud<pcl::PointXYZI> cloud; pcl::fromROSMsg(transformed,cloud);
      if(cloud.empty()||cloud.size()>1000000) return;
      nav_msgs::msg::OccupancyGrid grid; grid.header=input.header; grid.header.frame_id=frame_;
      grid.info.resolution=resolution_; grid.info.width=grid.info.height=size_; grid.info.origin.orientation.w=1;
      grid.info.origin.position.x=position_.x-size_*resolution_/2;
      grid.info.origin.position.y=position_.y-size_*resolution_/2;
      grid.data.assign(size_*size_,-1); bool observed=false;
      for(const auto& p:cloud) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.intensity)) continue;
        const double gx=(p.x-grid.info.origin.position.x)/resolution_;
        const double gy=(p.y-grid.info.origin.position.y)/resolution_;
        if(gx<0||gy<0||gx>=size_||gy>=size_) continue;
        const int x=std::floor(gx),y=std::floor(gy);
        auto& cell=grid.data[y*size_+x];
        // intensity is terrainAnalysis relative height, preserved through the rigid transform.
        const int cost=p.intensity>obstacle_||p.intensity < -drop_?100:0;
        cell=std::max<int>(cell,cost); observed=true;
      }
      if(!observed) return;
      grid_->publish(grid); grid_stamp_=input.header.stamp; grid_receipt_=std::chrono::steady_clock::now(); has_grid_=true;
    } catch(const std::exception& e) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what()); }
  }
  int size_; double resolution_,obstacle_,drop_,timeout_; std::string frame_;
  bool has_odom_=false,has_grid_=false; geometry_msgs::msg::Point position_;
  builtin_interfaces::msg::Time odom_stamp_,grid_stamp_;
  std::chrono::steady_clock::time_point odom_receipt_,grid_receipt_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
  greenhouse_cloud_normalizer::CloudTransformer transformer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); int result=0;
  try {rclcpp::spin(std::make_shared<TerrainGrid>());}
  catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("terrain_grid"),"%s",e.what());result=2;}
  rclcpp::shutdown(); return result;
}
