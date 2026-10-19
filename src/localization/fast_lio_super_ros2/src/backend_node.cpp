// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <algorithm>

namespace {
using Odom = nav_msgs::msg::Odometry;
using CloudMessage = sensor_msgs::msg::PointCloud2;
using Policy = message_filters::sync_policies::ExactTime<Odom,CloudMessage>;
using slam::graph::Pose;
Pose pose(const geometry_msgs::msg::Pose& message) {
  Eigen::Quaterniond q(message.orientation.w,message.orientation.x,
                       message.orientation.y,message.orientation.z);
  if (!q.coeffs().allFinite() || std::fabs(q.norm()-1) > 1e-3)
    throw std::invalid_argument("invalid odometry quaternion");
  Pose p = Pose::Identity();
  p.linear() = q.normalized().toRotationMatrix();
  p.translation() = Eigen::Vector3d(message.position.x,message.position.y,message.position.z);
  if (!slam::graph::validPose(p)) throw std::invalid_argument("invalid odometry pose");
  return p;
}
geometry_msgs::msg::Pose message(const Pose& p) {
  geometry_msgs::msg::Pose m;
  m.position.x=p.translation().x(); m.position.y=p.translation().y(); m.position.z=p.translation().z();
  const Eigen::Quaterniond q(p.linear());
  m.orientation.w=q.w(); m.orientation.x=q.x(); m.orientation.y=q.y(); m.orientation.z=q.z();
  return m;
}
class Node final : public rclcpp::Node {
 public:
  Node() : rclcpp::Node("fast_lio_super_backend") {
    if (!declare_parameter("enabled",true)) return;
    slam::graph::Config cfg;
#define DOUBLE_PARAM(name) cfg.name = declare_parameter(#name,cfg.name)
#define INT_PARAM(name) cfg.name = declare_parameter(#name,cfg.name)
#define SIZE_PARAM(name) { auto n=declare_parameter<int64_t>(#name,cfg.name); if(n<=0) throw std::invalid_argument(#name); cfg.name=n; }
    DOUBLE_PARAM(keyframe_translation); DOUBLE_PARAM(keyframe_rotation); DOUBLE_PARAM(voxel);
    DOUBLE_PARAM(candidate_period); DOUBLE_PARAM(min_loop_seconds); INT_PARAM(min_loop_keyframes);
    INT_PARAM(submap_frames); SIZE_PARAM(max_keyframes); SIZE_PARAM(max_frame_points);
    SIZE_PARAM(max_cloud_points); SIZE_PARAM(queue_capacity); INT_PARAM(rings); INT_PARAM(sectors);
    INT_PARAM(candidate_count); DOUBLE_PARAM(descriptor_radius); DOUBLE_PARAM(sensor_height);
    DOUBLE_PARAM(descriptor_threshold); INT_PARAM(gicp_iterations); INT_PARAM(min_inliers);
    DOUBLE_PARAM(correspondence_distance); DOUBLE_PARAM(min_inlier_ratio); DOUBLE_PARAM(max_mean_square_error);
    DOUBLE_PARAM(odom_rotation_sigma); DOUBLE_PARAM(odom_translation_sigma);
    DOUBLE_PARAM(loop_rotation_sigma); DOUBLE_PARAM(loop_translation_sigma);
    DOUBLE_PARAM(relinearize_threshold); INT_PARAM(relinearize_skip);
#undef DOUBLE_PARAM
#undef INT_PARAM
#undef SIZE_PARAM
    cfg.validate();
    frame_points_ = cfg.max_frame_points;
    map_frame_=declare_parameter<std::string>("map_frame","map");
    odom_frame_=declare_parameter<std::string>("odom_frame","odom");
    body_frame_=declare_parameter<std::string>("body_frame","base_link");
    if (map_frame_==odom_frame_ || body_frame_==odom_frame_ || map_frame_==body_frame_ ||
        map_frame_.empty() || odom_frame_.empty() || body_frame_.empty())
      throw std::invalid_argument("invalid SLAM frame ownership");
    const double output_period=declare_parameter("output_period",2.0);
    if (!std::isfinite(output_period) || output_period<=0) throw std::invalid_argument("output_period");
    backend_=std::make_unique<slam::graph::Backend>(cfg);
    corrected_=create_publisher<Odom>("global_odometry",rclcpp::QoS(10));
    global_pose_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose",rclcpp::QoS(10));
    path_=create_publisher<nav_msgs::msg::Path>("corrected_path",rclcpp::QoS(1));
    map_=create_publisher<CloudMessage>("corrected_map",rclcpp::QoS(1));
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",rclcpp::QoS(10));
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sync_=std::make_unique<message_filters::Synchronizer<Policy>>(Policy(10));
    sync_->registerCallback(&Node::ingest,this);
    odometry_=create_subscription<Odom>("odometry",rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr m){
        observeOdometry(m); publishPose(*m); sync_->add<0>(m);
      });
    cloud_input_=create_subscription<CloudMessage>("registered_cloud",rclcpp::SensorDataQoS(),
      [this](CloudMessage::ConstSharedPtr m){
        observeCloud(m); sync_->add<1>(m);
      });
    timer_=create_wall_timer(std::chrono::duration<double>(output_period),[this]{ publishOutputs(); });
    RCLCPP_INFO(get_logger(),"SLAM frames: %s -> %s -> %s",map_frame_.c_str(),odom_frame_.c_str(),body_frame_.c_str());
  }
 private:
  void observeOdometry(const Odom::ConstSharedPtr& o) {
    ++odom_messages_;
    last_odom_stamp_=static_cast<std::size_t>(rclcpp::Time(o->header.stamp).nanoseconds());
  }
  void observeCloud(const CloudMessage::ConstSharedPtr& c) {
    ++cloud_messages_;
    last_cloud_stamp_=static_cast<std::size_t>(rclcpp::Time(c->header.stamp).nanoseconds());
  }
  void ingest(const Odom::ConstSharedPtr& o,const CloudMessage::ConstSharedPtr& c) {
    ++synchronized_frames_;
    RCLCPP_DEBUG(get_logger(),"synchronized input: %s / %s",o->header.frame_id.c_str(),c->header.frame_id.c_str());
    try {
      if (o->header.frame_id!=odom_frame_ || o->child_frame_id!=body_frame_ ||
          c->header.frame_id!=odom_frame_) throw std::invalid_argument("synchronized cloud/odometry frame mismatch");
      if (c->width==0 || c->height==0 || c->point_step==0 ||
          c->data.size()>64*1024*1024) throw std::invalid_argument("invalid or oversized registered cloud");
      slam::graph::Cloud cloud;
      pcl::fromROSMsg(*c,cloud);
      slam::graph::Cloud::Ptr bounded(new slam::graph::Cloud);
      const auto count=std::min(frame_points_,cloud.size());
      bounded->reserve(count);
      for (std::size_t i=0;i<count;++i) bounded->push_back(cloud[i*cloud.size()/count]);
      const auto original=pose(o->pose.pose);
      if (!backend_->submit(rclcpp::Time(o->header.stamp).seconds(),original,
             slam::graph::transformCloud(*bounded,original.inverse())))
        throw std::invalid_argument("backend rejected frame");
    } catch(const std::exception& e) { RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what()); }
  }
  void publishPose(const Odom& raw) {
    RCLCPP_DEBUG(get_logger(),"local odometry input: %s -> %s",raw.header.frame_id.c_str(),raw.child_frame_id.c_str());
    try {
      if(raw.header.frame_id!=odom_frame_ || raw.child_frame_id!=body_frame_) return;
      const auto correction=backend_->snapshot()->map_from_odom;
      Odom corrected=raw; corrected.header.frame_id=map_frame_;
      corrected.pose.pose=message(correction*pose(raw.pose.pose));
      Eigen::Matrix<double,6,6> rotation=Eigen::Matrix<double,6,6>::Zero();
      rotation.block<3,3>(0,0)=correction.linear(); rotation.block<3,3>(3,3)=correction.linear();
      Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>> covariance(raw.pose.covariance.data());
      Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> output_covariance(corrected.pose.covariance.data());
      output_covariance=rotation*covariance*rotation.transpose();
      corrected_->publish(corrected);
      geometry_msgs::msg::PoseWithCovarianceStamped global;
      global.header=corrected.header; global.pose=corrected.pose; global_pose_->publish(global);
      geometry_msgs::msg::TransformStamped t;
      t.header=raw.header; t.header.frame_id=map_frame_; t.child_frame_id=odom_frame_;
      const auto m=message(correction);
      t.transform.translation.x=m.position.x; t.transform.translation.y=m.position.y; t.transform.translation.z=m.position.z;
      t.transform.rotation=m.orientation; tf_->sendTransform(t);
    } catch(const std::exception& e) { RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what()); }
  }
  void publishOutputs() {
    const auto state=backend_->snapshot();
    diagnostic_msgs::msg::DiagnosticArray d; d.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="fast_lio_super_backend"; status.hardware_id="software"; status.message=state->status;
    status.level=state->status.find("error")!=std::string::npos ? 2 :
      (state->status.find("exhausted")!=std::string::npos ? 1 : 0);
    for (auto pair : {std::make_pair("keyframes",state->keyframes.size()),
         std::make_pair("accepted_loops",state->accepted_loops),
         std::make_pair("rejected_loops",state->rejected_loops),
         std::make_pair("dropped_frames",state->dropped_frames),
         std::make_pair("odometry_messages",odom_messages_),
         std::make_pair("cloud_messages",cloud_messages_),
         std::make_pair("synchronized_frames",synchronized_frames_),
         std::make_pair("last_odometry_stamp_ns",last_odom_stamp_),
         std::make_pair("last_cloud_stamp_ns",last_cloud_stamp_)}) {
      diagnostic_msgs::msg::KeyValue v; v.key=pair.first; v.value=std::to_string(pair.second); status.values.push_back(v);
    }
    d.status.push_back(status); diagnostics_->publish(d);
    if(state->keyframes.empty()) return;
    nav_msgs::msg::Path path; path.header.frame_id=map_frame_;
    path.header.stamp=rclcpp::Time(static_cast<int64_t>(state->keyframes.back().stamp*1e9));
    if(path_->get_subscription_count()) {
      for(const auto& frame:state->keyframes) {
        geometry_msgs::msg::PoseStamped p; p.header=path.header;
        p.header.stamp=rclcpp::Time(static_cast<int64_t>(frame.stamp*1e9));
        p.pose=message(frame.optimized); path.poses.push_back(p);
      }
      path_->publish(path);
    }
    if(map_->get_subscription_count()) {
      slam::graph::Cloud cloud;
      for(const auto& frame:state->keyframes) cloud+=*slam::graph::transformCloud(*frame.cloud,frame.optimized);
      CloudMessage m; pcl::toROSMsg(cloud,m); m.header=path.header; map_->publish(m);
    }
  }
  std::size_t frame_points_=0;
  std::size_t odom_messages_=0,cloud_messages_=0,synchronized_frames_=0;
  std::size_t last_odom_stamp_=0,last_cloud_stamp_=0;
  std::string map_frame_,odom_frame_,body_frame_;
  std::unique_ptr<slam::graph::Backend> backend_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<Odom>::SharedPtr odometry_;
  rclcpp::Subscription<CloudMessage>::SharedPtr cloud_input_;
  rclcpp::Publisher<Odom>::SharedPtr corrected_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<CloudMessage>::SharedPtr map_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try { rclcpp::spin(std::make_shared<Node>()); }
  catch(const std::exception& e) { RCLCPP_FATAL(rclcpp::get_logger("fast_lio_super_backend"),"%s",e.what()); result=2; }
  rclcpp::shutdown(); return result;
}
