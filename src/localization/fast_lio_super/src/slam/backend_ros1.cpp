// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

namespace {
using slam::graph::Pose;
Pose pose(const geometry_msgs::Pose& m) {
  Eigen::Quaterniond q(m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z);
  if (!q.coeffs().allFinite() || std::fabs(q.norm()-1)>1e-3)
    throw std::invalid_argument("invalid quaternion");
  Pose p=Pose::Identity(); p.linear()=q.normalized().toRotationMatrix();
  p.translation()=Eigen::Vector3d(m.position.x,m.position.y,m.position.z);
  if (!slam::graph::validPose(p)) throw std::invalid_argument("invalid pose");
  return p;
}
geometry_msgs::Pose message(const Pose& p) {
  geometry_msgs::Pose m; m.position.x=p.translation().x(); m.position.y=p.translation().y();
  m.position.z=p.translation().z(); const Eigen::Quaterniond q(p.linear());
  m.orientation.w=q.w(); m.orientation.x=q.x(); m.orientation.y=q.y(); m.orientation.z=q.z();
  return m;
}
class BackendNode {
  using Policy=message_filters::sync_policies::ExactTime<nav_msgs::Odometry,sensor_msgs::PointCloud2>;
 public:
  BackendNode() : private_("~"), odom_(node_,"/Odometry",10), cloud_(node_,"/cloud_registered",10),
    sync_(Policy(10),odom_,cloud_) {
    slam::graph::Config c;
#define PARAM(name) private_.param(#name,c.name,c.name)
    PARAM(keyframe_translation); PARAM(keyframe_rotation); PARAM(voxel); PARAM(candidate_period);
    PARAM(min_loop_seconds); PARAM(min_loop_keyframes); PARAM(submap_frames); PARAM(rings);
    PARAM(sectors); PARAM(candidate_count); PARAM(descriptor_radius); PARAM(sensor_height);
    PARAM(descriptor_threshold); PARAM(gicp_iterations); PARAM(min_inliers);
    PARAM(correspondence_distance); PARAM(min_inlier_ratio); PARAM(max_mean_square_error);
    PARAM(odom_rotation_sigma); PARAM(odom_translation_sigma); PARAM(loop_rotation_sigma);
    PARAM(loop_translation_sigma); PARAM(relinearize_threshold); PARAM(relinearize_skip);
#undef PARAM
#define BUDGET(name) { int n=static_cast<int>(c.name); private_.param(#name,n,n); \
  if(n<=0) throw std::invalid_argument(#name); c.name=n; }
    BUDGET(max_keyframes); BUDGET(max_frame_points); BUDGET(max_cloud_points); BUDGET(queue_capacity);
#undef BUDGET
    c.validate(); points_=c.max_frame_points;
    private_.param<std::string>("map_frame",map_frame_,"map");
    private_.param<std::string>("odom_frame",odom_frame_,"camera_init");
    private_.param<std::string>("body_frame",body_frame_,"body");
    if(map_frame_.empty()||odom_frame_.empty()||body_frame_.empty()||map_frame_==odom_frame_||
       odom_frame_==body_frame_||map_frame_==body_frame_) throw std::invalid_argument("SLAM frames");
    backend_=std::make_unique<slam::graph::Backend>(c);
    corrected_=node_.advertise<nav_msgs::Odometry>("/slam/global_odometry",10);
    path_=node_.advertise<nav_msgs::Path>("/slam/corrected_path",1);
    map_=node_.advertise<sensor_msgs::PointCloud2>("/slam/corrected_map",1);
    diagnostics_=node_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",10);
    raw_=node_.subscribe("/Odometry",10,&BackendNode::publishPose,this);
    sync_.registerCallback(boost::bind(&BackendNode::ingest,this,boost::placeholders::_1,boost::placeholders::_2));
    double period=2; private_.param("output_period",period,period);
    if(!std::isfinite(period)||period<=0) throw std::invalid_argument("output_period");
    timer_=node_.createWallTimer(ros::WallDuration(period),&BackendNode::outputs,this);
  }
 private:
  void ingest(const nav_msgs::Odometry::ConstPtr& o,const sensor_msgs::PointCloud2::ConstPtr& m) {
    try {
      if(o->header.frame_id!=odom_frame_||o->child_frame_id!=body_frame_||m->header.frame_id!=odom_frame_||
         m->data.size()>64*1024*1024) throw std::invalid_argument("registered cloud/odom contract");
      slam::graph::Cloud cloud; pcl::fromROSMsg(*m,cloud);
      slam::graph::Cloud::Ptr bounded(new slam::graph::Cloud);
      const auto count=std::min(points_,cloud.size()); bounded->reserve(count);
      for(std::size_t i=0;i<count;++i) bounded->push_back(cloud[i*cloud.size()/count]);
      const auto p=pose(o->pose.pose);
      if(!backend_->submit(o->header.stamp.toSec(),p,slam::graph::transformCloud(*bounded,p.inverse())))
        throw std::invalid_argument("backend rejected frame");
    } catch(const std::exception& e) { ROS_ERROR_THROTTLE(2,"%s",e.what()); }
  }
  void publishPose(const nav_msgs::Odometry::ConstPtr& raw) {
    try {
      if(raw->header.frame_id!=odom_frame_||raw->child_frame_id!=body_frame_) return;
      const auto correction=backend_->snapshot()->map_from_odom;
      nav_msgs::Odometry global=*raw; global.header.frame_id=map_frame_;
      global.pose.pose=message(correction*pose(raw->pose.pose));
      Eigen::Matrix<double,6,6> rotation=Eigen::Matrix<double,6,6>::Zero();
      rotation.block<3,3>(0,0)=correction.linear(); rotation.block<3,3>(3,3)=correction.linear();
      Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>> covariance(raw->pose.covariance.data());
      Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> output(global.pose.covariance.data());
      output=rotation*covariance*rotation.transpose(); corrected_.publish(global);
      const auto m=message(correction);
      tf::Transform transform; transform.setOrigin(tf::Vector3(m.position.x,m.position.y,m.position.z));
      transform.setRotation(tf::Quaternion(m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w));
      tf_.sendTransform(tf::StampedTransform(transform,raw->header.stamp,map_frame_,odom_frame_));
    } catch(const std::exception& e) { ROS_ERROR_THROTTLE(2,"%s",e.what()); }
  }
  void outputs(const ros::WallTimerEvent&) {
    const auto s=backend_->snapshot(); diagnostic_msgs::DiagnosticArray d; d.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status; status.name="fast_lio_super_backend";
    status.hardware_id="software"; status.message=s->status;
    status.level=s->status.find("error")!=std::string::npos?2:(s->status.find("exhausted")!=std::string::npos?1:0);
    for(auto pair:{std::make_pair("keyframes",s->keyframes.size()),std::make_pair("accepted_loops",s->accepted_loops),
        std::make_pair("rejected_loops",s->rejected_loops),std::make_pair("dropped_frames",s->dropped_frames)}) {
      diagnostic_msgs::KeyValue v; v.key=pair.first; v.value=std::to_string(pair.second); status.values.push_back(v);
    }
    d.status.push_back(status); diagnostics_.publish(d); if(s->keyframes.empty()) return;
    nav_msgs::Path p; p.header.frame_id=map_frame_; p.header.stamp.fromSec(s->keyframes.back().stamp);
    if(path_.getNumSubscribers()) {
      for(const auto& f:s->keyframes) { geometry_msgs::PoseStamped m; m.header=p.header;
        m.header.stamp.fromSec(f.stamp); m.pose=message(f.optimized); p.poses.push_back(m); }
      path_.publish(p);
    }
    if(map_.getNumSubscribers()) {
      slam::graph::Cloud cloud; for(const auto& f:s->keyframes) cloud+=*slam::graph::transformCloud(*f.cloud,f.optimized);
      sensor_msgs::PointCloud2 m; pcl::toROSMsg(cloud,m); m.header=p.header; map_.publish(m);
    }
  }
  ros::NodeHandle node_,private_; std::size_t points_;
  std::string map_frame_,odom_frame_,body_frame_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_;
  message_filters::Synchronizer<Policy> sync_;
  std::unique_ptr<slam::graph::Backend> backend_; tf::TransformBroadcaster tf_;
  ros::Publisher corrected_,path_,map_,diagnostics_; ros::Subscriber raw_; ros::WallTimer timer_;
};
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"fast_lio_super_backend");
  try { BackendNode node; ros::spin(); }
  catch(const std::exception& e) { ROS_FATAL("%s",e.what()); return 2; }
  return 0;
}
