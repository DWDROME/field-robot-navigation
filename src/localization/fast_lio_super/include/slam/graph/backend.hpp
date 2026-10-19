// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace slam::graph {
using Pose = Eigen::Isometry3d;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Config {
  double keyframe_translation = 1.0, keyframe_rotation = 0.2;
  double voxel = 0.3, candidate_period = 1.0, min_loop_seconds = 30.0;
  int min_loop_keyframes = 30, submap_frames = 5;
  std::size_t max_keyframes = 2000, max_frame_points = 20000;
  std::size_t max_cloud_points = 10000000, queue_capacity = 8;
  int rings = 20, sectors = 60, candidate_count = 10;
  double descriptor_radius = 80.0, sensor_height = 2.0, descriptor_threshold = 0.2;
  int gicp_iterations = 64, min_inliers = 30;
  double correspondence_distance = 2.0, min_inlier_ratio = 0.3, max_mean_square_error = 0.25;
  double odom_rotation_sigma = 0.01, odom_translation_sigma = 0.03;
  double loop_rotation_sigma = 0.1, loop_translation_sigma = 0.2;
  double relinearize_threshold = 0.01;
  int relinearize_skip = 1;
  void validate() const;
};

struct Descriptor {
  Eigen::MatrixXf values;
  Eigen::VectorXf ring_key;
};
struct Candidate { std::size_t id = 0; double distance = 1, yaw = 0; };
class ScanContext {
 public:
  explicit ScanContext(const Config& config);
  Descriptor describe(const Cloud& cloud) const;
  Candidate compare(const Descriptor& query, const Descriptor& target) const;
 private:
  Config cfg_;
};

struct Alignment {
  bool accepted = false;
  Pose target_from_source = Pose::Identity();
  std::size_t inliers = 0;
  double inlier_ratio = 0, mean_square_error = 0;
  std::string reason;
};
Alignment verifyGicp(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                     const Pose& initial, const Config& cfg);

class PoseGraph {
 public:
  explicit PoseGraph(const Config& config);
  ~PoseGraph();
  void addOdometry(std::size_t id, const Pose& original);
  void addLoop(std::size_t target, std::size_t source, const Pose& target_from_source);
  Pose estimate(std::size_t id) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Keyframe {
  std::size_t id = 0;
  double stamp = 0;
  Pose original = Pose::Identity(), optimized = Pose::Identity();
  Cloud::ConstPtr cloud;
};
struct Snapshot {
  std::vector<Keyframe> keyframes;
  Pose map_from_odom = Pose::Identity();
  std::size_t accepted_loops = 0, rejected_loops = 0, dropped_frames = 0;
  std::string status = "waiting-for-input";
};

class Backend {
 public:
  explicit Backend(const Config& config);
  ~Backend();
  Backend(const Backend&) = delete;
  Backend& operator=(const Backend&) = delete;
  bool submit(double stamp, const Pose& original, Cloud::ConstPtr local_cloud);
  std::shared_ptr<const Snapshot> snapshot() const;
  void stop();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
Cloud::Ptr transformCloud(const Cloud& cloud, const Pose& pose);
Cloud::Ptr downsample(const Cloud::ConstPtr& cloud, double voxel);
bool validPose(const Pose& pose);
} // namespace slam::graph
