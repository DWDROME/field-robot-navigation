// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slam::graph {
void Config::validate() const {
  const double positive[] = {keyframe_translation, keyframe_rotation, voxel,
    candidate_period, descriptor_radius, correspondence_distance,
    odom_rotation_sigma, odom_translation_sigma, loop_rotation_sigma,
    loop_translation_sigma, relinearize_threshold};
  for (double n : positive)
    if (!std::isfinite(n) || n <= 0) throw std::invalid_argument("invalid positive SLAM parameter");
  if (!std::isfinite(min_loop_seconds) || min_loop_seconds < 0 ||
      !std::isfinite(sensor_height) || min_loop_keyframes < 1 || submap_frames < 0 ||
      rings < 2 || sectors < 4 || rings > 100 || sectors > 360 || candidate_count < 1 ||
      candidate_count > 100 || max_keyframes < 2 || max_frame_points < 30 ||
      max_cloud_points < max_frame_points || queue_capacity == 0 ||
      queue_capacity > 100 || gicp_iterations < 1 || min_inliers < 3 ||
      relinearize_skip < 1 || !std::isfinite(descriptor_threshold) || descriptor_threshold <= 0 || descriptor_threshold >= 1 ||
      !std::isfinite(min_inlier_ratio) || min_inlier_ratio <= 0 || min_inlier_ratio > 1 ||
      !std::isfinite(max_mean_square_error) || max_mean_square_error <= 0)
    throw std::invalid_argument("invalid SLAM resource or loop-validation parameter");
}

bool validPose(const Pose& pose) {
  return pose.matrix().allFinite() &&
    (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
    std::fabs(pose.linear().determinant() - 1) < 1e-3 &&
    (pose.matrix().row(3) - Eigen::RowVector4d(0,0,0,1)).norm() < 1e-6;
}

ScanContext::ScanContext(const Config& config) : cfg_(config) { cfg_.validate(); }
Descriptor ScanContext::describe(const Cloud& cloud) const {
  Descriptor result;
  result.values = Eigen::MatrixXf::Zero(cfg_.rings, cfg_.sectors);
  constexpr double tau = 6.283185307179586;
  for (const auto& p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double radius = std::hypot(p.x, p.y);
    if (radius > cfg_.descriptor_radius || radius <= 0) continue;
    double angle = std::atan2(p.y,p.x);
    if (angle < 0) angle += tau;
    const int ring = std::min(cfg_.rings-1, static_cast<int>(radius/cfg_.descriptor_radius*cfg_.rings));
    const int sector = std::min(cfg_.sectors-1, static_cast<int>(angle/tau*cfg_.sectors));
    result.values(ring,sector) = std::max(result.values(ring,sector),
      static_cast<float>(std::max(0.0, p.z + cfg_.sensor_height)));
  }
  result.ring_key = result.values.rowwise().mean();
  return result;
}

Candidate ScanContext::compare(const Descriptor& query, const Descriptor& target) const {
  Candidate best;
  if (query.values.rows() != cfg_.rings || target.values.rows() != cfg_.rings ||
      query.values.cols() != cfg_.sectors || target.values.cols() != cfg_.sectors)
    throw std::invalid_argument("Scan Context dimensions differ from configuration");
  for (int shift = 0; shift < cfg_.sectors; ++shift) {
    double sum = 0;
    int usable = 0;
    for (int col = 0; col < cfg_.sectors; ++col) {
      const auto q = query.values.col((col+shift)%cfg_.sectors);
      const auto t = target.values.col(col);
      const double norm = q.norm() * t.norm();
      if (norm <= 1e-6) continue;
      sum += std::clamp(static_cast<double>(q.dot(t))/norm, -1.0, 1.0);
      ++usable;
    }
    const double distance = usable >= 3 ? 1 - sum/usable : 1;
    if (distance < best.distance) {
      best.distance = distance;
      best.yaw = -shift * 6.283185307179586 / cfg_.sectors;
    }
  }
  return best;
}
} // namespace slam::graph
