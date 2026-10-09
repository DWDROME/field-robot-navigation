// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <cmath>

namespace slam::graph {
namespace {
class CheckedGicp : public pcl::GeneralizedIterativeClosestPoint<Point,Point> {
 public:
  int iterations() const { return nr_iterations_; }
};
}
Cloud::Ptr transformCloud(const Cloud& cloud, const Pose& pose) {
  Cloud::Ptr output(new Cloud);
  pcl::transformPointCloud(cloud, *output, pose.matrix().cast<float>());
  return output;
}
Cloud::Ptr downsample(const Cloud::ConstPtr& cloud, double voxel) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(voxel, voxel, voxel);
  filter.filter(*output);
  return output;
}

Alignment verifyGicp(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                     const Pose& initial, const Config& cfg) {
  Alignment result;
  cfg.validate();
  if (!source || !target || !validPose(initial) ||
      source->size() < static_cast<std::size_t>(std::max(cfg.min_inliers,20)) ||
      target->size() < static_cast<std::size_t>(std::max(cfg.min_inliers,20))) {
    result.reason = "invalid-pose-or-insufficient-points"; return result;
  }
  for (const auto* cloud : {source.get(), target.get()})
    for (const auto& p : *cloud)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        result.reason = "non-finite-point"; return result;
      }
  try {
    CheckedGicp registration;
    registration.setMaximumIterations(cfg.gicp_iterations);
    registration.setMaxCorrespondenceDistance(cfg.correspondence_distance);
    registration.setTransformationEpsilon(1e-6);
    registration.setInputSource(source);
    registration.setInputTarget(target);
    Cloud aligned;
    registration.align(aligned, initial.matrix().cast<float>());
    if (!registration.hasConverged()) { result.reason = "not-converged"; return result; }
    // PCL also reports hasConverged at the iteration limit. Conservatively
    // reject that stop condition instead of turning it into a loop factor.
    if (registration.iterations() >= cfg.gicp_iterations) {
      result.reason = "iteration-limit"; return result;
    }
    result.target_from_source.matrix() = registration.getFinalTransformation().cast<double>();
    if (!validPose(result.target_from_source)) { result.reason = "invalid-transform"; return result; }
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    double error = 0;
    for (const auto& p : aligned) {
      if (tree.nearestKSearch(p,1,indices,distances) == 1 &&
          distances[0] <= cfg.correspondence_distance*cfg.correspondence_distance) {
        ++result.inliers;
        error += distances[0];
      }
    }
    result.inlier_ratio = static_cast<double>(result.inliers)/source->size();
    result.mean_square_error = result.inliers ? error/result.inliers : INFINITY;
    if (result.inliers < static_cast<std::size_t>(cfg.min_inliers)) result.reason = "insufficient-inliers";
    else if (result.inlier_ratio < cfg.min_inlier_ratio) result.reason = "insufficient-overlap";
    else if (!std::isfinite(result.mean_square_error) ||
             result.mean_square_error > cfg.max_mean_square_error) result.reason = "excessive-residual";
    else { result.accepted = true; result.reason = "accepted"; }
  } catch (const std::exception& e) { result.reason = std::string("gicp-error: ") + e.what(); }
  return result;
}
} // namespace slam::graph
