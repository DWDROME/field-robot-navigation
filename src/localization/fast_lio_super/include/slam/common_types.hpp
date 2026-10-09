// SPDX-License-Identifier: BSD-2-Clause
// slam_core common types — 跨 MapManager / NeighborSearch / Relocalizer 共用。
// 此文件不依赖 ROS，仅依赖 Eigen / PCL。
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam {

// 与 FAST-LIO2 保持一致的点类型（见 common_lib.h::PointType）
using PointT   = pcl::PointXYZINormal;
using CloudT   = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

// 原 FAST-LIO2 与 ikd-tree 约定的 aligned 点向量，用于避免 CloudT <-> vector
// 之间的无谓拷贝（KD_TREE API / pcl::PointCloud::points 都用这个类型）。
using PointVec = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

// 轴对齐包围盒，用于 removeBoxes（取代 ikd-tree 内部的 BoxPointType）。
struct BoundingBox {
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();
};

struct Pose6D {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double          stamp = 0.0;  // sec
};

// 平面拟合结果，由 INeighborSearch::fitPlane 产出
struct PlaneCoef {
  Eigen::Vector4f abcd       = Eigen::Vector4f::Zero(); // ax+by+cz+d=0, |n|=1
  float           curvature  = 0.0f;
  bool            valid      = false;
};

// 邻居查询结果元素
struct Neighbor {
  PointT   p{};
  float    dist2     = 0.0f;
  uint32_t voxel_id  = 0;  // 0 表示未启用 voxel 索引（如 ikd-tree 后端）
};

// NUM_MATCH_POINTS 与 common_lib.h 保持一致（5）
constexpr int kDefaultKnn = 5;

}  // namespace slam
