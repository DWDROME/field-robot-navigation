// SPDX-License-Identifier: BSD-2-Clause
// IkdTreeMap：IMapManager + INeighborSearch 的 ikd-Tree 后端，用于 Phase A baseline。
// 把 FAST-LIO2 原先直接对 KD_TREE<PointType> 的调用收敛到 slam_core 接口，
// 让后续 OctVoxMap / HknnSearch 可以在不动 IESKF 代码的前提下替换。
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <deque>

#include "ikd-Tree/ikd_Tree.h"
#include "slam/common_types.hpp"
#include "slam/map/map_manager.hpp"
#include "slam/nn/neighbor_search.hpp"

namespace slam::map {

class IkdTreeMap final : public IMapManager, public nn::INeighborSearch {
 public:
  IkdTreeMap();
  ~IkdTreeMap() override;

  // IMapManager
  void configure(const MapConfig& cfg) override;
  void insert(const PointVec& world_pts) override;
  void insertNoDownsample(const PointVec& world_pts) override;
  nn::INeighborSearch& search() override { return *this; }
  CloudPtr snapshot(float voxel = 0.0f) const override;
  void flattenInto(PointVec& out) const override;
  int  removeBoxes(const std::vector<BoundingBox>& boxes) override;
  void cropAround(const Eigen::Vector3d& c, float radius) override;
  void drainRemoved(PointVec& out) override;
  bool        empty()     const override;
  std::size_t size()      const override;
  std::size_t validSize() const override;
  MapStats    stats()     const override;
  const char* name()      const override { return "ikdtree"; }

  // INeighborSearch
  int  knn(const nn::Query& q, std::array<Neighbor, 8>& out) const override;
  bool fitPlane(const nn::Query& q, PlaneCoef& out) const override;

 private:
  void recordRemoved(const PointVec& points);
  std::deque<PointT, Eigen::aligned_allocator<PointT>> removed_;
  std::size_t removed_dropped_ = 0;
  // KD_TREE 的一些操作是非 const 的（Nearest_Search 会动锁），
  // 故用 mutable 包装以满足 INeighborSearch 的 const 语义。
  // ikd-Tree 内部本身线程安全（支持并发 Nearest_Search），所以 knn()
  // 路径不再加额外的锁——B-1.c 移除了原先的 stats_mu_。
  // 计时字段用 atomic 避免 OMP 并行下的 write-torn。
  mutable KD_TREE<PointT>    tree_;
  MapConfig                  cfg_{};
  mutable std::atomic<uint64_t> last_query_ns_{0};
  mutable std::atomic<uint64_t> last_insert_ns_{0};
};

}  // namespace slam::map
