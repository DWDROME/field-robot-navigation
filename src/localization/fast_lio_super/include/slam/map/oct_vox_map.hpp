// SPDX-License-Identifier: GPL-3.0-or-later
//
// OctVoxMap 适配器：让 slam_octvox 里 vendored 的 Super-LIO OctVoxMap
// 实现 slam::map::IMapManager + slam::nn::INeighborSearch。
//
// 地图承载 xyz；其余 PCL 字段导出为零。原始 xyz 与代表点分别保留。
// 删除、空间裁剪与容量淘汰共用索引和有界删除回取缓存。
//
// 并发模型（B-1.c）
// ----------------
//   - knn()  const，热路径零锁；OctVoxMap::getTopK 本身对 const map 是
//     线程安全的（只读 grids_/data_，本地 KNNHeap 输出）
//   - insert() 不加锁；调用方应保证同一 map 不被多 writer 并发调用
//     （FAST-LIO2 里 insert 只在主线程 UpdateMap 中调用，天然串行）
//   - 计时用 std::atomic<uint64_t> ns，OMP 并行下允许 race
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "slam/common_types.hpp"
#include "slam/map/map_manager.hpp"
#include "slam/nn/neighbor_search.hpp"

#include "slam_octvox/OctVoxMap.hpp"   // GPLv3 header-only, from slam_octvox pkg

namespace slam::map {

class OctVoxMapAdapter final : public IMapManager, public nn::INeighborSearch {
 public:
  using V3       = Eigen::Vector3f;
  using BackendT = LI2Sup::OctVoxMap<V3, float>;
  using HeapT    = typename BackendT::KNNHeapType;

  OctVoxMapAdapter();
  ~OctVoxMapAdapter() override;

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
  bool        empty()     const override { return backend_->point_count() == 0; }
  std::size_t size()      const override { return backend_->point_count(); }
  std::size_t validSize() const override { return backend_->point_count(); }
  MapStats    stats()     const override;
  const char* name()      const override { return "octvox"; }

  // INeighborSearch
  int  knn(const nn::Query& q, std::array<Neighbor, 8>& out) const override;
  bool fitPlane(const nn::Query& q, PlaneCoef& out) const override;

 private:
  std::unique_ptr<BackendT>     backend_;
  MapConfig                     cfg_{};
  mutable std::atomic<uint64_t> last_query_ns_{0};
  mutable std::atomic<uint64_t> last_insert_ns_{0};
};

// 工厂
std::unique_ptr<IMapManager> makeOctVoxMap();

}  // namespace slam::map
