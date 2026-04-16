// SPDX-License-Identifier: GPL-3.0-or-later
//
// OctVoxMap 适配器：让 slam_octvox 里 vendored 的 Super-LIO OctVoxMap
// 实现 slam::map::IMapManager + slam::nn::INeighborSearch。
//
// 语义阻抗说明
// ------------
//   - Super-LIO OctVoxMap 的点类型是 Eigen::Vector3f（只 xyz），
//     我们的接口点类型是 pcl::PointXYZINormal。插入时投射掉 intensity/
//     normal；knn 回填时 intensity/normal 置 0（平面拟合只用 xyz，不受
//     影响）。
//   - Super-LIO 无 box-delete / drainRemoved / FoV crop 语义——LRU 负责
//     地图体积控制。本适配器里这些方法为 no-op，对齐 Super-LIO 行为；
//     A/B 实验时 FAST-LIO2 的 cub_needrm 代价会被"吞掉"。
//   - KNN K 固定为 5（Super-LIO `KNNHeap<5, Point>`），我们接口允许 k<=8，
//     这里对 k>5 的请求仍只返回 5 个并在日志里标注一次（rate-limited）。
#pragma once

#include <memory>
#include <mutex>

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
  bool        empty()     const override;
  std::size_t size()      const override;
  std::size_t validSize() const override;
  MapStats    stats()     const override;
  const char* name()      const override { return "octvox"; }

  // INeighborSearch
  int  knn(const nn::Query& q, std::array<Neighbor, 8>& out) const override;
  bool fitPlane(const nn::Query& q, PlaneCoef& out) const override;

 private:
  std::unique_ptr<BackendT>           backend_;
  MapConfig                           cfg_{};
  mutable MapStats                    stats_{};
  mutable std::mutex                  stats_mu_;
};

// 工厂
std::unique_ptr<IMapManager> makeOctVoxMap();

}  // namespace slam::map
