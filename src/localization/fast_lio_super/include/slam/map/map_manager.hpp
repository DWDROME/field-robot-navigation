// SPDX-License-Identifier: BSD-2-Clause
// IMapManager：显式区分 Active Map（进残差）与 Persistent Map（落盘/重定位/压缩）。
// 阶段 A 时两者可为同一指针；阶段 B 起 active=OctVox, persistent=OctVox(带 LRU)。
#pragma once

#include <cstddef>
#include <memory>

#include "slam/common_types.hpp"
#include "slam/nn/neighbor_search.hpp"

namespace slam::map {

struct MapConfig {
  float  voxel_size       = 0.5f;    // Active / Persistent 子体素边长
  int    max_pts_per_vox  = 8;       // OctVox 子体素代表点上限；ikdtree 后端忽略
  std::size_t capacity    = 2'000'000;  // 触发 LRU 淘汰的 voxel 数上限
  bool   enable_lru       = true;    // ikdtree 后端目前忽略
  bool   incremental_mean = true;    // 代表点的增量均值更新；ikdtree 后端忽略
  float  local_crop_radius = 0.0f;   // >0 时 cropAround 生效
  std::size_t raw_capacity = 200'000; // OctVox 原始点的独立保留预算
  std::size_t removed_capacity = 10'000; // 删除回取缓存上限
};

struct MapStats {
  std::size_t num_voxels   = 0;
  std::size_t num_points   = 0;
  std::size_t bytes        = 0;
  double      last_insert_ms = 0.0;
  double      last_query_ms  = 0.0;
  std::size_t raw_points = 0;
  std::size_t removed_dropped = 0; // 未及时回取时超过缓存预算的记录数
};

class IMapManager {
 public:
  virtual ~IMapManager() = default;

  virtual void configure(const MapConfig& cfg) = 0;

  // IESKF 每帧：去畸变后的世界系点云，地图负责下采样/去重/合并。
  // insert          : 后端决定是否 downsample（ikdtree 后端默认启用）
  // insertNoDownsample : 直接插入，不做 voxel 过滤（对应 FAST-LIO2 的
  //                      "PointNoNeedDownsample" 路径）
  virtual void insert(const PointVec& world_pts) = 0;
  virtual void insertNoDownsample(const PointVec& world_pts) = 0;

  // 暴露邻居搜索；同一 MapManager 可挂不同 NeighborSearch 实现。
  virtual nn::INeighborSearch& search() = 0;

  // 导出当前地图，便于可视化/落盘；voxel<=0 表示不额外下采样。
  virtual CloudPtr snapshot(float voxel = 0.0f) const = 0;

  // 把内部代表点扁平化写入 `out`（追加模式）。Phase A ikdtree 后端对应
  // `flatten(..., NOT_RECORD)`；Phase B OctVox 后端输出 voxel 代表点。
  virtual void flattenInto(PointVec& out) const = 0;

  // 按一组轴对齐盒子删除命中点；返回实际删除数量。对应 FAST-LIO2 的
  // FoV segmentation + `Delete_Point_Boxes`。
  virtual int removeBoxes(const std::vector<BoundingBox>& boxes) = 0;

  // 以 c 为中心、radius 为半径裁剪地图，防止长时运行地图无界增长。
  virtual void cropAround(const Eigen::Vector3d& c, float radius) = 0;

  // 排空"上次操作后被标记删除"的点（对应 `acquire_removed_points`）。
  virtual void drainRemoved(PointVec& out) = 0;

  virtual bool        empty() const = 0;
  virtual std::size_t size()      const = 0;  // 总节点数（含 tombstone）
  virtual std::size_t validSize() const = 0;  // 有效点数

  virtual MapStats    stats() const = 0;
  virtual const char* name()  const = 0;   // "ikdtree" / "octvox" ...
};

// 工厂入口
std::unique_ptr<IMapManager> makeIkdTreeMap();   // Phase A，封装 ikd-tree
std::unique_ptr<IMapManager> makeOctVoxMap();    // Phase B-1，封装 Super-LIO OctVoxMap（GPL）

}  // namespace slam::map
