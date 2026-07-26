# slam_octvox — Vendored Super-LIO OctVoxMap + HKNN

**归属项目**: 项目 A — FAST-LIO Super SLAM
**标签**: `GPLv3` `Header-only` `C++17` `OctVox` `HKNN` `Super-LIO`

---

## 技术核心

从 Super-LIO 源码提取的紧凑地图层：8-subvoxel 空间量化 + running-mean 点累积 + LRU 淘汰 + 6 组分段 HKNN 分层搜索 + tsl::robin_map 哈希索引。

### Vendored 文件清单

| 文件 | 行数 | 来源许可 | 修改 |
|------|------|----------|------|
| `include/slam_octvox/OctVoxMap.hpp` | ∼1200 | GPLv3 | 删 `<execution>` / `<filesystem>` / `saveMap()` |
| `include/slam_octvox/HKNN_list60_gem.h` | 593 | GPLv3 | 无 |
| `include/tsl/robin_map.h` | ∼800 | MIT | 无 |
| `include/tsl/robin_growth_policy.h` | ∼300 | MIT | 无 |

### OctVox 空间量化

```cpp
// 输入: pt (Eigen::Vector3f, 地图坐标系)
// 输出: key (父 voxel 坐标), local_idx (八分体索引 0-7)
fine_key = floor(pt * (2.0f / resolution));
key = fine_key >> 1;
local_idx = ((fine_key.z() & 1) << 2) |
            ((fine_key.y() & 1) << 1) |
            ((fine_key.x() & 1));

// 参数
constexpr float kResolution = 0.2f;           // voxel 边长 (m)
constexpr float kSubVoxelRes = 0.1f;          // sub-voxel 边长 (m)
constexpr int   kMaxPointsPerSubVoxel = 20;   // 每 octant 最大累积帧数
constexpr float kDistThreshSq = 0.01f;        // 0.1m² — 新点与均值距离阈值
```

### OctVox::AddPoint（子体素插入）

```cpp
bool OctVox::AddPoint(const Eigen::Vector3f& pt, int local_idx) {
  auto& c = counts_[local_idx];
  auto& p = points_[local_idx];

  if (c == 0) {
    p = pt;              // 初始化均值
    c = 1;
    return true;
  }
  if (c >= kMaxPointsPerSubVoxel)
    return false;         // 已达容量上限

  float d2 = (pt - p).squaredNorm();
  if (d2 > kDistThreshSq)
    return false;         // 与已有均值距离超标 → 拒绝

  p = (p * static_cast<float>(c) + pt) / static_cast<float>(c + 1);  // running mean
  c++;
  return true;
}
```

### LRU 淘汰

```cpp
// 容器
std::list<std::pair<Key, OctVox>> lru_list_;
tsl::robin_map<Key, decltype(lru_list_)::iterator> lru_map_;

// 插入新 voxel
void touch(const Key& key) {
  lru_list_.emplace_front(key, OctVox());
  lru_map_[key] = lru_list_.begin();
  if (lru_list_.size() > kCapacity) {
    lru_map_.erase(lru_list_.back().first);
    lru_list_.pop_back();
  }
}

// 命中 voxel（AddPoint 时调用）
void hit(const Key& key) {
  auto it = lru_map_[key];
  lru_list_.splice(lru_list_.begin(), lru_list_, it);  // 搬到头部
}
```

`resetMap(cloud)` 后 10 帧 skip insert（`reset_map_count_ = 10`）。

### HKNN 6 组分段搜索

搜索表 `flat_search_order[593]` 预计算了 `(voxel_offset_id, local_k, pattern)` 三元组：

```cpp
// 6 组搜索的分段偏移
const int flat_search_order_offsets[7] = {0, 43, 135, 219, 321, 465, 593};

// 输入: query_pt (Eigen::Vector3f)
// 输出: KNNHeap<5, Point> (K=5, 硬编码)

for (int group = 0; group < 6; group++) {
  for (int i = flat_search_order_offsets[group];
       i < flat_search_order_offsets[group + 1]; i++) {
    // 读取预计算条目
    int32_t code = flat_search_order[i];
    int neighbor_idx = (code >> 24) & 0xFF;    // 邻居 voxel 编号
    int k            = (code >> 16) & 0xFF;     // 局部搜索模式
    int pattern      = code & 0xFFFF;           // local_idx × k 交错

    // Mirror 优化
    Eigen::Vector3i mirror_axis(1 - 2*dx, 1 - 2*dy, 1 - 2*dz);
    Key n_key = query_key + mirror_axis * HKNN_neighbor_voxel[neighbor_idx];

    // XOR 优化
    int local_idx = pattern ^ query_local_idx;

    // Top-8 缓存: Group0 的 voxel 结果存储复用
  }

  // 早停判断
  if (heap.count() == 5 && heap.max_dist2() < kOrdersMinDist2[group])
    break;
}

// 早停平方距离阈值 (m²)
const float kOrdersMinDist2[6] = {0.0625f, 0.125f, 0.25f, 0.3125f, 0.375f, 100.0f};
```

### 适配器层（FAST_LIO_SUPER 中）

`OctVoxMapAdapter`（`src/slam/map/oct_vox_map.cpp`）同时实现 `IMapManager` + `INeighborSearch`：

```cpp
class OctVoxMapAdapter : public slam::map::IMapManager,
                         public slam::nn::INeighborSearch {
  OctVoxMap<Eigen::Vector3f, float> map_;  // 模板参数: Point=V3, scalar=float

  // IMapManager
  void insert(const PointVec& pts) override {
    for (auto& p : pts) map_.AddPoint(p.getVector3fMap());
  }
  void removeBoxes(const std::vector<BoundingBox>&) override { /* no-op */ }
  void cropAround(const Eigen::Vector3d&, float) override { /* no-op */ }

  // INeighborSearch
  int knn(const Query& q, std::array<Neighbor, 8>& out) const override {
    HKNN_Heap_t heap;
    map_.getTopK(q.center, heap);              // Super-LIO 原生 HKNN
    for (int i = 0; i < heap.count(); i++)
      out[i] = {heap[i].point, heap[i].dist2, 0};
    return heap.count();
  }
};
```

### 锁优化（Phase B-1.c）

移除了 HKNN 热路径中的 `std::mutex`：
- `knn()` 调用链中原本有 `std::lock_guard<std::mutex>` 保护 LRU list 和 hash map
- 分析确认：查询是纯读操作（仅有 `const` 方法被 `INeighborSearch::knn` 暴露），不存在写竞争
- 将 LRU 的写操作（`hit()` 中的 `splice`）转移到 insert 路径中，查询路径实现 lock-free
- OMP 调优：从编译期 `#define MP_PROC_NUM 3` 改为运行时 `omp_set_num_threads(n)`
