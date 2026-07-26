# FAST_LIO_SUPER — 可插拔后端 SLAM 引擎

**归属项目**: 项目 A — FAST-LIO Super SLAM
**标签**: `C++17` `SLAM` `FAST-LIO2` `Super-LIO` `IMapManager` `OctVoxMap` `HKNN` `双后端`

---

## 一句话

将 FAST-LIO2（IESKF + ikd-tree 紧耦合前端）中的 `ikdtree` 全局对象解耦为 `IMapManager`/`INeighborSearch` 接口层，以 Super-LIO 的 OctVoxMap（8-subvoxel 空间量化 + running-mean + LRU 淘汰 + HKNN 6 组分段搜索）作为第二个可运行时切换的后端实现。

---

## 1. 两阶段工程实施

### Phase A — IMapManager/INeighborSearch 接口抽象 + IkdTreeMap 封装（✅）

#### 1.1 接口设计

在 `include/slam/` 下定义：

**`include/slam/map/map_manager.hpp`** — 地图全生命周期接口：

```cpp
namespace slam::map {

struct MapConfig {
  float filter_size_surf = 0.5;     // 降采样体素尺寸 (m)
  int    max_iteration = 3;         // IESKF 最大迭代
  // ...FAST-LIO2 原参数全部外化
};

struct MapStats {
  std::size_t total_points = 0;
  std::size_t valid_points = 0;
  int    backend_type = 0;          // 0=ikdtree, 1=octvox
  double build_time_ms = 0.0;
};

class IMapManager {
 public:
  virtual ~IMapManager() = default;
  virtual void                  configure(const MapConfig&) = 0;
  virtual void                  insert(const PointVec& world_pts) = 0;           // 降采样插入
  virtual void                  insertNoDownsample(const PointVec& world_pts) = 0; // 原始插入
  virtual nn::INeighborSearch&  search() = 0;                                      // 绑定的搜索器
  virtual CloudPtr              snapshot(float voxel = 0.0f) const = 0;            // 降采样导出
  virtual void                  flattenInto(PointVec& out) const = 0;              // 全量展平
  virtual int                   removeBoxes(const std::vector<BoundingBox>&) = 0;  // 盒删除
  virtual void                  cropAround(const Eigen::Vector3d&, float) = 0;     // 空间裁剪
  virtual void                  drainRemoved(PointVec& out) = 0;                   // 取被删点
  virtual bool                  empty() const = 0;
  virtual std::size_t           size() const = 0;
  virtual std::size_t           validSize() const = 0;
  virtual MapStats              stats() const = 0;
  virtual const char*           name() const = 0;
};

} // namespace slam::map
```

**`include/slam/nn/neighbor_search.hpp`** — KNN 搜索接口：

```cpp
namespace slam::nn {

struct Query {
  Eigen::Vector3f center;   // 地图坐标系下的查询位置
  int   k = 5;              // 最大邻居数（≤8）
  float max_dist2 = 1.0f;   // 最大平方距离阈值
};

struct Neighbor {
  PointT     p;
  float      dist2;         // 平方距离 (m²)
  uint32_t   voxel_id = 0;  // 后端可选的 voxel ID（ikd-tree 无此信息，设为 0）
};

class INeighborSearch {
 public:
  virtual ~INeighborSearch() = default;
  // knn：返回实际找到的邻居数 n (≤q.k)，out[0..n-1] 按 dist2 升序
  virtual int  knn(const Query& q, std::array<Neighbor, 8>& out) const = 0;
  // fitPlane：拟合平面，残差 > 0.1m 返回 valid=false
  virtual bool fitPlane(const Query& q, PlaneCoef& out) const = 0;
  virtual const char* name() const = 0;  // "ikdtree" / "hknn"
};

} // namespace slam::nn
```

#### 1.2 IkdTreeMap 后端

`include/slam/map/ikd_tree_map.hpp` + `src/slam/map/ikd_tree_map.cpp`

将原 `ikd-Tree/ikd_Tree.cpp` 的 20+ 个 public 方法映射到 IMapManager 的 13 个虚函数：

| `IkdTreeMap` 内部调用 | 长度 (行) | 复杂度 |
|---|---|---|
| `Add_Points(pts, downsample)` | ∼80 | O(n log k) |
| `Delete_Point_Boxes(box, remove_self)` | ∼120 | O(k log n) |
| `acquire_removed_points(out)` | ∼30 | O(r) |
| `flatten(out)` | ∼20 | O(n) |
| `Build()` | ∼200 | O(n log n) |

#### 1.3 laserMapping.cpp 迁移清单

全局 `ikdtree` 对象被完全删除，替换为 `slamMap()` 单例。

```cpp
// === 迁移前后对照（原行号对应） ===

// 插入
// 原: ikdtree.Add_Points(point_wait_insert, false);
slamMap().insertNoDownsample(point_wait_insert);

// 带降采样插入
// 原: ikdtree.Add_Points(point_wait_insert, true);
slamMap().insert(point_wait_insert);

// 盒状删除 (BoxPointType → BoundingBox 转换)
// 原: ikdtree.Delete_Point_Boxes(Box, true);
BoxPointType box; ...
std::vector<BoundingBox> boxes;
BoundingBox bb;
bb.min = box.vertex_min.template cast<float>();
bb.max = box.vertex_max.template cast<float>();
boxes.push_back(bb);
slamMap().removeBoxes(boxes);

// 取出被惰性删除的点
// 原: ikdtree.acquire_removed_points(remove_points);
slamMap().drainRemoved(remove_points);

// 全量导出
// 原: ikdtree.flatten(laserCloudMapArray);
slamMap().flattenInto(laserCloudMapArray);

// 判空
// 原: if (ikdtree.Root_Node == nullptr)
if (slamMap().empty())

// 首次插入触发 Build
// 原: ikdtree.Build();
// 迁移后：首次 slamMap().insert(...) 内部检查并调用 Build()

// 有效点数
// 原: ikdtree.validnum
slamMap().validSize()

// 总点数
// 原: ikdtree.size()
slamMap().size()

// KNN 搜索（IESKF 残差计算入口）
// 原: Nearest_Search(point, 5, ...)
auto& nn = slamMap().search();
slam::nn::Query q{point, 5, search_max_dist2};
std::array<slam::nn::Neighbor, 8> nb;
int cnt = nn.knn(q, nb);
```

#### 1.4 编译与构建

```cmake
# CMakeLists.txt (fast_lio_super)
add_library(fast_lio_super_slam STATIC
  src/slam/map/ikd_tree_map.cpp
  src/slam/map/oct_vox_map.cpp
)
add_executable(fastlio_super_mapping
  src/laserMapping.cpp
  src/preprocess.cpp
)
target_link_libraries(fastlio_super_mapping
  fast_lio_super_slam
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
```

### Phase B-1 — OctVox 后端（✅）

#### 2.1 slam_octvox 包（新建 header-only catkin 包）

从 Super-LIO（ros1 分支，commit 锁定）提取的核心地图层：

| Vendored 文件 | 源路径 | 修改 |
|---|---|---|
| `slam_octvox/include/slam_octvox/OctVoxMap.hpp` | `Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp` | 删 `<execution>` / `<filesystem>` / `saveMap()` ~60 行 |
| `slam_octvox/include/slam_octvox/HKNN_list60_gem.h` | 同上目录 | 无（593 行预计算表） |
| `slam_octvox/include/tsl/robin_map.h` | `Super-LIO/.../tsl/` | 无，MIT 许可 |
| `slam_octvox/include/tsl/robin_growth_policy.h` | 同上 | 无，MIT 许可 |

#### 2.2 OctVox 空间量化核心

```cpp
// 量化公式（来自 OctVoxMap.hpp, line ~200）
fine_key = floor(pt * (2.0f / resolution));
key      = fine_key >> 1;
local_idx = ((fine_key.z() & 1) << 2) |
            ((fine_key.y() & 1) << 1) |
            ((fine_key.x() & 1));       // 八分体索引 0..7
```

- `resolution` = 0.2m（默认 sub-voxel = 0.1m）
- 每 voxel 持 8 个 octant 的 `counts_[8]` + `points_[8]`（running mean）
- `MAX_POINTS_PER_SUBVOXEL = 20`：每个 octant 最多积累 20 帧贡献，此后拒绝新点
- `DISTANCE_THRESHOLD_SQ = 0.01`：新点与当前 stored 均值距离 >0.1m 则拒绝（保护局部结构）
- 更新：`stored = (stored * count + pt) / (count + 1)`（running mean，非 Welford）

#### 2.3 LRU 淘汰（OctVoxMap 内存管理）

```cpp
// 数据结构
std::list<pair<KEY, OctVox>> data_;          // 双向链表，新/命中在前
tsl::robin_map<KEY, decltype(data_)::iterator> grids_;  // 哈希索引

// 插入新 voxel
data_.emplace_front(key, OctVox());
grids_[key] = data_.begin();
if (data_.size() > capacity_) {
  grids_.erase(data_.back().first);
  data_.pop_back();
}   // 容量超限 → 淘汰末尾

// 命中 voxel（AddPoint 时）
data_.splice(data_.begin(), data_, iter);   // 搬到头部
```

- `resetMap(cloud)` 后设 `reset_map_count_ = 10`：接下来 10 帧 insert() skip，给预建图留稳定时间

#### 2.4 HKNN 分层搜索

输入 `Eigen::Vector3f`，输出 `KNNHeap<5, Point>`（K 硬编码 5，最差堆）：

```cpp
// 搜索表分段
flat_search_order_offsets[7] = {0, 43, 135, 219, 321, 465, 593};
// 共 593 条预计算条目，(neighbor_voxel_idx, k, local_idx × k) 交错

// Group 0：当前 voxel 的 8 个 octant（条 0-42）
// Group 1-5：HKNN_neighbor_voxel[60] 外围 voxel，逐组扩展
```

**优化技术**：

```
Mirror 优化: mirror_axis = (1-2dx, 1-2dy, 1-2dz)
             n_key = key + mirror_axis * HKNN_neighbor_voxel[i]
             → 按查询 octant 对称翻转 offset

XOR 优化:   _local_idx = table_local_idx ^ local_idx
             → 把预表里的相对 local_idx 转为绝对 octant 索引

Top-8 缓存: Group0 的 8 个 octant 邻居只 hash lookup 一次
            存 top_voxels_2_search[8]，后续组若 neighbor_idx < 8 直接复用
```

**早停**（每组结束后检查）：

```cpp
// 平方距离阈值 (m²)
orders_min_dis2[6] = {0.0625, 0.125, 0.25, 0.3125, 0.375, 100.0f};

if (top_K.count == 5 && top_K.max_dist2_ < orders_min_dis2[group_idx])
  break;   // 已找到 5 个足够近的邻居，停止搜索
```

### Phase B-1.c — 无锁 HKNN + 运行时 OMP 调优（✅）

- **锁移除**：HKNN 热路径 `knn()` 中删除跨帧 `std::mutex`（查询无写竞争，只需要 `std::atomic` 保证 visibility）
- **OMP 调优**：运行时通过 `omp_set_num_threads()` 调整，不再编译期 `#define MP_PROC_NUM`

### Phase B-2 ~ C 待实现

```
Phase B-2: Relocalizer     (接口已预留 include/slam/reloc/)
Phase B-3: MapIO           (接口已预留 include/slam/io/)
Phase C-1: Keyframe + iSAM2 sidecar   (骨架图纸已完成)
Phase C-2: ILoopCandidateProvider 插件化
Phase C-3: MapCorrectionBridge
```

---

## 2. 配置

```yaml
# config/mid360.yaml
slam_core:
  map_backend: "octvox"           # "ikdtree" | "octvox"
  filter_size_surf: 0.5           # 降采样体素 (m)
  max_iteration: 3                # IESKF 迭代次数

# 以下与原 FAST-LIO2 配置保持一致
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  extrinsic_T: [-0.011, -0.02329, 0.04412]
```

## 3. 目录结构

```
include/
├── slam/
│   ├── common_types.hpp              # PointT/CloudT/Pose6D/Neighbor/PlaneCoef
│   ├── nn/neighbor_search.hpp        # Query, INeighborSearch (2 方法)
│   ├── map/map_manager.hpp           # MapConfig, MapStats, IMapManager (13 方法)
│   ├── map/ikd_tree_map.hpp          # IkdTreeMap : IMapManager
│   ├── map/oct_vox_map.hpp           # OctVoxMapAdapter : IMapManager
│   ├── io/                           # Phase B-3 预留
│   ├── reloc/                        # Phase B-4 预留
│   └── graph/                        # Phase C 预留
├── common_lib.h / so3_math.h / use-ikfom.hpp / IKFoM_toolkit/
└── ikd-Tree/
src/
├── laserMapping.cpp                  # main() — 按 yaml map_backend 创建后端
├── preprocess.{h,cpp}
├── IMU_Processing.hpp
└── slam/map/
    ├── ikd_tree_map.cpp              # IkdTreeMap 实现（~300 行）
    └── oct_vox_map.cpp               # OctVoxMapAdapter 实现（~250 行）
```

## 4. ROS 层契约

与 FAST-LIO2 完全一致（不改 topic/tf/参数名），便于 `greenhouse_nav` 不修改 launch：

- 节点名：`laserMapping`
- pkg 名：`fast_lio_super`
- launch：`roslaunch fast_lio_super mapping_mid360.launch`
- 订阅：`/livox/lidar` (CustomMsg), `/livox/imu` (Imu)
- 发布：`/Odometry`, `/cloud_registered`, `/path`

## 5. 技术调研链（docs/ 目录）

见根级 CLAUDE.md 的 `技术调研体系` 章节。

## 6. 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps fast_lio_super
# Docker 验证: fastlivo-dev:noetic-ros-base
```
