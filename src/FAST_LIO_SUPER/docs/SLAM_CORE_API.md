# FAST-LIO Super — slam_core API Contract

本文档锁定 `slam_core` 层的接口语义。**调用方应仅依赖这里描述的行为**；
实现方（IkdTreeMap / OctVoxMap / HknnSearch / …）在不违反本合同的前提下可自由替换。

命名空间：`slam`、`slam::nn`、`slam::map`、`slam::io`、`slam::reloc`、`slam::graph`。

---

## 1. 公共类型（`slam/common_types.hpp`）

```cpp
using PointT   = pcl::PointXYZINormal;   // 与 FAST-LIO2 PointType 一致
using CloudT   = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

struct Pose6D  { Eigen::Matrix3d R; Eigen::Vector3d t; double stamp; };
struct PlaneCoef { Eigen::Vector4f abcd; float curvature; bool valid; };
struct Neighbor  { PointT p; float dist2; uint32_t voxel_id; };

constexpr int kDefaultKnn = 5;
```

**约束**：
- `PointT` 与 `common_lib.h::PointType` 指向同一 PCL 类型，迁移期内可互转。
- `Neighbor::dist2` 为**平方距离**，单位 m²。
- `Neighbor::voxel_id = 0` 表示后端不提供 voxel 索引（如 ikd-tree）。

---

## 2. `nn::INeighborSearch`

```cpp
struct Query { PointT center; int k = 5; float max_dist = 1.0f; };

class INeighborSearch {
 public:
  virtual int  knn(const Query&, std::array<Neighbor, 8>& out) const = 0;
  virtual bool fitPlane(const Query&, PlaneCoef& out) const = 0;
  virtual const char* name() const = 0;
};
```

### 2.1 `knn`
- **输入**：`q.center` 在**地图坐标系**下；`q.k` ≤ 8；`q.max_dist` 单位 m。
- **输出**：`out[0..n-1]` 按 `dist2` 升序；返回值 `n ≤ q.k`。
- **非阻塞**：不得持有跨帧锁；允许内部短暂锁。
- **不变式**：对同一 map 快照和同一 query，多次调用结果一致（线程安全级别 = 读多写一）。

### 2.2 `fitPlane`
- 等价于 FAST-LIO2 的 `esti_plane`：在邻居点集上拟合平面，最大残差 > 0.1 m 返回 `valid=false`。
- 子类可以提供更优实现（例如基于 voxel 内代表点的预拟合），但对外语义不变。

### 2.3 `name`
- 仅用于日志与 stats；允许出现 `"ikdtree"`、`"hknn"`、`"brute"`（测试用）等。

---

## 3. `map::IMapManager`

```cpp
struct MapConfig { /* 省略，见头文件 */ };
struct MapStats  { /* 省略，见头文件 */ };

class IMapManager {
 public:
  virtual void                  configure(const MapConfig&) = 0;
  virtual void                  insert(const PointVec& world_pts) = 0;
  virtual void                  insertNoDownsample(const PointVec& world_pts) = 0;
  virtual nn::INeighborSearch&  search() = 0;
  virtual CloudPtr              snapshot(float voxel = 0.0f) const = 0;
  virtual void                  flattenInto(PointVec& out) const = 0;
  virtual int                   removeBoxes(const std::vector<BoundingBox>&) = 0;
  virtual void                  cropAround(const Eigen::Vector3d&, float radius) = 0;
  virtual void                  drainRemoved(PointVec& out) = 0;
  virtual bool                  empty()     const = 0;
  virtual std::size_t           size()      const = 0;
  virtual std::size_t           validSize() const = 0;
  virtual MapStats              stats()     const = 0;
  virtual const char*           name()      const = 0;
};

std::unique_ptr<IMapManager> makeIkdTreeMap();   // Phase A
// std::unique_ptr<IMapManager> makeOctVoxMap(); // Phase B
```

### 3.1 `insert` / `insertNoDownsample`
- `insert`：后端执行下采样（`configure().voxel_size`）。
- `insertNoDownsample`：不做下采样，保留原始点；对应 FAST-LIO2 的
  "PointNoNeedDownsample" 路径。
- 两者**都接受空首次调用**：内部若地图为空会触发首次 Build 等价操作。

### 3.2 `removeBoxes`
- 接收 `std::vector<BoundingBox>`（min/max 轴对齐）；返回实际删除数量。
- ikd-tree 后端对应 `Delete_Point_Boxes`；OctVox 后端按 voxel id 批量标记删除。

### 3.3 `drainRemoved`
- 取出"上次操作后被后端标记删除"的点并清空内部队列。
- FAST-LIO2 里用于可选的 `points_cache_collect`。

### 3.4 `flattenInto`
- 把内部代表点以**追加**方式写入 `out`。性能便宜，可用于帧末尾可视化。

### 3.5 `empty / size / validSize`
- `empty()` 首次 insert 前返回 true。
- `size()` 包含 tombstone；`validSize()` 只计有效点。ikd-tree 后端两者通常相近。

### 3.6 `search / snapshot / cropAround / stats`
- `search()` 返回引用在对象生命期内有效。
- `snapshot()` 做一次拷贝导出，适合偶发落盘/可视化。
- `cropAround()` 对 ikd-tree 后端是 no-op；Phase B 的 OctVox 用它做 LRU 硬裁剪。
- `stats()` 便宜调用；`last_*_ms` 可用于在线监控。

---

## 4. Phase B 预留接口

以下接口在 Phase A 不实现，但类型签名已约定，后续 impl 直接对齐：

```cpp
namespace slam::io {
  class IMapIO {
    virtual bool save(const map::IMapManager&, const std::string& path) const = 0;
    virtual bool load(map::IMapManager&,       const std::string& path) const = 0;
  };
}

namespace slam::reloc {
  struct RelocRequest { CloudPtr scan; Pose6D prior; bool has_prior = false; };
  struct RelocResult  { bool ok; Pose6D pose; float score; int inliers; double elapsed_ms; };
  class IRelocalizer {
    virtual void        attach(const map::IMapManager&) = 0;
    virtual RelocResult relocate(const RelocRequest&) = 0;
  };
}
```

---

## 5. 迁移期兼容约定

**Phase A 完成后（当前状态）**：`laserMapping.cpp` 不再持有任何 `KD_TREE` 全局；
所有地图访问都经 `slamMap()` 返回的 `IkdTreeMap&`。`IkdTreeMap::raw()` 仍保留
以便调试，Phase B 替换 OctVoxMap 前删除。

**数值等价判据**：
- 同一 bag，`fast_lio/fastlio_mapping` vs `fast_lio_super/fastlio_super_mapping`
  回放，`/Odometry` 位置差 < 1e-4 m
- 地图尺寸 `validnum()` 相同或差 ≤ 10（ikd-tree 的插入顺序可能极小影响 rebuild）。

---

## 6. 线程模型

Phase A：
- 主线程：ROS 回调 + IESKF
- ikd-tree 后台：rebuild 线程（ikd-tree 自带）
- `IMapManager` 不再新增线程

Phase B：
- 允许 OctVox insert 与 IESKF knn 并行（读写分离）
- TBB/OpenMP 并发模型在 CMakeLists.txt 里单独开关（`SLAM_USE_TBB`）

**共识**：任何"让 IESKF 在后台线程跑"的改动都**不允许**出现在 slam_core，
它属于 `laserMapping.cpp` 的职责。
