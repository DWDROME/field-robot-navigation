# FAST-LIO Super — slam_core API Contract

本文档锁定 `slam_core` 层的接口语义。**调用方应仅依赖这里描述的行为**；
实现方（IkdTreeMap / OctVoxMap / HknnSearch / …）在不违反本合同的前提下可自由替换。闭环实现和双 ROS 入口见 [CLOSED_LOOP.md](CLOSED_LOOP.md)。

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
- **输入**：`q.center` 在**地图坐标系**下；k 超过 8 时截断；`q.max_dist` 单位 m，包含边界，正无穷表示不限距离。非正 k、非法坐标、负/NaN 距离返回 0。
- **输出**：`out[0..n-1]` 按 `dist2` 升序；返回值 `n ≤ q.k`。
- **非阻塞**：不得持有跨帧锁；允许内部短暂锁。
- **不变式**：允许多个查询并行。调用方将插入、删除、裁剪和导出与查询阶段串行化；OctVox 不支持写入与查询同时执行。

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

std::unique_ptr<IMapManager> makeIkdTreeMap();
std::unique_ptr<IMapManager> makeOctVoxMap();
```

### 3.1 `insert` / `insertNoDownsample`
- `insert`：后端执行下采样（`configure().voxel_size`）。
- `insertNoDownsample`：不做下采样，保留原始点；对应 FAST-LIO2 的
  "PointNoNeedDownsample" 路径。
- 两者**都接受空首次调用**：内部若地图为空会触发首次 Build 等价操作。

### 3.2 `removeBoxes`
- 接收 `std::vector<BoundingBox>`（min/max 轴对齐）；返回实际删除数量。
- 两个后端均删除闭盒内的实际有效点，包括原始点；重叠盒不重复计数。OctVox 同步维护代表点、原始点和体素索引。

### 3.3 `drainRemoved`
- 取出"上次操作后被后端标记删除"的点并清空内部队列。
- FAST-LIO2 里用于可选的 `points_cache_collect`。
- 缓存上限为 `removed_capacity`，溢出丢弃最旧记录，累计数量见 `removed_dropped`。ikd-Tree 适配器禁用底层重复删除缓存，空间删除即时记录；代表点均值更新不形成空间删除事件。

### 3.4 `flattenInto`
- 把有效代表点及原始点以**追加**方式写入 `out`。此操作遍历地图，不在残差或每帧必要流程中调用。

### 3.5 `empty / size / validSize`
- `empty()` 首次 insert 前返回 true。
- `size()` 包含 tombstone；`validSize()` 只计有效点。ikd-tree 后端两者通常相近。

### 3.6 `search / snapshot / cropAround / stats`
- `search()` 返回引用在对象生命期内有效。
- `snapshot(voxel <= 0)` 导出有效地图；正 voxel 对导出额外下采样，在线地图不变。
- `cropAround()` 两个后端均保留给定中心和半径的闭球内点，球面保留。非有限中心和非正半径抛出参数错误。空间裁剪与 OctVox LRU 独立。
- OctVox `capacity` 限制启用 LRU 时的体素数；`raw_capacity` 独立限制原始点，即使关闭 LRU 也生效。紧凑代表点只保留 XYZ，查询和快照的其他字段为零。不能对非空地图修改体素尺度。
- OctVox 坐标须能在其 int32 子体素键范围内表示，预留搜索偏移余量；超范围坐标拒绝插入和查询，避免整数转换溢出。
- `stats()` 便宜调用；`last_*_ms` 可用于在线监控。

---

## 4. 预留接口

以下接口保留类型签名，当前未实现：

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

## 5. 前端接口

`laserMapping.cpp` 不再持有任何 `KD_TREE` 全局；所有地图访问都经
`IMapManager` / `INeighborSearch`。`IkdTreeMap::raw()` 已删除，地图后端由参数选择。

---

## 6. 线程模型

当前实现：
- 主线程：ROS 回调 + IESKF
- ikd-tree 后台：rebuild 线程（ikd-tree 自带）
- `IMapManager` 不再新增线程

- 查询阶段允许 OpenMP 并行读；插入、裁剪和导出与查询阶段串行。
- 闭环后端持有自己的有界关键帧点云，不读取正在更新的局部地图；工作线程停止时唤醒并 join。

**共识**：任何"让 IESKF 在后台线程跑"的改动都**不允许**出现在 slam_core，
它属于 `laserMapping.cpp` 的职责。
