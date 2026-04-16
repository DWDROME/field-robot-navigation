# Super-LIO Reference Notes

**读源时间**：2026-04-17
**上游**：`https://github.com/Liansheng-Wang/Super-LIO.git`（ros1 分支）
**论文**：arXiv:2509.05723（RA-L 2026 accepted）
**许可证**：**GPLv3**（见 `src/slam_octvox/LICENSE`）

本文档记录我们真正从 Super-LIO 源码里抽出来的、影响 `slam_octvox` 适配器设计的
**可引用事实**。不写到这里的任何关于 Super-LIO 的描述都不可信。

---

## 1. 文件构成

我们在 `slam_octvox` 里 vendored 的文件：

| 路径 | 来源 | 许可证 | 修改 |
|---|---|---|---|
| `include/slam_octvox/OctVoxMap.hpp` | `src/super_lio/include/OctVoxMap/OctVoxMap.hpp` | GPLv3 | 删 `<execution>`（未使用）/ `<filesystem>` / `saveMap()` |
| `include/slam_octvox/HKNN_list60_gem.h` | 同目录 | GPLv3 | 无 |
| `include/tsl/robin_*.h` | `src/super_lio/include/OctVoxMap/tsl/` | MIT | 无 |

**没有** vendored 的文件：
- `src/super_lio/src/lio/super_lio.cpp`：完整前端，与 FAST-LIO2 冲突，不用
- `src/super_lio/include/lio/*.h`：same
- `src/basic/`：V3/SE3/scalar 别名包，我们直接用 `Eigen::Vector3f`

## 2. OctVoxMap 核心事实

### 2.1 点类型
- 模板参数 `Point` = `Eigen::Vector3f`（实际用法，见 `super_lio.h:55`
  `OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>` 且 `V3 = Matrix<float,3,1>`）
- 地图**不承载** intensity / normal / ring / time。原始点的这些字段在 `super_lio.cpp`
  里经 `transform * point_body` 转为 V3 之前就被丢了。

### 2.2 空间量化
```cpp
fine_key = floor(pt * (2/resolution))          // 双倍分辨率的 sub-voxel 格
key      = fine_key >> 1                       // 父 voxel
local_idx = (dz<<2) | (dy<<1) | dx             // 低位作为八分体 idx，0..7
```

### 2.3 子体素（OctVox::AddPoint）
- 每 voxel 持 `counts_[8]` + `points_[8]`；`UNINIT_MASK = 0` 表示空
- `MAX_POINTS_PER_SUBVOXEL = 20`：累计贡献上限
- `DISTANCE_THRESHOLD_SQ = 0.01`（即 0.1m²）：**新点与当前均值距离 >0.1m 直接拒绝**
- 更新：`stored = (stored*count + pt) / (count+1); count++;`
  这是 running mean，不是 Welford，数值稳定性对 20 次累计够用

### 2.4 LRU
- `std::list<pair<KEY, OctVox>> data_` + `tsl::robin_map<KEY, iter> grids_`
- 新 voxel：`data_.emplace_front(...); grids_[k] = begin();`
  若 `data_.size() >= capacity_` → `grids_.erase(back); pop_back();`
- 命中 voxel：`AddPoint; data_.splice(front, data_, iter);`（把命中 voxel 搬到头）

### 2.5 resetMap 保护期
- `resetMap(cloud)` 后 `reset_map_count_ = 10`
- 接下来 10 帧 `insert()` 直接 skip，给刚载入的预建图留稳定时间
- 对重定位/预建图流程至关重要

## 3. HKNN（getTopK）核心事实

### 3.1 输入 / 输出
- 输入 `Point`（V3）；输出 `KNNHeap<5, Point>`（K 硬编码 5）
- `KNNHeap` 用 "最差点索引 + unrolled 比较" 维护前 5 近邻
- 查询结束时 `count` 可能 <5；`super_lio.cpp` 判定 `count < 4` 则为无效残差

### 3.2 分层搜索
- 6 个组（Group0..Group5），每组拆一段遍历表
  - `flat_search_order_offsets[7] = {0, 43, 135, 219, 321, 465, 593}`
  - `flat_search_order[593]` = 每组内的 `(neighbor_voxel_idx, k, local_idx × k)` 交错
- Group0 只访问当前 voxel 的 8 个 octant，优先级最高
- Groups 1-5 访问 `HKNN_neighbor_voxel[60]` 里的外围 voxel，逐组扩展
- **Mirror 优化**：`mirror_axis = (1-2dx, 1-2dy, 1-2dz)`，把相对 offset 按查询 octant 对称翻转；`n_key = key + mirror_axis * HKNN_neighbor_voxel[i]`
- **XOR 优化**：`_local_idx = table_local_idx ^ local_idx` 把预表里的相对 local_idx 转成绝对的 8 octants 之一
- **Top-8 voxel 缓存**：Group0 的 8 个 octant 邻居只 hash lookup 一次，存 `top_voxels_2_search[8]`，后续组若 neighbor_idx < 8 直接复用

### 3.3 早停（early stop）
- 每组结束后：若 `top_K.count == 5 && top_K.max_dist2_ < orders_min_dis2[group_idx]` → break
- `orders_min_dis2 = {0.0625, 0.125, 0.25, 0.3125, 0.375, 100}`（最后一个等于 "永远不停"）
- 这些值是 Super-LIO 预计算的**当前组之外 voxel 的最小可能平方距离下界**，作者
  对 resolution=0.5 的特化。**如果改 resolution，这些阈值也要重算**，否则早停
  会过激或过保守。

### 3.4 `decrease_max_group()` / `reset_max_group()`
- 手动调节最多搜索到第几组；`super_lio.cpp` 里在某些场景下收窄 group 上限
- 默认 `group_idx_max_ = 6`（即全部 6 组都走）

## 4. 语义阻抗（与 FAST-LIO2 / 我们的 IMapManager 的差异）

| 语义 | FAST-LIO2 ikd-tree | Super-LIO OctVox | 我们的处理 |
|---|---|---|---|
| 删除 | `Delete_Point_Boxes(FoV cubes)` | 无，靠 LRU | `OctVoxMapAdapter::removeBoxes` no-op |
| 淘汰点回取 | `acquire_removed_points()` | 不追踪 | `drainRemoved` no-op |
| 半径裁剪 | 隐式（FoV + rebuild） | LRU 均摊 | `cropAround` no-op |
| insert 去重 | `Add_Points(downsample_on=true/false)` 两档 | running-mean + 0.1m 阈值，只有一档 | `insertNoDownsample` 降级为 `insert` |
| knn K | 5（`NUM_MATCH_POINTS`），`<5` 无效 | 5（硬编码），`<4` 无效 | 接口留 k 参数，实际按后端强制 |
| 点类型 | `pcl::PointXYZINormal` | `Eigen::Vector3f` | 适配器做双向投射，intensity/normal=0 |

## 5. resolution / voxel_size 的语义

- FAST-LIO2 `filter_size_map` = ikd-tree 的 `downsample_size`，代表点间距下限
- Super-LIO `resolution` = voxel 边长；**sub-voxel 边长 = resolution/2**
- 直接把 FAST-LIO 的 0.5 填给 Super-LIO 的 resolution，**有效代表点密度变一倍
  稀疏**（因为 Super-LIO 的 sub-voxel 是 0.25m）。A/B 实验时这个要作为配置
  对齐的第一档变量。

## 6. 并发模型

Super-LIO 用 TBB，我们用 OpenMP。这**不影响** OctVoxMap.hpp 本身——map 的
insert 与 getTopK 都是**单线程接口**。并发调度发生在调用层（`super_lio.cpp`
对每帧 knn 循环做 `tbb::parallel_for`）。我们的 `laserMapping.cpp` 对应的
knn 循环是单线程（在 `h_share_model` 里），这意味着：

- **初版 OctVox 后端 in FAST-LIO2 会比 Super-LIO 官方慢**，因为并发模型没上
- 要重现论文级的时延，需要 Phase B 的另一项工作：laserMapping 并行化

---

## 7. 后续复核点（Phase B-2+ 必读）

- [ ] Group 内部顺序对早停保守度的影响（orders_min_dis2 的物理意义）
- [ ] `MAX_POINTS_PER_SUBVOXEL=20` 的灵敏度：是否应该跟 voxel_size 联动
- [ ] `DISTANCE_THRESHOLD_SQ=0.01` 对 LiDAR 噪声 σ 的假设
- [ ] `decrease_max_group` / `reset_max_group` 的切换条件（`super_lio.cpp` 里）
- [ ] `getTopK_VN`（朴素 19-邻居）vs `getTopK`（HKNN）在不同场景下的优劣；
      是否需要在 yaml 里留开关

## 8. 我们没读但要读的

- `super_lio.cpp`：`UpdateMap` 流程、`Observe` 流程（尤其 `calc_plane_coeff`
  与 FAST-LIO `esti_plane` 的差异）
- `super_lio_reloc.cpp`：relocalization 服务（Phase B-4 参考）
- `basic/Manifold.h`：SE3 + 切空间实现（如我们要升级到更干净的状态表示时用）
