# terrain_analysis — CMU 地形分析 (Elevation Mapping)

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `Elevation Map` `Quantile Ground Estimation` `Voxel Grid Rollover` `Scan Decay`

---

## 算法原理

此包是 CMU/CARLA 地形的核心。并非简单地形分类，而是完整的 **局部高程地图** (elevation mapping) 管线：

### 双层体素结构

```
第1层: terrainVoxel (分辨率 1.0m, 21×21 = 21m×21m 局部地图)
  └─ 原始点云累积 + 时间衰减管理
  
第2层: planarVoxel (分辨率 0.2m, 51×51 = 10.2m×10.2m)
  └─ 地面估计 + 高程图 + 动态障碍物检测
```

### terrainVoxel — 滚动栅格 (Rolling Grid)

地图跟随车辆移动做 **环形缓冲区移位**（非暴力重建）：

```cpp
// terrain_voxel: 21×21, 1.0m/格, 以车辆为中心
// 当车辆跨越 voxel 边界时，整体平移一格、清空新暴露的 voxel

// X 负向溢出: 每行向右移一格，最左列清空复用给最右列
while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
  for (int indY = 0; indY < terrainVoxelWidth; indY++) {
    terrainVoxelCloud[terrainVoxelWidth*(terrainVoxelWidth-1) + indY]  // 保存最右列
    for (int indX = terrainVoxelWidth-1; indX >= 1; indX--)
      terrainVoxelCloud[terrainVoxelWidth*indX + indY] = terrainVoxelCloud[terrainVoxelWidth*(indX-1) + indY]
    terrainVoxelCloud[indY] = saved_ptr;  // 最右列移到最左列
    terrainVoxelCloud[indY]->clear();      // 清空
  }
  terrainVoxelShiftX--;
}
// 同理 Y 方向 (上/下)、X 正方向
```

**关键参数**：
- `voxelPointUpdateThre = 100`：每 voxel 累积 100 个点时触发更新
- `voxelTimeUpdateThre = 2.0s`：强制更新间隔
- `decayTime = 2.0s`：点云时间衰减常数，超过 2s 的点被丢弃
- `noDecayDis = 4.0m`：4m 内的点不受时间衰减影响

更新前做 `pcl::VoxelGrid` 体素降采样（`scanVoxelSize = 0.05m`）。

### planarVoxel — 地面估计与高程图

**分位数地面估计**（`useSorting=true, quantileZ=0.25`）：

```cpp
// 每个 0.2m×0.2m 栅格收集附近 3×3 邻域内所有点的高度
for (int dX = -1; dX <= 1; dX++)
  for (int dY = -1; dY <= 1; dY++)
    planarPointElev[indX+dX][indY+dY].push_back(point.z);

// 排序后取 quantileZ = 25% 分位数作为地面高度
sort(planarPointElev[i].begin(), planarPointElev[i].end());
int quantileID = int(0.25 * size);
planarVoxelElev[i] = planarPointElev[i][quantileID];
```

若 `useSorting=false` 则取最低点的 minZ 作为地面（`terrain_analysis_ext` 使用此模式）。

**高程图生成**：每点计算相对于地面高度 `disZ = point.z - planarVoxelElev[ind]`，写入 `point.intensity` 字段发布为 `/terrain_map`。只有 `disZ < vehicleHeight(1.5m)` 且 `planarPointNum >= minBlockPointNum(10)` 的点被保留。

**高程图输出格式**：

```
/terrain_map: PointCloud2, frame_id="map"
  每个点:
    x,y,z = 原始坐标
    intensity = 离地高度 disZ (m)
  // 可视化时按 intensity 着色：蓝色=地面(0m) ~ 红色=1.5m 障碍物
```

### 动态障碍物检测 (`clearDyObs=true`)

```cpp
// 将点从 map 系变换到 vehicle 车身坐标系
// 计算 body-frame 下仰角 angle4
// 若 angle4 ∈ [minDyObsVFOV, maxDyObsVFOV] = [-16°, 16°]
// 或 |点z| < absDyObsRelZThre(0.2m)
// → 标记为 planarVoxelDyObs[ind]++

// 二次过滤: 原始未降采样点中角度 > minDyObsAngle 的点 → 清除 DyObs 标记
// 最终: planarVoxelDyObs[ind] < minDyObsPointNum(1)  → 该栅格视为无效
```

### 无数据区障碍物填充 (`noDataObstacle=false`, 默认关闭)

当启用时，未被激光扫描覆盖的区域被标记为障碍物。通过形态学膨胀（`noDataBlockSkipNum` 次迭代）填充未观测栅格。

---

## 双图层配置（你的集成工作）

两个实例并行运行，参数不同以适应不同用途：

| 参数 | `terrain_analysis` | `terrain_analysis_ext` | 含义 |
|------|-------------------|----------------------|------|
| `scanVoxelSize` | **0.05m** | 0.1m | 预处理降采样分辨率 |
| `decayTime` | **2.0s** | 10.0s | 点云衰减速度（快 vs 慢） |
| `clearingDis` | 8.0m | 30.0m | 清除半径 |
| `useSorting` | **true** | false | 排序分位数 vs 最小值估计地面 |
| `quantileZ` | **0.25** | 0.1 | Z 分位数 |
| `noDecayDis` | 4.0m | 0m | 近距离免衰减半径 |
| `clearDyObs` | true | — | 动态障碍物过滤 |
| `lowerBoundZ/upperBoundZ` | — | **-2.5/1.0** | 扩展 Z 范围 |
| `checkTerrainConn` | — | **false** (可启用) | 地形连通性检查 |

**图层用途**：
- `/terrain_map` (terrain_analysis)：**快衰减 + 动态障碍物过滤** → 供 local_planner 实时避障
- `/terrain_map_ext` (terrain_analysis_ext)：**慢衰减 + 更宽 Z 范围** → 供 far_planner 全局路径规划保留更多历史/远距离信息

### 你的集成改动

此包来自 CMU/CARLA 自动驾驶生态，你做的集成工作包括：

1. 将 topic 从上游默认连接改为对接 FAST-LIO 的 `/registered_scan`（仓库内重映射为 `/cloud_registered`）
2. 帧对齐：保证 `/terrain_map` 在 `map` 坐标系下发布，与 `/state_estimation` 一致
3. 双图层并行：两个实例用不同参数同时运行，分别供给 local_planner 和 far_planner
4. 整合进 `greenhouse_nav` 的 launch 装配
