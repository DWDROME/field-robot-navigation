# terrain_analysis_ext — CMU 扩展地形分析 (Elevation Mapping, Slow-decay)

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `Elevation Map` `Slow Decay` `MinZ Ground` `Terrain Connectivity`

---

## 角色

与 `terrain_analysis` 并行运行的第二个地形分析实例，参数配置为**慢衰减、长范围、minZ 地面估计**模式。

### 参数差异

| 参数 | `terrain_analysis` (快) | `terrain_analysis_ext` (慢) | 战术意义 |
|------|------------------------|----------------------------|----------|
| `scanVoxelSize` | 0.05m | **0.1m** | 更粗的预处理，减少计算量 |
| `decayTime` | 2.0s | **10.0s** | 5× 慢衰减，保留历史点 |
| `noDecayDis` | 4.0m | **0m** | 没有免衰减区 |
| `clearingDis` | 8.0m | **30.0m** | 更长清除距离 |
| `useSorting` | true | **false** | minZ 作为地面（比分位数更低） |
| `quantileZ` | 0.25 | 0.1 | 实际不用（useSorting=false） |
| `noDataObstacle` | false | **false** (可通过 arg 启用) | — |
| `minDyObsDis` | 0.3 | **无** | 不启用动态障碍物检测 |

### 特有参数

```xml
<param name="checkTerrainConn" type="bool" value="$(arg checkTerrainConn)"/>
<param name="terrainConnThre" type="double" value="0.5"/>
<param name="terrainUnderVehicle" type="double" value="-0.75"/>
<param name="ceilingFilteringThre" type="double" value="2.0"/>
<param name="localTerrainMapRadius" type="double" value="4.0"/>
```

`terrainConnThre=0.5m` 用于地形连通性检查（两个相邻栅格高度差 >0.5m 标为不可通行）。

### 用途

输出 `/terrain_map_ext` 供 far_planner 使用。因衰减更慢、更宽容的 Z 范围，适合远距离全局路径规划的 cost 构建，而非近处实时避障。
