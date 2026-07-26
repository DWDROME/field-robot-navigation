# FAST_LIO — 原版 FAST-LIO2 Baseline

**归属项目**: 项目 B — 温室自主导航系统（baseline）
**标签**: `C++14` `IESKF` `ikd-Tree` `LiDAR Odometry`

---

## 技术背景

FAST-LIO2 核心算法（Xu et al., IEEE TRO 2022）：
- 紧耦合 IESKF（迭代误差状态卡尔曼滤波），直接 raw point scan-to-map 配准
- ikd-Tree 增量式 KD-Tree，支持高效动态插删和 KNN 搜索
- Livox Mid360 非重复扫描适配（preprocess.cpp 中的时间戳/畸变校正）

## 在此仓库中的角色

对照 baseline。`FAST_LIO_SUPER` 的改动以此包的字节级行为为目标（Phase A 完成后退出一个可比的 `/Odometry`）

### 你的改动

- `launch/mapping_mid360.launch`：添加 `LD_PRELOAD` 解决 libusb 冲突

```xml
<!-- 远端装有海康 MVS，/opt/MVS/lib/64 会污染 libusb 解析 -->
<env name="LD_PRELOAD" value="/lib/x86_64-linux-gnu/libusb-1.0.so.0" />
```

- 传感器前下倾 27° 的外参从 `config/mid360.yaml` 迁移到 `greenhouse_nav`

### 关键文件

| 文件 | 行数 | 核心功能 |
|------|------|----------|
| `src/laserMapping.cpp` | ∼1000 | IESKF 主循环：预测 → KNN 配准 → 更新 → map insert |
| `src/IMU_Processing.hpp` | ∼300 | IMU 预积分、流形状态传播 |
| `src/preprocess.cpp` | ∼200 | LiDAR 点云畸变校正、时间戳对齐 |
| `config/mid360.yaml` | 36 | 传感器参数、滤波参数、PCD 保存开关 |

### 与 FAST_LIO_SUPER 对照

| 维度 | FAST_LIO | FAST_LIO_SUPER |
|------|----------|----------------|
| C++ | C++14 | C++17 |
| License | BSD-3 | GPLv3 |
| 地图 | 全局 `ikdtree` 变量 | `slamMap()` 接口 + 双后端 |
| 搜索 | `Nearest_Search()` 直接调用 | `slamMap().search().knn()` |
| 配置 | `mapping:` yaml 段 | `mapping:` + `slam_core:` 段 |
