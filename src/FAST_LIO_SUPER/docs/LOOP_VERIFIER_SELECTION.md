# Loop Verifier 选型：small_gicp vs nano_gicp

**日期**：2026-04-18
**决策对象**：Phase C-1 的 `ILoopVerifier` 默认实现
**结论一句话**：**选 small_gicp**；nano_gicp 保留为编译期冲突的 fallback。

---

## 0. 候选总表

| 维度 | **small_gicp**（koide3/AIST） | **nano_gicp**（engcang fork） |
|---|---|---|
| License | MIT | MIT |
| C++ 标准 | **C++17**（匹配我们） | C++14 |
| 代码量 | 10,337 行 | 3,261 行 |
| 核心依赖 | Eigen + **bundled** nanoflann + **bundled** Sophus | PCL ≥ 1.8 + OpenMP + Eigen |
| PCL 依赖 | **可选**（drop-in 接口存在） | **强依赖** |
| 并行后端 | OpenMP **+ TBB** | 仅 OpenMP |
| CI / 测试 / codecov | 完整 | 无 |
| 论文/发表 | JOSS 2024（Koide） | DLO/DLIO 2022（UCLA） |
| 维护者状态 | **主线活跃**（与 GLIM / hdl_graph_slam 同作者） | engcang fork，原 Kenny Chen 仓库不更新 |
| 原生 API 返回信息 | `{ converged, error, iterations, num_inliers, H, b, T }` | pcl::Registration 风格（`getFitnessScore`） |
| 支持的 factor | GICP / plane-ICP / point-ICP / VGICP + 可自定义 | 仅 GICP |
| 额外功能 | voxelmap、incremental registration、Python 绑定 | 无 |
| 实战验证 | GLIM / hdl_graph_slam / 多篇 2024 论文 | FAST-LIO-SAM-QN（engcang） |

---

## 1. API 对比

### 1.1 nano_gicp（PCL drop-in 风格）

```cpp
nano_gicp::NanoGICP<PointT, PointT> gicp;
gicp.setNumThreads(4);
gicp.setMaxCorrespondenceDistance(1.5);
gicp.setInputSource(src);
gicp.setInputTarget(dst);

pcl::PointCloud<PointT> aligned;
gicp.align(aligned, init_guess);

if (gicp.hasConverged() && gicp.getFitnessScore() < 0.3) {
  Eigen::Matrix4f T = gicp.getFinalTransformation();
}
```

**优点**：与 FAST-LIO-SAM 的 `pcl::IterativeClosestPoint` **字面上几乎一模一样**，切换时改的行数可能只有 5-10 行。

**缺点**：返回信息少（只有 fitness），loop factor 方差建模只能沿用 FAST-LIO-SAM 的 "各向同性 6D = score" 粗糙方案。

### 1.2 small_gicp（原生 API）

```cpp
using namespace small_gicp;

auto target = voxelgrid_sampling_omp(*raw_tgt, 0.3, num_threads);
auto source = voxelgrid_sampling_omp(*raw_src, 0.3, num_threads);
auto target_tree = std::make_shared<KdTree<PointCloud>>(target, KdTreeBuilderOMP(num_threads));
auto source_tree = std::make_shared<KdTree<PointCloud>>(source, KdTreeBuilderOMP(num_threads));
estimate_covariances_omp(*target, *target_tree, 10, num_threads);
estimate_covariances_omp(*source, *source_tree, 10, num_threads);

Registration<GICPFactor, ParallelReductionOMP> registration;
registration.reduction.num_threads = num_threads;
registration.rejector.max_dist_sq = 1.5 * 1.5;

auto result = registration.align(*target, *source, *target_tree, init_T);
// result.converged / result.error / result.iterations / result.num_inliers / result.H / result.b / result.T_target_source
```

**优点**：
- `num_inliers` 直接暴露 → 可以做 **inlier-ratio-based loop validation**（比 fitness 更鲁棒）
- `H`（6×6 Hessian）暴露 → 可以从 Hessian 反演 covariance 作 loop factor 方差（**LIO-SAM 风格 fitnessScore 的升级**）
- `result.T_target_source` 是 `Eigen::Isometry3d`，直接喂 GTSAM Pose3

**缺点**：首次集成要多写 20-30 行 setup。

### 1.3 small_gicp 的 PCL drop-in（折中）

`#include <small_gicp/pcl/pcl_registration.hpp>` 后，有 `small_gicp::RegistrationPCL<PointT, PointT>` 类，继承 `pcl::Registration`，API 和 nano_gicp 一模一样：

```cpp
small_gicp::RegistrationPCL<PointT, PointT> gicp;
gicp.setRegistrationType("GICP");   // 或 "VGICP"
gicp.setNumThreads(4);
gicp.setInputSource(src);
gicp.setInputTarget(dst);
gicp.align(aligned);
```

**这是最佳过渡方案**：迁移成本 = nano_gicp；未来要升级时可无缝切到原生 API 拿 H/inliers。

---

## 2. 与 FAST-LIO-SAM 的替换成本

FAST-LIO-SAM 的 `loop_closure.cpp` 使用 `pcl::IterativeClosestPoint`。

**替换到 nano_gicp**：改 ~5 行（只是 include + 类型名）。
**替换到 small_gicp PCL drop-in**：改 ~6 行（include + 类型名 + setRegistrationType）。
**替换到 small_gicp 原生**：改 ~40 行（重写 align 前后的数据转换）。

---

## 3. 决定性差异

### 3.1 维护活跃度（长期）

| | small_gicp | nano_gicp |
|---|---|---|
| 作者 | Kenji Koide（AIST） | engcang fork UCLA |
| 同作者其它项目 | GLIM, hdl_graph_slam, fast_gicp, direct_visual_lidar_calibration | 个人 fork |
| 2024-2026 活跃度 | 持续发布新 release，JOSS paper | engcang fork 基本冻结 |
| upstream（原 Kenny Chen DLO） | — | **已停更**（2023 后） |

**向前看 3-5 年**，small_gicp 的持续性明显更好。nano_gicp 是 FAST-LIO-SAM-QN 的伴生仓库，若 engcang 转向，我们需要自己 fork 维护。

### 3.2 loop factor 方差建模

深度报告 v2 建议："Hessian 反演 / inlier-ratio 作 loop factor noise" 比 "fitness score 各向同性" 更合理。small_gicp 的 `result.H` 直接给出 6×6 Hessian，**反演就是 covariance**：

```cpp
Eigen::Matrix<double, 6, 6> covariance = result.H.inverse();
auto loop_noise = gtsam::noiseModel::Gaussian::Covariance(covariance);
```

nano_gicp 要自己从 pcl::Registration 内部取 H（需修改库），或退回 fitness 方案。

### 3.3 潜在依赖冲突（头文件）

small_gicp bundled **Sophus** 和 **nanoflann**。我们项目已有的 header-only 库：
- `slam_octvox`（自维护）— 无 nanoflann
- `FAST_LIO_SUPER` — 无 Sophus
- 依赖 PCL 间接引入 FLANN（与 nanoflann 是不同库）

**预估冲突风险**：**低**。但 catkin 工作空间其它包（如 `far_planner` / `terrain_analysis`）若单独用了 Sophus/nanoflann，需要逐个验证。

---

## 4. 推荐方案

### 4.1 C-1：`small_gicp`（PCL drop-in 模式）

```cpp
// include/slam/graph/loop_verifier_small_gicp.hpp
#include <small_gicp/pcl/pcl_registration.hpp>

class SmallGicpLoopVerifier : public ILoopVerifier {
  small_gicp::RegistrationPCL<PointT, PointT> gicp_;
public:
  SmallGicpLoopVerifier(const LoopVerifierConfig& cfg) {
    gicp_.setRegistrationType("GICP");
    gicp_.setNumThreads(cfg.num_threads);
    gicp_.setMaxCorrespondenceDistance(cfg.max_corr_dist);
  }
  RegistrationOutput verify(const CloudPtr& src, const CloudPtr& dst,
                             const Eigen::Matrix4f& init) override {
    gicp_.setInputSource(src);
    gicp_.setInputTarget(dst);
    pcl::PointCloud<PointT> aligned;
    gicp_.align(aligned, init);
    RegistrationOutput out;
    out.converged = gicp_.hasConverged();
    out.score = gicp_.getFitnessScore();
    out.T = gicp_.getFinalTransformation().cast<double>();
    return out;
  }
};
```

**首版用 fitness 做判收**（和 FAST-LIO-SAM 一致），之后升级到原生 API 拿 H/inliers。

### 4.2 C-1 的 catkin 集成路线

- **推荐**：把 `small_gicp` 作为 catkin package 加入工作空间（已 clone 到 `~/refs/`，复制到 `src/` 或作为 submodule）。小石头实测：包本身 header-only + src 可用，`catkin build small_gicp` 即可。
- **备选**：若 docker 镜像里装了 `libsmall-gicp-dev` apt 包，更干净（**待确认 noetic 下是否已打包**；初步判断是 **没有** — small_gicp 2024 年才正式发布）。

### 4.3 Fallback：nano_gicp

仅当出现以下情况之一时切到 nano_gicp：
1. small_gicp 与现有依赖的 Sophus/nanoflann 冲突
2. catkin build small_gicp 在我们的 docker 里失败且 15 分钟内不能修
3. 需要完全与 FAST-LIO-SAM-QN 的精度对齐（同一验证器）

nano_gicp 作为 vendored 也容易（3k 行 header-only + PCL）。

---

## 5. 实测前的风险清单

1. **small_gicp 的 CMake 版本要求**：`CMake >= 3.16`。noetic docker 默认 cmake 3.16.3（OK）。
2. **Eigen 版本**：小要求 3.3.4，我们 3.3.7（OK）。
3. **TBB 要不要开**：**暂不开**。C-1 用 OpenMP 即可，避免混用并行模型。yaml 留 `num_threads` 参数。
4. **PCL 版本**：small_gicp PCL drop-in 要 PCL ≥ 1.11。noetic 默认 PCL 1.10。需要验证。**❗潜在阻塞点**。
5. **与我们 GPLv3 的兼容**：MIT → GPLv3 compatible，整体分发 GPLv3。OK。

**最高优先级要实测的**：PCL 版本。如果 noetic 的 PCL 1.10 不兼容 small_gicp PCL drop-in，有两个出路：
- 用 small_gicp 原生 API（绕开 PCL interface）
- 换 nano_gicp（PCL ≥ 1.8 即可）

---

## 6. 决策落地

**选 small_gicp，PCL drop-in 模式**。

| 项 | 值 |
|---|---|
| Package 入库方式 | 先复制到 `src/small_gicp/`，作为 catkin workspace member |
| 首版 API | PCL drop-in（`RegistrationPCL`） |
| 判收条件 | `hasConverged() && fitness < threshold` |
| Loop factor noise | **C-1 用 fitness 各向同性**（对齐 FAST-LIO-SAM）；**C-2 升级到 H.inverse()** |
| Threads | yaml `slam_core/backend/loop_verifier/num_threads`（默认 4） |
| MaxCorrDist | yaml（默认 `loop_radius × 1.5`） |
| Fallback 触发条件 | PCL 1.10 不兼容 → 切原生 API 或 nano_gicp |

---

## 7. 给 C-1 写代码时要做的事

1. `cp -r ~/refs/fast_lio_super/small_gicp/ src/small_gicp/`（首先实测编译）
2. 给 docker 镜像 `fastlivo-dev:noetic-ros-base` 加 `catkin build small_gicp` 步骤
3. 如 PCL 兼容问题：把 small_gicp 的 include 裁到 header-only 子集，绕开 pcl_registration_impl.hpp
4. 在 `fast_lio_super/CMakeLists.txt` 里 `find_package(small_gicp REQUIRED)` 或 `catkin_package( ... CATKIN_DEPENDS small_gicp )`
5. `include/slam/graph/loop_verifier.hpp`（接口）+ `loop_verifier_small_gicp.hpp`（实现）

---

## 8. 下一步选项

- **D** — 直接动手 C-1：docker 加 GTSAM 层 → 试编 small_gicp（PCL 1.10 兼容测试）→ 写接口头
- **E** — 读 FAST_LIO_SLAM/SC-PGO 对比 sidecar 边界（另一种设计参考）
- **F** — 读 SOLiD CPU 实现（269 行）评估接入成本
- **G** — 先做 docker 镜像加 GTSAM 4.0.3 + small_gicp 的预编译验证（可能卡住的那一步）

回字母/组合。
