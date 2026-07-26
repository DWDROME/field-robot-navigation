# FAST_LIO_LOCALIZATION — 全局重定位

**归属项目**: 项目 B — 温室自主导航系统
**定位**: 基于预建 PCD 地图 + 粗初值的全局重定位模块。

> **许可证**: GPL-2.0（只读代码参考，不可并库）
> **语言**: C++14 + Python

---

## 1. 对此模块的了解（来自你的 License 审计）

在 `CLONE_AUDIT.md` 和 `BACKEND_SELECTION_REPORT_V2.md` 中你已确认：

- **GPL-2.0-only** 与 FAST_LIO_SUPER 的 **GPLv3 不兼容**
- 此包只能作为设计参考，不可将其代码直接并入 FAST_LIO_SUPER
- 已评估 LiLoc（MIT）作为替代方案，见 `docs/LILOC_EVALUATION.md`

## 2. 关键文件

| 文件 | 说明 |
|------|------|
| `src/laserMapping.cpp` | 重定位主 node |
| `scripts/global_localization.py` | 全局定位（Python 2.7 + Open3D 0.9） |
| `scripts/transform_fusion.py` | TF 融合（map→odom→base_link） |
| `scripts/publish_initial_pose.py` | 发布粗初值工具 |

## 3. 工作流

1. 依赖预建 PCD（默认 `FAST_LIO/PCD/scans.pcd`）
2. 等待 `/initialpose` 粗初值
3. 低频（0.5-0.2 Hz）全局定位
4. 发布 `/state_estimation`

## 4. 已知限制

你的调研发现：
- 依赖 Python 2.7 + Open3D 0.9（EOL 技术栈）
- 作者 TODO 标注 "把全局定位更平滑地融合回 FAST-LIO 状态估计" — 桥接不完整
- Phase C 的 relocalizer 规划已走向 LiLoc 路线
