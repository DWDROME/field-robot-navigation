# Clone Audit Report — Phase C 候选仓库

**日期**：2026-04-18
**位置**：`~/refs/fast_lio_super/`（持久化）
**磁盘占用**：~77 MB（仅代码，未含 bag/model）

---

## 0. Clone 结果汇总

### 0.1 成功（16 个，全部到位）

| 仓库 | License | cpp/h/py 行数 | keyframe/gtsam 文件数 | 磁盘 |
|---|---|---:|---:|---:|
| `small_gicp` | **MIT** | 10,337 | 0 | 6.8M |
| `glim` | **MIT** | 40,029 | 61 | 3.3M |
| `mola` | **GPL-3.0-only** | 30,914 | 3 | 6.6M |
| `LiLoc` | **MIT** | 7,288 | 7 | 864K |
| `nano_gicp` | **MIT** | 3,354 | 0 | 412K |
| `kiss-slam` | **MIT** | 832 (+ Python) | 2 | 668K |
| `Bonxai` | **MPL-2.0** | 6,136 | 0 | 16M |
| `fastlio2_ros2` | **MIT**（子包级） | 5,438 | 4 | 780K |
| `STD` | **GPL-2.0**（商业需谈） | 3,207 | 4 | 508K |
| `SOLiD` | **BSD-3 学术 / 商业需谈** | 269 cpp + Python | 0 | 37M |
| `FAST-LIO-SAM` | **CC-BY-NC-SA 4.0** | 1,056 | 5 | 1.8M |
| `FAST-LIO-SAM-QN` | **CC-BY-NC-SA 4.0** | 1,203 | 5 | 2.2M |
| `FAST-LIO-SAM-SC-QN` | **CC-BY-NC-SA 4.0** | 1,203 | 5 | 444K |
| `FAST_LIO_SLAM` | FAST-LIO=GPL-2.0；**SC-PGO 无 LICENSE** | 17,932 | 3 | ~16M |
| `FAST_LIO_LC` | FAST-LIO=GPL-2.0；**PGO 无 LICENSE** | 47,577 | 4 | ~20M |
| `BEVPlace` | **MIT** | 1,074 (Python) | 0 | ~5M |

**Clone 修复记录**：初始 `git clone` 因 GnuTLS `early EOF` 全部失败（WSL libcurl + TLS 大包 bug）。修复：
```
git config --global http.version HTTP/1.1
git config --global http.postBuffer 1048576000
```
切 HTTP/1.1 后 3 个仓库一次成功。

---

## 1. License 矩阵（与 Phase C 决策直接相关）

### 1.1 可直接并库（GPLv3 兼容）

- `small_gicp`（MIT） — **loop verifier** 首选
- `nano_gicp`（MIT） — loop verifier 备选
- `LiLoc`（MIT） — **relocalizer** 主线
- `glim`（MIT） — 高精度参考，可并库
- `kiss-slam`（MIT） — 轻量后端参考
- `Bonxai`（MPL-2.0） — planning sidecar，MPL 与 GPLv3 兼容
- `mola`（GPL-3.0-only） — 架构参考，可并库
- `fastlio2_ros2`（MIT） — ROS2 参考
- `BEVPlace`（MIT） — **可并库**（学习式 descriptor 备用，需 PyTorch 推理环境）
- `SOLiD`（BSD-3 学术） — **Mid360 descriptor 首选**；但"商业需联系作者"，商业化前要确认

### 1.2 只能读代码，不能并库（GPLv3 不兼容 或 非商业）

- `STD`（GPL-2.0，且商业需谈）
- `FAST-LIO-SAM` / `QN` / `SC-QN`（CC-BY-NC-SA 4.0，NonCommercial）
- `FAST_LIO_SLAM/SC-PGO`（顶层无 LICENSE，子包无 LICENSE，保守按 "All rights reserved"）
- `FAST_LIO_LC/PGO`（同上）

---

## 2. 关键发现（对比 v2 选型报告）

### 2.1 确认的结论

- **LiLoc = MIT**（v2 报告推测是 MIT，确认）
- **SOLiD = BSD-3 学术**（可读可用，商业化要谈）
- **glim = MIT**（v2 确认，且代码量 40k 行，最完整）
- **mola = GPL-3.0-only**（v2 标"未知"，现在确认 — **和我们 GPLv3 兼容，可并库**）
- **FAST-LIO-SAM 全家 = CC-BY-NC-SA**（v2 确认，只能作图纸）

### 2.2 新信息

- **mola 代码量 30k 行，但只有 3 个文件提 keyframe/gtsam** — 说明 mola 的 keyframe 抽象不在主代码，而在 sub-package（pkg 级别分散）。实际借用成本可能比想象高。
- **glim 代码量 40k 行，61 个文件提 keyframe/gtsam** — 确实是"完整系统"。代码量远高于我们项目，整包引入会吃下整个主线。适合借模块，不适合并库。
- **LiLoc 的 `<license>TODO</license>` 写在 package.xml** — 但根 LICENSE 文件是 MIT。说明作者 package.xml 没填，以根 LICENSE 为准。
- **fastlio2_ros2 根目录无 LICENSE，但 fastlio2/pgo/interface 三个子包都是 MIT** — 可以按 MIT 对待。
- **FAST-LIO-SAM 代码量 1056 行** — 这正好是我们需要的"最小可行后端图纸"尺寸。我们自己实现 C-1 估计也是这个量级。
- **FAST-LIO-SAM / QN / SC-QN 三个仓库代码量几乎一样（1056/1203/1203）** — 说明差异只在 descriptor/verifier 模块，后端主框架相同。

### 2.3 意外

- **SOLiD 占 37MB 磁盘**（包含训练用模型 checkpoint 和 demo 数据）。核心 C++ descriptor 代码其实只 269 行，非常轻。适合 vendored。
- **Bonxai 占 16MB**（含 benchmark data）。核心库更小。

---

## 3. 按主线用途分组

### 3.1 C-1 后端骨架 —— 重点读的 3 个仓库

1. **`FAST-LIO-SAM`**（engcang，CC-BY-NC-SA 只作图纸）
   - 1056 行，规模正好
   - 目录结构、keyframe/loop 线程分工、iSAM2 调用方式是首参
   - **不能并库**，我们自己重写

2. **`FAST_LIO_SLAM`**（gisbi-kim，待下）
   - sidecar 边界最干净（FAST-LIO2 与 SC-PGO 各自独立节点）
   - 看拓扑，不看代码细节

3. **`fastlio2_ros2/pgo/`**（MIT）
   - ROS2 下的 FAST-LIO2 + PGO 融合，有 MIT 许可
   - 作为"可并库参考"

### 3.2 Loop Verifier —— 直接可用

- **`small_gicp`**（MIT）— koide3 出品，质量高
- **`nano_gicp`**（MIT）— engcang 出品，FAST-LIO-SAM-QN 用的就是它
- 选一个作为 C-1 默认，不重写

### 3.3 Loop Candidate Provider

- **`SOLiD`**（BSD-3 学术）— Mid360/Livox 专项 FOV-aware descriptor，C++ 版只 269 行
- **`STD`**（GPL-2.0）— 只作图纸参考
- ScanContext（FAST-LIO-SAM-SC-QN 的 SC 部分）— 只作图纸
- 插件化接口保留，第一版默认 SOLiD

### 3.4 Relocalizer

- **`LiLoc`**（MIT）— 直接借，作为主线
- 7288 行，包含 submap 管理、topological graph、egocentric factor graph

### 3.5 Planning sidecar

- **`Bonxai`**（MPL-2.0）— 直接用，CPU-only 稀疏体素

### 3.6 长期/高级参考（读不并）

- **`glim`** — 若未来要全局一致性优化
- **`mola`** — simple-map 架构
- **`kiss-slam`** — 轻量非 GTSAM 路线的参考

---

## 4. ~~仍需你手动补下的 3 个仓库~~ 全部到位

修复方法：`git config --global http.version HTTP/1.1` + `http.postBuffer 1048576000` 后一次成功。

16 个仓库全部在 `~/refs/fast_lio_super/`。

---

## 5. 接下来可以做的三件事

A. **精读 FAST-LIO-SAM 的 main_node 和 loop thread**，提取"后端主循环的伪代码骨架"，作为 C-1 实现图纸。
B. **读 LiLoc 的 submap manager 和 factor graph 逻辑**，确认"能不能作为 relocalizer 直接整包引入"。
C. **把 small_gicp 和 nano_gicp 对齐一下**，选一个作为 loop verifier 的默认实现。

回 `A/B/C` 或组合（`AB` / `ABC`），我按顺序推进。或者让你先把 3 个失败仓库补下再继续。
