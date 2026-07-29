# 工作空间目录治理收口：git 边界、死体系与死产物清理

> 任务目录: `.trellis/tasks/07-26-workspace-cleanup`
> 日期: 2026-07-26

## Goal

git status 归零、根目录只留活体系：收敛 git 追踪边界，移除两代死治理体系壳，
删除死构建产物，并把信息密度最高的文档纳入版本控制。

## 背景

仓库根目录存在三代 AI 治理体系叠加（`.agentdocs` → `.spec-workflow` → `.trellis`）、
四份字节级重复且未纳管的平台配置目录（`.agents` `.claude` `.codex` `.cursor`）、
以及死构建产物（`build/` 2025-03 产物、`other.zip` 与 `other/` 重复）。
git status 长期挂 31 条 dirty 路径，信息密度最高的文档（根级与包级 CLAUDE.md、
FAST_LIO_SUPER 调研报告）反而在版本控制之外。

## Requirements

1. **R1 — git 边界收敛**：git status 归零。每类未追踪路径明确二选一：
   - ignore：`.agents/` `.claude/` `.codex/` `.cursor/`（Trellis 生成、跨平台字节重复、可再生）
   - commit：根级 `CLAUDE.md` `AGENTS.md`、全部包级 `CLAUDE.md`（含 `src/global_to_local_adapter/`）、
     `src/FAST_LIO_SUPER/docs/*.md`（9 份）、`.agentdocs/research/`
1b. **R1b — 修复断裂 gitlink（执行期发现的 PRD 缺陷，2026-07-26 回补）**：
   - 事实：`src/FAST_LIO` `src/FAST_LIO_LOCALIZATION` `src/MoveBase3D` `src/livox_ros_driver2`
     四包在索引中是 mode 160000 gitlink，但无 `.gitmodules`，且各包 `.git` 已按仓库约定
     改名为 `.git-upstream-ref` → gitlink 失效，**四包源码未被任何仓库版本化**。
   - 处置：`git rm --cached` 移除 4 个 gitlink；纳管全部源码；忽略上游大体积
     媒体/数据目录：`src/FAST_LIO/PCD/`(211M) `src/FAST_LIO/doc/`(128M)
     `src/FAST_LIO/Log/` `src/FAST_LIO_LOCALIZATION/doc/`(26M)。
     MoveBase3D/doc(5.1M) 与 livox(1.1M) 体积可接受，整包纳管。
   - `src/dl_msgs/.git`：32 字节 gitdir 指针残留文件（包内 19 文件已被父仓库正常追踪），删除。
2. **R2 — 死体系移除**：
   - `git rm -r .spec-workflow/`（specs/approvals/archive/steering 全空，仅剩模板）
   - `git rm -r .runtime/`（仅 8 行孤儿 decision_trace.jsonl）
   - `.agentdocs/` 五份计划书保留但需 README 标注"一代规划，已被 .trellis 取代，仅作历史记录"
   - 删除 `.agentdocs/runtime_wp3_*.log` 两个散落日志（未追踪、被 *.log 忽略规则覆盖）
3. **R3 — 死产物清理**（仅本地磁盘，均在 .gitignore 内，不涉及 git 历史）：
   - `rm -rf build/`（CMakeCache 2025-03-29，早于 build_super 一年，已死）
   - `rm other.zip`（与 `other/` 目录内容重复）
   - 保留：`build_super/`（活跃）、`devel/`（2026-04-17，与 build_super 同期）、`other/`
4. **R4 — 文档同步**：根级 `CLAUDE.md` 追加 changelog 行记录本次治理。

## 明确不做（Out of Scope）

- ~~修剪 `.trellis/spec/frontend/`~~ — 核实后发现四个文件已被改写为"本仓库无前端"
  的守门文档，内容真实有效，保持现状（对原始提案的修正）。
- 不动 `src/` 任何源代码。
- 不动 `.trellis/` 工作流本体。
- 不删 `.agentdocs/` 计划书内容（个人工程记录/简历素材库定位，只标注不删除）。
- 不动 `other/`、`devel/`、`build_super/`。

## Acceptance Criteria

- [ ] A1: `git status --porcelain` 输出为空。
- [ ] A2: 根目录不再存在 `.spec-workflow/`、`.runtime/`、`build/`、`other.zip`。
- [ ] A3: `.gitignore` 覆盖 `.agents/` `.claude/` `.codex/` `.cursor/`。
- [ ] A4: `git ls-files` 包含根级 CLAUDE.md、AGENTS.md、20 个包级 CLAUDE.md、
  9 份 FAST_LIO_SUPER/docs 报告、.agentdocs/research/ 报告。
- [ ] A5: 本次提交只触及上述路径，`src/` 下除新增 CLAUDE.md/docs 外无源码 diff。
- [ ] A6: `.agentdocs/README.md` 存在且说明历史地位。

## 用户授权记录

- 用户已明确同意建任务并授权"彻底干净"，包括 R3 的两项不可逆本地删除
  （build/、other.zip）。删除范围以本 PRD 列出的为限，超出即停。
