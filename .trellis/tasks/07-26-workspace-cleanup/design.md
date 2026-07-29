# Design — 工作空间目录治理收口

## 核心判断（证据基础）

| 对象 | 证据 | 处置 |
|------|------|------|
| `.agents/.claude/.codex/.cursor` | 0 文件被追踪；`trellis-check/SKILL.md` 三处 md5 相同（`1747e65d…`）；由 Trellis 步骤 1.3 生成、`.template-hashes.json` 管理 | gitignore（可再生的生成物） |
| `.spec-workflow/` | specs/approvals/archive/steering 四目录全空，仅 6 模板 + 1 README；tracked 文档零引用 | git rm 整目录 |
| `.runtime/` | 仅 8 行 decision_trace.jsonl，2026-04-13 某 search orchestrator 遗留；零引用 | git rm 整目录 |
| `.agentdocs/` 五份计划书 | 一代规划（2026-04），有简历素材价值；`research/` 报告被根 CLAUDE.md changelog 引用 | 保留 + README 标注历史地位；research/ 纳入追踪 |
| `build/` | CMakeCache 2025-03-29；`build_super/`+`devel/` 均为 2026-04-17 同期，活跃链路不依赖它 | rm -rf（本地，gitignore 已覆盖） |
| `other.zip` | 76M，与 435M `other/` 目录重复 | rm（本地，gitignore 已覆盖） |
| `.trellis/spec/frontend/` 四文件 | 已被改写为"无前端"守门文档，非模板残留 | **不动**（修正原提案） |
| 根级/包级 CLAUDE.md、FAST_LIO_SUPER/docs | 信息密度最高的文档，当前未追踪 | git add + commit |

## 平台目录 ignore 而非 commit 的理由

- 三平台 skills 字节级重复，commit 会把同一内容三份写进历史；
- `.trellis/`（源头，71 文件已追踪）+ Trellis 1.3 可随时再生平台配置；
- 单人仓库，无协作者需要同步平台 hooks。

回滚：若日后发现某平台配置需版本化，从 `.gitignore` 删掉对应行再 add 即可，无历史损失。

## 提交拓扑（2 个功能提交，本地删除不产生提交）

```
commit 1  chore(git): 收敛追踪边界 — ignore 平台配置目录，纳管 CLAUDE.md/调研文档
          .gitignore + CLAUDE.md + AGENTS.md + src/**/CLAUDE.md(20)
          + src/FAST_LIO_SUPER/docs/*.md(9) + .agentdocs/research/
commit 2  chore(cleanup): 移除死体系 .spec-workflow/.runtime，标注 .agentdocs 历史地位
          git rm -r .spec-workflow .runtime + 新增 .agentdocs/README.md
          + CLAUDE.md changelog 行（R4）
（无提交）rm -rf build/ && rm other.zip && rm .agentdocs/runtime_wp3_*.log
（自动）  Trellis add_session 的 journal 提交
```

CLAUDE.md 的 changelog 更新放 commit 2：commit 1 让文件先以现状入库，
commit 2 记录治理动作本身，diff 语义干净。

## 风险与回滚

- commit 1/2 均为纯 git 操作，可 `git revert` 回滚。
- `rm -rf build/` 不可逆但可再生：`catkin_make` 全量重建即可（代价是一次编译时间）。
- `other.zip` 不可逆但内容与 `other/` 重复，`other/` 保留。
- 死守边界：任何 `rm`/`git rm` 目标必须与 PRD 列表逐字匹配，出现意外路径立即停。
