# Implement — 工作空间目录治理收口

> 执行方式：主循环内联执行（不 dispatch trellis-implement）。
> 理由：本任务全部是不可逆 git/文件系统操作，无代码编写；主循环已持有全部
> 核实过的证据（md5、时间戳、追踪清单），拆给子代理反而引入误解风险。

## Step 1 — .gitignore 收敛（R1-ignore）

- [ ] 在 `.gitignore` 新增段落：`.agents/` `.claude/` `.codex/` `.cursor/`
- 验证：`git check-ignore .agents .claude .codex .cursor` 全部命中

## Step 2 — 文档纳管（R1-commit）

- [ ] `git add CLAUDE.md AGENTS.md src/*/CLAUDE.md src/FAST_LIO_SUPER/docs/*.md ".agentdocs/research/"`
- 注意 `src/global_to_local_adapter/` 整目录未追踪，add 其 CLAUDE.md
- [ ] `git add .gitignore` 后提交 commit 1
- 验证：`git status --porcelain` 只剩 .trellis 任务工件（随 journal 提交）

## Step 3 — 死体系移除（R2）

- [ ] `git rm -r .spec-workflow .runtime`
- [ ] `rm .agentdocs/runtime_wp3_bridge.log .agentdocs/runtime_wp3_roscore.log`
- [ ] 新增 `.agentdocs/README.md`（标注：一代规划文档，2026-04，已被 .trellis 取代，
      仅作历史记录；runtime/ 为 far_planner 运行诊断，被 .gitignore 忽略）
- [ ] CLAUDE.md 追加 changelog 行（R4，日期 2026-07-26）
- [ ] 提交 commit 2
- 验证：`ls -d .spec-workflow .runtime` 均报不存在

## Step 4 — 死产物删除（R3，不可逆，逐字核对路径）

- [ ] 删除前复核：`stat build/CMakeCache.txt`（须为 2025-03）；`ls other/`（须非空）
- [ ] `rm -rf /mnt/d/test_move3d/build`
- [ ] `rm /mnt/d/test_move3d/other.zip`
- 验证：`du -sh` 前后对比，预计释放 ~345M

## Step 5 — 全量验收（对应 PRD A1–A6）

```bash
git status --porcelain                    # A1: 空（.trellis 任务文件除外，随 journal 收口）
ls -d .spec-workflow .runtime build other.zip 2>&1   # A2: 全部 No such file
git check-ignore .agents .claude .codex .cursor       # A3: 4 行输出
git ls-files | grep -c "CLAUDE.md"                    # A4: ≥21（根 1 + 包级 20）
git show --stat HEAD HEAD~1                           # A5: 路径逐一核对
test -f .agentdocs/README.md && head .agentdocs/README.md  # A6
```

## Step 6 — Phase 3 收尾

- [ ] 3.3 spec 更新：检查 `.trellis/spec/backend/runtime-artifacts.md` 与
      `directory-structure.md` 是否提及 build/、.spec-workflow、.runtime，同步修正
- [ ] 3.4 提交（若 spec 有改动则并入或单独 chore(spec) 提交）
- [ ] task.py finish + archive，add_session 记 journal

## 回滚点

- Step 2/3 之后：`git revert HEAD` 即可。
- Step 4 之后无法回滚（build/ 可由 catkin_make 再生；other.zip 内容存于 other/）。
