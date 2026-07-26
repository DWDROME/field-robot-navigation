# PRD: Workspace Overview Doc

## Goal

在仓库根部生成一份 `OVERVIEW.md` 总方案文档，回答四个问题：

1. 我们用了哪些包（以及包之间怎么调用）
2. 每个包的上游 GitHub 地址是什么
3. 有没有对包进行魔改，改了什么
4. 整体技术路线是什么

明确不写：接口签名、命名规范、topic/TF 细节 —— 这些已在 `.trellis/spec/backend/` 覆盖。

## Requirements

### R1. 单文件、简短

只产出根目录 `OVERVIEW.md` 一份，不建立文档树。

### R2. 包清单表

按功能分组列出使用的 ROS 包，每行包含：包名 / 上游 GitHub 地址（或"自研"/"vendored 来源"）/ 是否魔改 / 改动一句话摘要。

### R3. 技术路线

用简洁的数据流图（文本或 Mermaid）描述两条主线：

- SLAM 线：Mid360 → FAST-LIO2 → IMapManager 双后端（ikd-Tree / OctVox）
- 导航线：驱动 → 里程计 → 定位 → 地形 → 全局/局部规划 → 底盘

### R4. 证据来源真实

上游地址与魔改结论必须来自仓库内真实证据（`CLONE_AUDIT.md`、各包 `CLAUDE.md`、`package.xml`、源码 README/头注释、git 历史）。查不到确切上游地址时写"待确认"，不编造 URL。

## Acceptance Criteria

- [ ] 根目录存在 `OVERVIEW.md`，长度约 100-200 行
- [ ] 覆盖两条主线所有生产包 + 主要备选包（MoveBase3D、dlrobot 至少提及）
- [ ] 每个上游包给出 GitHub 地址或明确标注"待确认"
- [ ] 魔改列只写"改了什么"，不写接口/命名细节
- [ ] 不修改任何源码

## Out Of Scope

- 接口文档、topic/TF 契约（已在 .trellis/spec/backend/）
- 修改源码或 launch
- 多文件文档树
