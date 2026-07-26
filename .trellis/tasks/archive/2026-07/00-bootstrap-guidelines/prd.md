# PRD: Workspace Trellis Bootstrap And Inventory

## Goal

基于现有代码、`README`、各模块 `CLAUDE.md` 与已有技术文档，整理这个 ROS1 `catkin` 工作空间的结构认知，并把可复用的工程规则沉淀到 `.trellis/`，让后续 AI 会话能够按本仓库的真实约定工作，而不是按通用模板猜测。

## User Value

- 用户可以更快知道这个工作空间里有哪些 ROS 包、它们分别负责什么。
- 后续 AI 在改仓库时能优先遵循本项目已有的模块边界、入口、文档习惯与验证方式。
- 工作空间的“哪里是主线、哪里是参考/备选、哪里是消息/桥接/驱动层”会更清晰。

## Confirmed Facts

- 仓库已经存在 `.trellis/`，不是空白初始化状态；当前 Trellis 任务目录为 `.trellis/tasks/00-bootstrap-guidelines/`。
- 当前会话已将 `00-bootstrap-guidelines` 设为 current task。
- 顶层是 ROS1 Noetic `catkin` 工作空间，入口在 [src/CMakeLists.txt](/mnt/d/test_move3d/src/CMakeLists.txt:1)。
- 顶层 [README.md](/mnt/d/test_move3d/README.md:1) 已明确当前主线是温室导航工作空间，并给出建图/导航入口。
- 顶层 [CLAUDE.md](/mnt/d/test_move3d/CLAUDE.md:1) 已提供按模块分类的工作空间索引，包含两个主要工程方向：
  - 项目 A：`FAST_LIO_SUPER` 为核心的可插拔 SLAM 方向
  - 项目 B：`greenhouse_nav` + `terrain_analysis` + `far_planner` + `local_planner` + `dlrobot_robot` 为核心的温室导航方向
- `.claude/index.json` 记录了 19 个模块的扫描结果、用途摘要、入口文件与一部分许可证/语言信息，可作为模块整理索引来源。
- `src/` 下当前可见 ROS 包包括：
  - `FAST_LIO`
  - `FAST_LIO_LOCALIZATION`
  - `FAST_LIO_SUPER`
  - `boundary_handler`
  - `dl_msgs`
  - `dlrobot`
  - `dlrobot_robot`
  - `far_planner`
  - `graph_decoder`
  - `greenhouse_nav`
  - `livox_ros_driver2`
  - `livox_to_pointcloud2`
  - `local_planner`
  - `slam_octvox`
  - `terrain_analysis`
  - `terrain_analysis_ext`
  - `visibility_graph_msg`
  - `MoveBase3D`（目录下含多个子包）
- 仓库中已存在大量可复用文档来源，包括：
  - 顶层 `README.md` / `CLAUDE.md`
  - 各包内 `CLAUDE.md`
  - `FAST_LIO_SUPER/docs/*.md`
  - 多个 `package.xml`、`launch/*.launch`、`config/*.yaml`
- 用户已确认本轮优先级为：先覆盖整个工作空间的模块索引与规则基线，再对主线模块逐步细化。

## Requirements

### R1. 建立面向 Trellis 的仓库认知基线

把这个工作空间的真实结构、主线模块、辅助模块、备选模块、消息/驱动/桥接模块整理进 `.trellis/spec/`，优先描述“实际存在且已经在用”的组织方式。

### R2. 以 ROS 包为单位说明职责

对工作空间内的重要 ROS 包给出职责说明，并尽量区分：

- 生产主线
- 参考或备选方案
- 驱动 / 消息定义 / 桥接
- 研究型或底层基础设施

### R3. 记录真实入口与协作边界

需要从现有 `launch`、`README`、`CLAUDE.md`、源码入口文件中归纳出：

- 用户入口 launch
- 核心节点或脚本
- 关键 topic / TF / 数据流边界
- 模块之间的依赖关系

### R4. 规则文档必须来自真实证据

写入 `.trellis/spec/` 的内容必须优先依据真实文件，不写“理想状态”规则。若某层没有实际前端代码，应明确说明而不是编造前端约定。

### R5. 输出应支持后续 AI 修改代码

最终整理出的 Trellis spec 至少应能回答下面这些问题：

- 在这个仓库里，改某类问题应先看哪些包？
- 哪些 launch / 脚本是主入口？
- 哪些包是消息、驱动、桥接、规划、建图、定位、执行？
- 当前验证方式更接近 `catkin_make` / `roslaunch --nodes` / 手工 smoke test，而不是自动化测试。

## Acceptance Criteria

- `.trellis/spec/backend/` 中的相关文档被填充为基于本仓库实际情况的内容，而不是空模板。
- 文档中能看出当前工作空间至少包含两条主要工程脉络：`FAST_LIO_SUPER` SLAM 方向与 `greenhouse_nav` 温室导航方向。
- 文档中能说明主要 ROS 包的职责分类，并覆盖顶层主线模块与至少一组参考/备选模块。
- 文档中引用了真实文件路径作为证据来源。
- 若前端层与仓库现状不匹配，spec 会明确这一点，不伪造前端工程规范。

## Out Of Scope

- 修改任何 ROS 算法实现、launch 逻辑或运行参数。
- 引入新的依赖、测试框架或 CI。
- 把所有历史/研究文档重写成统一风格的大型手册。
- 对尚未使用的模块做过度结论化整理。
