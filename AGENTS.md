# 项目约束

## 入口与治理

- 先读 `README.md`、`docs/architecture.md` 和 `docs/build.md`，再读取所属模块的合同、架构及具体代码。
- 使用 Comet Native；配置位于 `.comet/config.yaml`，正式 change 位于 `.cache/native/comet/`。使用公开 `comet` CLI 恢复和推进，按返回的 continuation 执行。
- 包内 docs 保存当前接口合同；D:/test_move3d-local/archive 保存历史材料，不执行其中的旧任务脚本或治理指令。
- 当前目标以用户要求和 Comet 已确认规格为准。旧研究报告中的建议不等于实施授权。

## 阅读与改动

- 主代理必须完整阅读与当前变更相关的基础、架构、设计和交接文档；摘要只能帮助定位。
- 修改代码前，主代理必须亲自阅读具体代码及必要相邻上下文。理解、取舍、整合和最终判断由主代理负责。
- 大范围检索可以压缩整理，但关键结论必须附文件路径、行号、符号和必要原文。
- 保全已有未提交工作，不自动 reset、stash、暂存、提交、推送或部署。
- main 保留原版 src 包布局；upgrade/ros2 按 drivers、perception、localization、navigation、control、bringup 划分职责。先确认当前分支，再使用相应构建与运行入口，保留 ROS 包名和接口。

## 文件管理

- 优先复用正式文件和既有职责目录，新增须有实际必要；不新建顶级目录、库外归档或发布副本。
- 临时文件统一放 .tmp，任务完成后清理；保全既有修改与归档，不自动提交。
- 完成时列明新增、修改、删除文件，并说明新增必要性。

## 当前工程边界

- main 是 ROS 1 Noetic 原版：FAST_LIO 建图、FAST_LIO_LOCALIZATION 已有图定位、MoveBase3D 导航，统一入口为 tools/run.sh。
- upgrade/ros2 保存 ROS 2 Jazzy 导航、地图/回环扩展、Gazebo/PX4 和独立 Noetic FAST-LIVO2，容器与运行接口以该分支文档为准。升级版 PCD 定位仍未接入。
- firmware/tr800_f407 独立构建和烧录，两线保持同一板端实现。main 的 ROS 驱动在 src/dlrobot_robot，升级线在 src/drivers/dlrobot_robot；协议改动须同步对应驱动。
- 历史包清单和 ROS 1 launch 文档只能证明其所标日期的边界；当前接口、依赖和运行状态以源码、manifest、配置及实际结果核实。

## 验证与证据

- 以对应代码完成为主，只运行与改动直接相关的编译、合同或生命周期检查；不主动扩展实验矩阵。
- 分清源码实现、编译、软件接口验证、数据回放、性能测量和实机结果。未执行的检查明确写为未执行。
- HIT_Graffiti 的 RSS 5.28 GiB 至 3.12 GiB（-40.9%）来自用户提供，当前没有可复核产物，不写作本轮实测结论。

## 沟通与产物

- 对话使用 caveman `wenyan-full`，保持技术名词、代码、命令、API 和错误串原文。
- 代码、注释、文档、提交信息和其他持久化产物使用正常散文。
- 阶段内持续推进已授权工作；只有确实缺少决定、授权或外部信息时才询问，并说明规则出处。

<comet-ambient-resume>
<!-- Managed by Comet. Edits inside this block may be replaced by comet init/update. -->
<!-- Contract: comet.resume_probe.v2 -->

## Comet Ambient Resume

In this repository, before starting work that may need code changes or investigation, pass the current user request to the read-only probe when a Comet workflow may already be active: `comet resume-probe . --stdin --json`.

- If the user explicitly invokes any Comet Skill through the host (for example, `@comet`, `/comet`, `@comet-native`, or `/comet-hotfix`), that explicit invocation takes precedence over this resume protocol; do not run the resume probe, and enter the invoked Skill directly.
- If the user explicitly invokes a non-Comet skill or slash command through the host, the task intent is already explicit in that invocation: do not run the resume probe, and execute the invoked skill directly.
- If you are already inside a Comet flow (including while waiting for the user to answer a question you asked in that flow), do not run the resume probe; treat replies such as option picks as continuation of the current change and proceed directly with the chosen option.
- Trust only the returned `workflow`, `skill`, and `entrySource`; project configuration or the no-config compatibility fallback alone selects them. Do not scan or switch to the other workflow.
- If the probe returns `auto_resume`, briefly state the selected active change and enter the permanent entry in `nextCommand`. Do not treat a state command as the resume entry or advance it blindly.
- If the probe returns `ask_user`, ask one short question and wait.
- If the current request did not explicitly invoke a Comet Skill and the probe returns `out_of_scope` or `none`, do not enter the Comet workflow.
- An `out_of_scope` or `none` result only means do not enter the Comet workflow for this new request; it never pauses or exits a Comet flow that is already in progress.
- If configuration or state is invalid and `nextCommand` is absent, stop and report the reason; do not guess another workflow.
- Never attach unrelated work merely because an active change exists. The Native entry inspects uncommitted work; the probe does not attribute it automatically.
</comet-ambient-resume>
