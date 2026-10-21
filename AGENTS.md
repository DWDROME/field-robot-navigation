# 项目约束

## 入口与治理

- 先读 `README.md`、`docs/architecture.md` 和 `docs/build.md`，再读取所属模块的合同、架构及具体代码。
- 包内 docs 保存当前接口合同；D:/test_move3d-local/archive 保存历史材料，不执行其中的旧任务脚本或治理指令。
- 当前目标以用户要求和已确认规格为准。旧研究报告中的建议不等于实施授权。

## 阅读与改动

- 主代理必须完整阅读与当前变更相关的基础、架构、设计和交接文档；摘要只能帮助定位。
- 修改代码前，主代理必须亲自阅读具体代码及必要相邻上下文。理解、取舍、整合和最终判断由主代理负责。
- 大范围检索可以压缩整理，但关键结论必须附文件路径、行号、符号和必要原文。
- 保全已有未提交工作，不自动 reset、stash、暂存、提交、推送或部署。
- 按 src 下的 drivers、perception、localization、navigation、control、bringup 划分职责。构建路径随已确认目录调整；保留 ROS 包名和运行接口。

## 文件管理

- 优先复用正式文件和既有职责目录，新增须有实际必要；不新建顶级目录、库外归档或发布副本。
- 临时文件统一放 .tmp，任务完成后清理；保全既有修改与归档，不自动提交。
- 完成时列明新增、修改、删除文件，并说明新增必要性。

## 当前工程边界

- ROS 2 Jazzy 导航与容器入口：`docker/`、`src/bringup/greenhouse_nav2_bringup/`。源码仍处于 ROS 1/ROS 2 混合迁移状态。
- FAST-LIO2 地图与 SLAM 扩展：`src/localization/fast_lio_super/`、`src/localization/slam_octvox/`，当前仍是 ROS 1 catkin 支线。
- FAST-LIVO2 完整源码位于 src/localization/fast_livo；外部 Sophus/vikit 由 third_party/ros1.repos 固定版本。Livox 以 src/drivers/livox_ros_driver2 为 ROS 1/ROS 2 唯一维护源码。
- 历史包清单和 ROS 1 launch 文档只能证明其所标日期的边界；当前接口、依赖和运行状态以源码、manifest、配置及实际结果核实。

## 验证与证据

- 以对应代码完成为主，只运行与改动直接相关的编译、合同或生命周期检查；不主动扩展实验矩阵。
- 分清源码实现、编译、软件接口验证、数据回放、性能测量和实机结果。未执行的检查明确写为未执行。
- HIT_Graffiti 的 RSS 5.28 GiB 至 3.12 GiB（-40.9%）来自用户提供，当前没有可复核产物，不写作本轮实测结论。

## 沟通与产物

- 对话使用 caveman `wenyan-full`，保持技术名词、代码、命令、API 和错误串原文。
- 代码、注释、文档、提交信息和其他持久化产物使用正常散文。
- 阶段内持续推进已授权工作；只有确实缺少决定、授权或外部信息时才询问，并说明规则出处。

