# 模式与运行

先构建并加载 `/opt/ros/noetic/setup.bash` 和项目 `devel/setup.bash`。统一入口：

```bash
bash tools/run.sh mapping hardware rviz:=true
bash tools/run.sh localization hardware map:=/absolute/map.pcd rviz:=true
bash tools/run.sh navigation hardware map:=/absolute/map.pcd rviz:=true
```

| 模式 | 装配内容 |
| --- | --- |
| mapping | Livox、FAST_LIO、地图输出；不启动规划和底盘控制 |
| localization | Livox、FAST_LIO_LOCALIZATION、地图发布与 TF |
| navigation | 定位链、dlrobot_robot、MoveBase3D、实时点云过滤 |

`navigation` 与 `localization` 必须提供 PCD 地图。运行时通过 RViz 的初始位姿工具进行初始定位，目标及多点操作见 [MoveBase3D](../src/MoveBase3D/README.md)。mapping 默认 `save_map:=true`，正常 Ctrl+C 结束时沿用 FAST_LIO 保存逻辑写入 `src/FAST_LIO/PCD/scans.pcd`；`save_map:=false` 关闭保存，分段输出间隔沿用包内配置。

附加参数直接传给 ROS launch：`user_config_path:=/absolute/MID360_config.json`、`usart_port_name:=/dev/serial/by-id/your-chassis`、`serial_baud_rate:=115200`、`rviz:=false`。雷达网络与外参保留在对应配置中，启动脚本不修改宿主网络。

默认运行三维地图链。需要旧版二维辅助输出时，另传 `publish_2d_map:=true map_2d:=/absolute/map.yaml`；该 YAML 与 PGM 图片应来自同一张地图。

旧入口 `bash start_all.sh map:=/absolute/map.pcd` 会加载 Noetic 和项目环境，再启动 navigation/hardware。前台 roslaunch 统一管理进程，Ctrl+C 结束本轮节点。

main 的 simulation 入口会提示切换升级线，使用方法见[仿真说明](simulation.md)。
