# 版本对应的运行环境

main 使用 ROS 1 Noetic 主机工作区，构建见 [docs/build.md](../docs/build.md)。本分支没有套用 ROS 2 Dockerfile。

ROS 2 生产镜像、Gazebo Harmonic、PX4 工具链及 Compose 配置保留在 upgrade/ros2。切换该分支后使用其 docker/README.md 和 docs/simulation.md，配置与源码一起按版本切换。
