# 来源与外部依赖

main 保留既有 ROS 1 工程的 FAST_LIO、FAST_LIO_LOCALIZATION、MoveBase3D、Livox、点云工具与底盘包，包内原始源码、许可和使用声明一并保留。原版算法未因版本组织改写。

Livox-SDK2 独立安装，固定源码见 [common.repos](common.repos)。保留升级线已使用的两个 `<cstdint>` 头文件补丁，准备方式如下：

```bash
mkdir -p .tmp/deps
vcs import .tmp/deps < third_party/common.repos
for patch in "$PWD"/third_party/patches/livox_sdk2/*.patch; do
  git -C .tmp/deps/livox_sdk2 apply "$patch"
done
```

随后按 SDK 的 CMake 安装说明构建。ROS 侧驱动源码始终位于 src/livox_ros_driver2。

升级版 GTSAM、Sophus、vikit、Jazzy mapper、FAR、Clearpath、虚拟玉米田和 PX4 的版本清单与补丁完整保存在 upgrade/ros2。各分支使用自己的依赖清单和构建入口。

包内 LICENSE、版权头和 README 使用声明各自有效，MoveBase3D 的原始使用约束见其 [README](../src/MoveBase3D/README.md)。
