# 实施计划

## 1. 准备

- [x] 确认 `/opt/ros/jazzy`、`colcon` 和 ROS 2 `serial` 包可用。
- [x] 读取原包协议、姿态计算、参数、Topic 和 launch 的确切实现。
- [x] 记录迁移前 Git 状态，不触碰无关用户文件。

## 2. 迁移和实现

- [x] 将 `src/dlrobot_robot` 完整移动到 `other/历史归档/ros1/dlrobot_robot`。
- [x] 在 `src/dlrobot_robot` 创建 `ament_cmake` 包。
- [x] 实现固定长度协议编码、解码和流式重同步。
- [x] 迁移原姿态计算和里程计发布行为。
- [x] 实现订阅 `geometry_msgs/msg/TwistStamped` 的 ROS 2 串口节点。
- [x] 实现 `base_serial.launch.py`。
- [x] 添加简洁 README。
- [x] 添加协议测试。

## 3. Spec

- [x] 新增 ROS 2 机器人 C++ 代码风格规范并加入 backend 索引。
- [x] 更新包清单、目录结构、构建和 ROS 接口相关 spec 中的 `dlrobot_robot` 事实。

## 4. 验证

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/dlrobot-log build \
  --base-paths src/dlrobot_robot \
  --build-base /tmp/dlrobot-build \
  --install-base /tmp/dlrobot-install \
  --packages-select dlrobot_robot

source /tmp/dlrobot-install/setup.bash
colcon --log-base /tmp/dlrobot-test-log test \
  --base-paths src/dlrobot_robot \
  --build-base /tmp/dlrobot-build \
  --install-base /tmp/dlrobot-install \
  --packages-select dlrobot_robot

colcon test-result --test-result-base /tmp/dlrobot-build --verbose
python3 -m py_compile src/dlrobot_robot/launch/base_serial.launch.py
git diff --check
```

- [x] 构建通过。
- [x] 协议测试通过。
- [x] launch 语法检查通过。
- [x] 检查最终 diff，无隐藏 fallback、双重串口所有者或无关改动。

验证使用 CMU 官方 `serial` 源码构建临时 underlay；系统未全局安装该包。

## 5. 回滚点

- 迁移后原 ROS 1 包只存在于归档路径。
- 若 ROS 2 构建失败，先修复新包，不在原路径恢复双实现。
- 不删除归档，不自动修改 STM32 或导航算法。
