# TR800_F407

这是 TR800 底盘第一阶段重写工作区，基于原始 F407 Keil 工程整理，当前目标是先把车稳定跑起来，再逐步清理历史遗留功能。

## 当前状态

- 主工程目录已经统一为 `TR800_F407`
- 第一阶段主路径已收口为：
  - `UART4`：串口主输入
  - `USART2`：IBUS 遥控输入
  - `SERIAL / RC`：两种外部控制入口
  - 统一速度闭环
- 已补通 `CMake + GCC + Ninja + OpenOCD` 现代化构建链
- 当前可稳定生成：
  - `dlrobot.elf`
  - `dlrobot.hex`
  - `dlrobot.bin`

## 目录说明

- `BALANCE/`：控制、状态机、显示相关
- `HARDWARE/`：串口、电机、编码器、ADC、IMU、OLED 等底层
- `SYSTEM/`：系统与串口基础封装
- `USER/`：启动入口、中断、系统时钟
- `cmake/`：GCC 启动文件、linker script、toolchain
- `openocd/`：DAPLink / OpenOCD 配置
- `scripts/`：Windows 下的连接辅助脚本
- `docs/`：阶段说明、TODO、遗留说明

## Windows 编译

已验证可用的构建方式：

```bat
cd /d <仓库根>\firmware\tr800_f407
C:\ST\STM32CubeCLT_1.19.0\CMake\bin\cmake.exe -G Ninja -S . -B build\tr800-st-preset-win -DCMAKE_TOOLCHAIN_FILE=cmake\toolchains\tr800_stm32f407.cmake -DCMAKE_BUILD_TYPE=Debug
C:\ST\STM32CubeCLT_1.19.0\CMake\bin\cmake.exe --build build\tr800-st-preset-win -- -j4
```

构建产物默认在：

- `build/tr800-st-preset-win/dlrobot.elf`
- `build/tr800-st-preset-win/dlrobot.hex`
- `build/tr800-st-preset-win/dlrobot.bin`

## 烧录方式

当前按 `DAPLink + OpenOCD + CMSIS-DAP + SWD` 处理。

脚本会优先使用你手工设置的：

- `OPENOCD_EXE`
- `OPENOCD_SCRIPTS`

如果没有手工设置，脚本会自动扫描 `D:\ST\STM32CubeIDE_*` 和 `C:\ST\STM32CubeIDE_*` 下的常见安装目录。

先做连接测试：

```bat
scripts\windows\openocd_connect.bat
```

再烧录：

```bat
scripts\windows\openocd_flash.bat
```

## 说明

- 推荐优先烧录 `elf`，不要先用 `bin`
- 旧的试验性 `build/` 目录可能缓存了历史路径，不建议直接复用
- 当前仍有一批编译警告，但不影响生成固件
- 如果对方环境比较特殊，也可以先手工设置：
  - `set OPENOCD_EXE=实际的openocd.exe路径`
  - `set OPENOCD_SCRIPTS=实际的st_scripts路径`
  - 然后再运行 `openocd_connect.bat` 或 `openocd_flash.bat`

## 进一步资料

- 第一阶段主线事实表：`docs/TR800_PHASE1_MAINLINE.md`
- 构建与 OpenOCD 说明：`docs/TR800_CMAKE_OPENOCD_指南.md`
- 第一阶段遗留说明：`docs/TR800_PHASE1_LEGACY_NOTE.md`
- 第一阶段 TODO：`docs/TR800_PHASE1_TODO.md`

## 与 ROS 驱动的协议绑定

main 的 ROS 1 驱动位于 `src/dlrobot_robot/include/dlrobot_robot.h`，upgrade/ros2 的 ROS 2 驱动位于 `src/drivers/dlrobot_robot/include/dlrobot_robot/protocol.hpp`。两线保留同一板端实现并使用同一帧格式，协议改动须同步相应 ROS 驱动。

| 常量 | 固件 `HARDWARE/usartx.h` | main 驱动 `include/dlrobot_robot.h` |
| --- | --- | --- |
| 帧头 | `FRAME_HEADER 0x7B` | `FRAME_HEADER 0X7B` |
| 帧尾 | `FRAME_TAIL 0x7D` | `FRAME_TAIL 0X7D` |
| 命令帧（上位机→车） | `RECEIVE_DATA_SIZE 11` | `SEND_DATA_SIZE 11` |
| 遥测帧（车→上位机） | `SEND_DATA_SIZE 24` | `RECEIVE_DATA_SIZE 24` |

`HARDWARE/usartx.h` 里的 `PROTOCOL_25` 会把遥测帧改成 25 字节，该开关在 `robot_select_init.h` 中定义为 `0`，**必须保持关闭**；驱动按 24 字节解析。

当前 `TR800_MOTOR_TELEMETRY_DEBUG=0`、`TR800_CONTROL_TELEMETRY_DEBUG=1`，遥测 `8..19` 字节是三页控制诊断，不是 IMU。两线 ROS 驱动保留轮速里程计与电压，不对这些字节做 IMU 换算、姿态解算或发布；导航使用 Livox IMU。

核对当前诊断可用 `python tools/tr800_serial_analyzer.py --port COM4 --diag-control`（设备名按现场修改）。字段与页号见 [通信协议](docs/TR800_通信协议.md)。本轮仅调整上位机和文档，固件实现与编译开关保留。
