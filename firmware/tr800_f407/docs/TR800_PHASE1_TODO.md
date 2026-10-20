# TR800 第一阶段 TODO

这份清单只记录“当前代码里还真实存在、后面确实还要处理”的遗留项。

## 当前判断

下面这些不再作为第一阶段 TODO：

- `HARDWARE/timer.c`
  当前已经只保留输入捕获，不再负责控制源切换。
- `HARDWARE/can.c`
  当前 `CAN1_RX0_INT_ENABLE` 已经关闭，CAN RX 中断不再进入第一阶段主控。
- 旧的 `rc_idle_times / b_rc_idle / rc_stable`
  这条兼容路径已经移除，不需要再继续追。

## 已完成

- [x] 重新定义 `hand_stop / hand_stop1 / NoControlstop / hall_stop` 的归属。
  当前仍保留这些旧全局变量作为兼容输入，但停机原因已统一同步到 `TR800_Runtime.fault_flags`。
  电机停机判断统一走 `tr800_runtime_stop_requested()`，避免后续继续扩散散落判断。

- [x] 移除 `UART4` 旧 `5` 字节 `tank_mode` 小包与 `tank_mode -> RELAY1` 副作用路径。
  `UART4` 第一阶段只解析 `11` 字节 SERIAL 速度包；`RC_task` 不再根据 `tank_mode` 操作 `RELAY1`。

- [x] 隔离 `RC_Light -> RELAY2 / RELAY3` 这条旧副作用路径。
  `RC_Read()` 不再根据 RC 灯光通道操作 `RELAY2/RELAY3`。

## 现在真正剩下的 TODO

- [ ] 决定 `HARDWARE/pstwo.c` 的最终去留。
  现在它已经不参与第一阶段构建，但代码文件还在。
  后面要决定是作为历史留档保留，还是直接移出当前主工程。

- [ ] 在实车验证稳定后，清掉剩余 RC 兼容全局和显示兼容位。
  当前还保留了一批老 RC 全局，例如 `RC_power / RC_navigate / RC_slam / RC_reboot / RC_Light`。
  其中一部分已经不是第一阶段主路径必须项，需要在跑车验证完成后再收一轮。

- [ ] 在功能稳定后补一轮编译告警清理。
  当前 GCC 构建已经能出 `elf/hex/bin`，但还有一些历史告警：
  - `rc_loop_control` 的显式声明问题
  - `NULL` 重定义
  - `OLED_ShowString` 的 signedness 告警
  这些不阻塞第一阶段跑车，但后面应该收掉。

## 当前第一阶段约定

- 主线事实表：`TR800_PHASE1_MAINLINE.md`
- SERIAL 输入：`UART4`
- 遥控输入：`USART2` 的 `IBUS`
- 控制状态：`BOOT_DELAY / READY / RUN / FAULT`
- 外部控制入口：`SERIAL / RC`
- 运动控制：统一走一条速度闭环主路径

## 遗留说明

继电器和停机位的背景见 [TR800_PHASE1_LEGACY_NOTE.md](TR800_PHASE1_LEGACY_NOTE.md)。
