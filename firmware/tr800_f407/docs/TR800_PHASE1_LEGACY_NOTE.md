# TR800 第一阶段遗留说明

这份档案专门记录老 TR800 固件里留下来的 RC 侧副作用，方便后面回看时知道它们原来是做什么的。

## 术语

- `RELAY1`：一个 GPIO 驱动的外部继电器输出，接在 `GPIOD3`。老 RC 路径里它被用来给外部电机驱动器重新上电或者断电。
- `RELAY2`：一个 GPIO 驱动的外部继电器输出，接在 `GPIOD4`。旧代码里它曾用于灯光相关逻辑，当前第一阶段已隔离。
- `RELAY3`：一个 GPIO 驱动的外部继电器输出，接在 `GPIOD7`。旧代码里它曾和 `RELAY2` 一起用于灯光相关逻辑，当前第一阶段已隔离。
- `hand_stop`：老的停机/控制位，原来会被旧 RC 继电器逻辑写入。
- `hand_stop1`：老的刹车停机位，也是旧遥控副作用的一部分。

## 已移除内容

- 旧 RC 分支里直接用 `RC_power` 去切 `RELAY1` 的逻辑。
- 旧 RC 分支里根据摇杆状态写 `hand_stop` 和 `hand_stop1` 的逻辑。
- 旧的 `rc_idle_times / b_rc_idle / rc_stable` 兼容路径，这条路径会把旧 RC 状态机重新拉回来。
- RC 输入解析和继电器上电控制之间的直接绑定。
- `UART4` 旧 `5` 字节 `tank_mode` 小包解析，以及它和 `RELAY1` 的历史联动。
- `RC_Light` 到 `RELAY2/RELAY3` 的灯光继电器联动。

## 为什么要这样做

第一阶段的目标是让车通过两条明确输入、一条运动链路工作：

`SERIAL / RC 输入 -> TR800_Runtime -> Move_X/Y/Z -> Drive_Motor()`

只要 RC 输入还能在 `TR800_Runtime` 之外偷偷改电源或改停机位，排障就会变得很难判断。

## 当前还保留什么

- RC 作为第一阶段主输入之一仍然保留，当前还会从 `USART2` 的 `IBUS` 读取遥控值。
- OLED 显示也仍然保留，当前会显示 RC 值、运行状态、模式和电压。
- `RELAY1/2/3` 的 GPIO 定义和初始化仍保留，但第一阶段不再由 RC 或串口控制链路隐式驱动。
- `hand_stop` / `hand_stop1` / `NoControlstop` / `hall_stop` 仍保留为旧兼容输入，但停机原因已同步到 `TR800_Runtime.fault_flags`。
- 电机停机判断现在统一走 `tr800_runtime_stop_requested()`，不再继续新增散落的旧全局判断。

## 后续

- 如果第一阶段运行稳定，下一步再决定是否彻底移除旧全局变量，只保留 `TR800_Runtime.fault_flags` 作为内部事实源。
