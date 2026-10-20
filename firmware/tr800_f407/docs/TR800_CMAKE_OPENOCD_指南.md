# TR800 CMake + OpenOCD 说明

这份说明对应第一版现代化构建入口。

## 已对齐的本地工具

当前构建入口会优先使用完整工具链：

- `D:\DevEnv\GNU-tools-for-STM32`

如果这套不存在，再回退到 `D:\ST` / `C:\ST` 下的官方组件。

当前 `D:\ST` 下已经有现成官方组件，不需要再额外下载一套：

- `STM32CubeIDE`
- `arm-none-eabi-gcc.exe`
- `ninja.exe`
- `openocd.exe`
- `STM32_Programmer_CLI.exe`

对应路径已经写入：

- `CMakePresets.json`
- `cmake/toolchains/tr800_stm32f407.cmake`
- `scripts/windows/openocd_connect.bat`

## 构建方式

首选方式：

```bat
cd /d D:\test_move3d\test_move3d\other\当前开发主线\TR800重写工作区\TR800_F407
C:\ST\STM32CubeCLT_1.19.0\CMake\bin\cmake.exe -G Ninja -S . -B build\tr800-st-preset-win -DCMAKE_TOOLCHAIN_FILE=cmake\toolchains\tr800_stm32f407.cmake -DCMAKE_BUILD_TYPE=Debug
C:\ST\STM32CubeCLT_1.19.0\CMake\bin\cmake.exe --build build\tr800-st-preset-win -- -j4
```

构建产物：

- `build/tr800-st-preset-win/dlrobot.elf`
- `build/tr800-st-preset-win/dlrobot.hex`
- `build/tr800-st-preset-win/dlrobot.bin`
- `build/tr800-st-preset-win/TR800_F407.map`

当前已验证通过的干净构建目录：

- `build/tr800-st-preset-win`

注意：

- 早期试验目录如果是在本次修正前生成的，可能仍然缓存旧工具链路径，不建议直接复用。
- 这次已经修正 `.data` 的加载地址，`dlrobot.bin` 现在应为约 `52K`，不再出现百兆级错误产物。
- `CMakePresets.json` 仍然保留，后续可以继续整理；当前优先使用上面这组已验证的显式命令。

## OpenOCD 方式

DAPLink 按 `CMSIS-DAP` 处理，使用 SWD。

连接测试脚本：

```bat
scripts\windows\openocd_connect.bat
```

实际烧录脚本：

```bat
scripts\windows\openocd_flash.bat
```

说明：

- 两个脚本都支持优先读取 `OPENOCD_EXE` 和 `OPENOCD_SCRIPTS`
- 如果没有手工设置，会自动扫描 `D:\ST\STM32CubeIDE_*` 和 `C:\ST\STM32CubeIDE_*`
- `openocd_connect.bat` 只做连接和复位检查
- `openocd_flash.bat` 默认烧录 `build\tr800-st-preset-win\dlrobot.elf`

## 说明

- 现有 Keil 工程是 `STM32F407VETx`
- linker script 按 `512K Flash + 128K RAM + 64K CCMRAM` 配置
- 启动文件已经换成 GCC 可用的 `startup_stm32f407vetx.s`
- `HARDWARE/pstwo.c` 没有放进 CMake 主构建
