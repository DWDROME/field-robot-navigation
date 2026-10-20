cmake_minimum_required(VERSION 3.21)

project(TR800_F407 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TR800_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(TR800_LINKER_SCRIPT "${TR800_ROOT}/cmake/linker/tr800_stm32f407.ld")
set(TR800_STARTUP_FILE "${TR800_ROOT}/cmake/startup/startup_stm32f407vetx.s")
set(TR800_ELF_TARGET "${PROJECT_NAME}_elf")

add_executable(${TR800_ELF_TARGET})

target_sources(${TR800_ELF_TARGET} PRIVATE
  "${TR800_STARTUP_FILE}"
  "USER/main.c"
  "USER/stm32f4xx_it.c"
  "USER/system_stm32f4xx.c"
  "HARDWARE/adc.c"
  "HARDWARE/can.c"
  "HARDWARE/encoder.c"
  "HARDWARE/key.c"
  "HARDWARE/LED.C"
  "HARDWARE/motor.c"
  "HARDWARE/oled.c"
  "HARDWARE/timer.c"
  "HARDWARE/usartx.c"
  "HARDWARE/stmflash.c"
  "HARDWARE/I2C.c"
  "HARDWARE/MPU6050.c"
  "SYSTEM/delay/delay.c"
  "SYSTEM/sys/sys.c"
  "SYSTEM/usart/usart.c"
  "FWLIB/src/misc.c"
  "FWLIB/src/stm32f4xx_gpio.c"
  "FWLIB/src/stm32f4xx_rcc.c"
  "FWLIB/src/stm32f4xx_syscfg.c"
  "FWLIB/src/stm32f4xx_usart.c"
  "FWLIB/src/stm32f4xx_adc.c"
  "FWLIB/src/stm32f4xx_can.c"
  "FWLIB/src/stm32f4xx_tim.c"
  "FWLIB/src/stm32f4xx_pwr.c"
  "FWLIB/src/stm32f4xx_flash.c"
  "FreeRTOS/croutine.c"
  "FreeRTOS/event_groups.c"
  "FreeRTOS/list.c"
  "FreeRTOS/queue.c"
  "FreeRTOS/tasks.c"
  "FreeRTOS/timers.c"
  "FreeRTOS/portable/GCC/ARM_CM4F/port.c"
  "FreeRTOS/portable/MemMang/heap_4.c"
  "BALANCE/system.c"
  "BALANCE/balance.c"
  "BALANCE/robot_select_init.c"
  "BALANCE/show.c"
  "BALANCE/BiasAdjust.c"
  "BALANCE/mavlink.c"
)

set_source_files_properties(
  "HARDWARE/LED.C"
  PROPERTIES LANGUAGE C
)

target_include_directories(${TR800_ELF_TARGET} PRIVATE
  "USER"
  "CORE"
  "FWLIB/inc"
  "FreeRTOS/include"
  "FreeRTOS/portable/GCC/ARM_CM4F"
  "BALANCE"
  "HARDWARE"
  "SYSTEM/delay"
  "SYSTEM/sys"
  "SYSTEM/usart"
)

if(EXISTS "${TR800_ROOT}/ICM20948")
  target_include_directories(${TR800_ELF_TARGET} PRIVATE
    "ICM20948"
  )
endif()

target_compile_definitions(${TR800_ELF_TARGET} PRIVATE
  STM32F40_41xxx
  USE_STDPERIPH_DRIVER
  __FPU_PRESENT=1
  __TARGET_FPU_VFP
  ARM_MATH_CM4
  HSE_VALUE=8000000
)

target_compile_options(${TR800_ELF_TARGET} PRIVATE
  $<$<COMPILE_LANGUAGE:C>:-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -fno-common -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare>
  $<$<COMPILE_LANGUAGE:ASM>:-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard>
)

target_link_options(${TR800_ELF_TARGET} PRIVATE
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  "--sysroot=${TR800_GNU_SYSROOT}"
  "-B${TR800_GNU_GCC_LIB_DIR}"
  "-L${TR800_GNU_GCC_MULTILIB_DIR}"
  "-L${TR800_GNU_GCC_LIB_DIR}"
  "-L${TR800_GNU_NEWLIB_MULTILIB_DIR}"
  "-L${TR800_GNU_SYSROOT}/lib"
  -specs=nano.specs
  -specs=nosys.specs
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map"
  "-T${TR800_LINKER_SCRIPT}"
)

target_link_libraries(${TR800_ELF_TARGET} PRIVATE m)
set_target_properties(${TR800_ELF_TARGET} PROPERTIES
  OUTPUT_NAME "dlrobot"
  SUFFIX ".elf"
)

if(CMAKE_OBJCOPY)
  add_custom_command(TARGET ${TR800_ELF_TARGET} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.elf" "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.elf" "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.bin"
    BYPRODUCTS
      "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.hex"
      "${CMAKE_CURRENT_BINARY_DIR}/dlrobot.bin"
    COMMENT "Generating hex and bin outputs"
  )
endif()
