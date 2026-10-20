.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global g_pfnVectors
.global Default_Handler
.global Reset_Handler

.extern SystemInit
.extern main

.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word TAMP_STAMP_IRQHandler
  .word RTC_WKUP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Stream0_IRQHandler
  .word DMA1_Stream1_IRQHandler
  .word DMA1_Stream2_IRQHandler
  .word DMA1_Stream3_IRQHandler
  .word DMA1_Stream4_IRQHandler
  .word DMA1_Stream5_IRQHandler
  .word DMA1_Stream6_IRQHandler
  .word ADC_IRQHandler
  .word CAN1_TX_IRQHandler
  .word CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler
  .word CAN1_SCE_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_TIM9_IRQHandler
  .word TIM1_UP_TIM10_IRQHandler
  .word TIM1_TRG_COM_TIM11_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word OTG_FS_WKUP_IRQHandler
  .word TIM8_BRK_TIM12_IRQHandler
  .word TIM8_UP_TIM13_IRQHandler
  .word TIM8_TRG_COM_TIM14_IRQHandler
  .word TIM8_CC_IRQHandler
  .word DMA1_Stream7_IRQHandler
  .word FSMC_IRQHandler
  .word SDIO_IRQHandler
  .word TIM5_IRQHandler
  .word SPI3_IRQHandler
  .word UART4_IRQHandler
  .word UART5_IRQHandler
  .word TIM6_DAC_IRQHandler
  .word TIM7_IRQHandler
  .word DMA2_Stream0_IRQHandler
  .word DMA2_Stream1_IRQHandler
  .word DMA2_Stream2_IRQHandler
  .word DMA2_Stream3_IRQHandler
  .word DMA2_Stream4_IRQHandler
  .word ETH_IRQHandler
  .word ETH_WKUP_IRQHandler
  .word CAN2_TX_IRQHandler
  .word CAN2_RX0_IRQHandler
  .word CAN2_RX1_IRQHandler
  .word CAN2_SCE_IRQHandler
  .word OTG_FS_IRQHandler
  .word DMA2_Stream5_IRQHandler
  .word DMA2_Stream6_IRQHandler
  .word DMA2_Stream7_IRQHandler
  .word USART6_IRQHandler
  .word I2C3_EV_IRQHandler
  .word I2C3_ER_IRQHandler
  .word OTG_HS_EP1_OUT_IRQHandler
  .word OTG_HS_EP1_IN_IRQHandler
  .word OTG_HS_WKUP_IRQHandler
  .word OTG_HS_IRQHandler
  .word DCMI_IRQHandler
  .word 0
  .word RNG_IRQHandler
  .word FPU_IRQHandler

.size g_pfnVectors, .-g_pfnVectors

.section .text.Reset_Handler,"ax",%progbits
.type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_sidata
  ldr   r1, =_sdata
  ldr   r2, =_edata
1:
  cmp   r1, r2
  bcc   2f
  b     3f
2:
  ldr   r3, [r0], #4
  str   r3, [r1], #4
  b     1b
3:
  ldr   r1, =_sbss
  ldr   r2, =_ebss
4:
  cmp   r1, r2
  bcc   5f
  b     6f
5:
  movs  r3, #0
  str   r3, [r1], #4
  b     4b
6:
  bl    SystemInit
  bl    main
7:
  b     7b
.size Reset_Handler, .-Reset_Handler

.macro DEF_HANDLER name
  .weak \name
  .thumb_set \name, Default_Handler
.endm

.section .text.Default_Handler,"ax",%progbits
.type Default_Handler, %function
Default_Handler:
  b .
.size Default_Handler, .-Default_Handler

DEF_HANDLER NMI_Handler
DEF_HANDLER HardFault_Handler
DEF_HANDLER MemManage_Handler
DEF_HANDLER BusFault_Handler
DEF_HANDLER UsageFault_Handler
DEF_HANDLER SVC_Handler
DEF_HANDLER DebugMon_Handler
DEF_HANDLER PendSV_Handler
DEF_HANDLER SysTick_Handler
DEF_HANDLER WWDG_IRQHandler
DEF_HANDLER PVD_IRQHandler
DEF_HANDLER TAMP_STAMP_IRQHandler
DEF_HANDLER RTC_WKUP_IRQHandler
DEF_HANDLER FLASH_IRQHandler
DEF_HANDLER RCC_IRQHandler
DEF_HANDLER EXTI0_IRQHandler
DEF_HANDLER EXTI1_IRQHandler
DEF_HANDLER EXTI2_IRQHandler
DEF_HANDLER EXTI3_IRQHandler
DEF_HANDLER EXTI4_IRQHandler
DEF_HANDLER DMA1_Stream0_IRQHandler
DEF_HANDLER DMA1_Stream1_IRQHandler
DEF_HANDLER DMA1_Stream2_IRQHandler
DEF_HANDLER DMA1_Stream3_IRQHandler
DEF_HANDLER DMA1_Stream4_IRQHandler
DEF_HANDLER DMA1_Stream5_IRQHandler
DEF_HANDLER DMA1_Stream6_IRQHandler
DEF_HANDLER ADC_IRQHandler
DEF_HANDLER CAN1_TX_IRQHandler
DEF_HANDLER CAN1_RX0_IRQHandler
DEF_HANDLER CAN1_RX1_IRQHandler
DEF_HANDLER CAN1_SCE_IRQHandler
DEF_HANDLER EXTI9_5_IRQHandler
DEF_HANDLER TIM1_BRK_TIM9_IRQHandler
DEF_HANDLER TIM1_UP_TIM10_IRQHandler
DEF_HANDLER TIM1_TRG_COM_TIM11_IRQHandler
DEF_HANDLER TIM1_CC_IRQHandler
DEF_HANDLER TIM2_IRQHandler
DEF_HANDLER TIM3_IRQHandler
DEF_HANDLER TIM4_IRQHandler
DEF_HANDLER I2C1_EV_IRQHandler
DEF_HANDLER I2C1_ER_IRQHandler
DEF_HANDLER I2C2_EV_IRQHandler
DEF_HANDLER I2C2_ER_IRQHandler
DEF_HANDLER SPI1_IRQHandler
DEF_HANDLER SPI2_IRQHandler
DEF_HANDLER USART1_IRQHandler
DEF_HANDLER USART2_IRQHandler
DEF_HANDLER USART3_IRQHandler
DEF_HANDLER EXTI15_10_IRQHandler
DEF_HANDLER RTC_Alarm_IRQHandler
DEF_HANDLER OTG_FS_WKUP_IRQHandler
DEF_HANDLER TIM8_BRK_TIM12_IRQHandler
DEF_HANDLER TIM8_UP_TIM13_IRQHandler
DEF_HANDLER TIM8_TRG_COM_TIM14_IRQHandler
DEF_HANDLER TIM8_CC_IRQHandler
DEF_HANDLER DMA1_Stream7_IRQHandler
DEF_HANDLER FSMC_IRQHandler
DEF_HANDLER SDIO_IRQHandler
DEF_HANDLER TIM5_IRQHandler
DEF_HANDLER SPI3_IRQHandler
DEF_HANDLER UART4_IRQHandler
DEF_HANDLER UART5_IRQHandler
DEF_HANDLER TIM6_DAC_IRQHandler
DEF_HANDLER TIM7_IRQHandler
DEF_HANDLER DMA2_Stream0_IRQHandler
DEF_HANDLER DMA2_Stream1_IRQHandler
DEF_HANDLER DMA2_Stream2_IRQHandler
DEF_HANDLER DMA2_Stream3_IRQHandler
DEF_HANDLER DMA2_Stream4_IRQHandler
DEF_HANDLER ETH_IRQHandler
DEF_HANDLER ETH_WKUP_IRQHandler
DEF_HANDLER CAN2_TX_IRQHandler
DEF_HANDLER CAN2_RX0_IRQHandler
DEF_HANDLER CAN2_RX1_IRQHandler
DEF_HANDLER CAN2_SCE_IRQHandler
DEF_HANDLER OTG_FS_IRQHandler
DEF_HANDLER DMA2_Stream5_IRQHandler
DEF_HANDLER DMA2_Stream6_IRQHandler
DEF_HANDLER DMA2_Stream7_IRQHandler
DEF_HANDLER USART6_IRQHandler
DEF_HANDLER I2C3_EV_IRQHandler
DEF_HANDLER I2C3_ER_IRQHandler
DEF_HANDLER OTG_HS_EP1_OUT_IRQHandler
DEF_HANDLER OTG_HS_EP1_IN_IRQHandler
DEF_HANDLER OTG_HS_WKUP_IRQHandler
DEF_HANDLER OTG_HS_IRQHandler
DEF_HANDLER DCMI_IRQHandler
DEF_HANDLER RNG_IRQHandler
DEF_HANDLER FPU_IRQHandler
