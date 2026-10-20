# Development-Board-C-Examples - RC DMA 双缓冲接收

来源：

- 仓库：`RoboMaster/Development-Board-C-Examples`
- 文件：`9.remote_control_dma/applications/remote_control.c`

## 摘录代码

```c
void USART3_IRQHandler(void)
{
    if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart3);

        __HAL_DMA_DISABLE(&hdma_usart3_rx);
        this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
        hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
        }
        else
        {
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
        }
    }
}
```

## 值得学的点

1. 输入接收和控制决策分开。
2. IRQ 里关注的是“这一帧是否完整”，不是“这帧应该怎么驱动车”。
3. 双缓冲能降低连续帧覆盖风险。

## 对 TR800 的直接启发

- 你当前 UART4 控制帧虽然更短，但设计原则一样：
  - IRQ 只负责接收完整性
  - 任务上下文才负责把帧变成运动命令

## 不要照搬的点

- TR800 不需要为了 11 字节 UART4 帧先上双缓冲 DMA。
- 先把职责边界做对，比先把接收机制做复杂更重要。
