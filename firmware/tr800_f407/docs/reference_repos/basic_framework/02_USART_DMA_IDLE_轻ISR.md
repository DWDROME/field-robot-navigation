# basic_framework - USART DMA/IDLE 轻 ISR

来源：

- 仓库：`HNUYueLuRM/basic_framework`
- 文件：`bsp/usart/bsp_usart.c`

## 摘录代码

```c
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle,
                                 _instance->recv_buff,
                                 _instance->recv_buff_size);
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle,
                                         usart_instance[i]->recv_buff,
                                         usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}
```

## 值得学的点

1. ISR 回调只做分发和重启接收，不碰业务控制量。
2. 具体协议解析下沉到模块回调，不堆在 BSP 或 IRQ 里。
3. 收到一帧后立即 re-arm 接收，接收链路连续。

## 对 TR800 的直接启发

你现在最该学的就是这一条：

`UART4_IRQHandler()` 只保留“接收完成 -> 标记完整帧 -> 重新收下一帧”。

不要在 ISR 里做：

- `tr800_runtime_request_mode()`
- `Move_X/Y/Z` 写入
- 业务级模式切换

## 不要照搬的点

- TR800 第一阶段不必强上 HAL DMA-to-idle。
- 只要先把“ISR 直接写运动状态”去掉，即使用当前字节流拼帧方式，也已经是明显进步。
