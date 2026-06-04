#include "hal/hal_uart.h"

/*
 * hal_uart.c
 *
 * UART mock/stub 实现。
 * 当前不产生实际串口输出；移植时应绑定中断、DMA 或环形缓冲。
 */

bool hal_uart_init(void)
{
    return true;
}

bool hal_uart_send(const uint8_t *data, size_t length)
{
    /* mock 丢弃发送数据；真实后端可使用 DMA 或 TX FIFO。 */
    (void)data;
    (void)length;
    return true;
}

size_t hal_uart_receive(uint8_t *data, size_t max_length)
{
    /* mock 无接收数据；真实后端应从 RX ring buffer 中取数。 */
    (void)data;
    (void)max_length;
    return 0u;
}
