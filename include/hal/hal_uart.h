#ifndef HAL_UART_H
#define HAL_UART_H

/*
 * hal_uart.h
 *
 * UART 硬件抽象接口。
 * 用于低频调试和参数配置，不应在 PWM ISR 中调用。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 UART 外设。 */
bool hal_uart_init(void);
/* 发送数据；协议层可用，ISR 禁止调用。 */
bool hal_uart_send(const uint8_t *data, size_t length);
/* 非阻塞接收，返回实际读到的字节数。 */
size_t hal_uart_receive(uint8_t *data, size_t max_length);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H */
