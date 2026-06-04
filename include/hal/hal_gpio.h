#ifndef HAL_GPIO_H
#define HAL_GPIO_H

/*
 * hal_gpio.h
 *
 * GPIO 硬件抽象接口。
 * 主要用于 gate driver 使能、nFAULT 读取和限位输入。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 读取栅极驱动 nFAULT；返回 true 表示有故障。 */
bool hal_gpio_read_fault_pin(void);
/* 控制 EN_GATE；false 必须让功率级进入安全状态。 */
void hal_gpio_set_gate_enable(bool enabled);
/* 读取限位/急停等数字输入，bit 定义由具体硬件后端决定。 */
uint32_t hal_gpio_read_limit_inputs(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
