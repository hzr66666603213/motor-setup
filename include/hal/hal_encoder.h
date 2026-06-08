#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

/*
 * hal_encoder.h
 *
 * 增量编码器硬件抽象层。
 *
 * MT6701_ABZ 驱动只关心“Encoder0 当前累计计数是多少”，不应该直接包含
 * stm32f4xx_hal.h，也不应该直接访问 htim3。真实 STM32F405 后端负责把 TIM3
 * Encoder Mode 的 16-bit CNT 展开成 int32_t 累计计数。
 *
 * 调用频率：
 * - hal_encoder0_get_count()：20 kHz ISR 或 1 kHz 编码器更新任务；
 * - init/reset：启动、IDLE 或校准前调用。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hal_encoder0_init(void);
int32_t hal_encoder0_get_count(void);
void hal_encoder0_reset_count(void);
bool hal_encoder0_index_found(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ENCODER_H */
