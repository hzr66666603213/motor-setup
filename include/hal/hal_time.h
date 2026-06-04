#ifndef HAL_TIME_H
#define HAL_TIME_H

/*
 * hal_time.h
 *
 * 时间基准抽象接口。
 * 低频任务、超时判断和调试统计可使用；20 kHz ISR 中应谨慎调用。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回系统启动后的微秒计数，允许自然溢出。 */
uint32_t hal_time_micros(void);
/* 返回系统启动后的毫秒计数，允许自然溢出。 */
uint32_t hal_time_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_TIME_H */
