#ifndef HAL_ADC_H
#define HAL_ADC_H

/*
 * hal_adc.h
 *
 * ADC 硬件抽象接口。
 * 三电阻低边采样要求 ADC 与 PWM 同步，在合适的低边导通窗口取样。
 * 本接口只提供原始 count，不在 HAL 层做物理量换算。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t u; /* U 相电流 ADC 原始值，count */
    uint16_t v; /* V 相电流 ADC 原始值，count */
    uint16_t w; /* W 相电流 ADC 原始值，count */
} HalAdcPhaseRaw;

/* 初始化 ADC、触发源和 DMA/注入通道；stub 中直接返回 true。 */
bool hal_adc_init(void);
/* 读取最近一次 PWM 同步采样得到的三相电流原始值。 */
bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw);
/* 读取母线电压 ADC 原始值。 */
uint16_t hal_adc_get_vbus_raw(void);
/* 读取 MOS/板温 ADC 原始值。 */
uint16_t hal_adc_get_mos_temperature_raw(void);
/* 读取电机温度 ADC 原始值。 */
uint16_t hal_adc_get_motor_temperature_raw(void);
/* 判断当前 ADC 样本是否有效，例如 DMA 完成、未过载、采样窗口正确。 */
bool hal_adc_samples_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H */
