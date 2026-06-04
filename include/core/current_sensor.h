#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

/*
 * current_sensor.h
 *
 * 电流/电压/温度采样转换模块。
 * 输入来自 hal_adc 的原始 ADC count。
 * 输出统一转换为 SI 单位，供 FOC、电机状态和保护模块使用。
 */

#include "core/motor_types.h"
#include "hal/hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 三相电流原始 ADC 值转换为 A，使用 FocConfig 中的零偏和比例系数。 */
void current_sensor_raw_to_phase(const FocConfig *config,
                                 const HalAdcPhaseRaw *raw,
                                 float *ia_a,
                                 float *ib_a,
                                 float *ic_a);
/* 母线电压 ADC 原始值转换为 V；比例需要按硬件分压修正。 */
float current_sensor_vbus_from_raw(uint16_t raw);
/* 温度 ADC 原始值转换为 degC；真实工程应按 NTC/传感器曲线实现。 */
float current_sensor_temperature_from_raw(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSOR_H */
