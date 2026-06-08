#ifndef AXIS0_CURRENT_SENSOR_H
#define AXIS0_CURRENT_SENSOR_H

/*
 * current_sensor.h
 *
 * ODrive v3.6 Axis0 电流/电压采样转换。
 * ODrive v3.x 常见为两相电流采样，第三相可由 ia+ib+ic=0 推算。
 * 真实比例必须根据采样电阻、运放增益、ADC 参考电压和 ODrive v3.6 原理图确认。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float offset_a_count;       /* A/U 相 ADC 零偏，count */
    float offset_b_count;       /* B/V 相 ADC 零偏，count */
    float offset_c_count;       /* C/W 相 ADC 零偏，count */
    float amp_per_count;        /* ADC count 到电流的比例，A/count */
    float adc_ref_voltage_v;    /* ADC 参考电压，V */
    float adc_full_scale_count; /* ADC 满量程 count，例如 4095 */
    float shunt_resistance_ohm; /* 分流电阻，ohm，必须按 ODrive v3.6 实物确认 */
    float drv8301_gain_v_v;     /* DRV8301 shunt amplifier gain，V/V */
    float vbus_scale_v_per_count; /* VBUS ADC 比例，V/count，24V/56V 版本需实测 */
    bool two_shunt_mode;        /* true 表示第三相由 -ia-ib 推算 */
} CurrentSensorConfig;

typedef struct {
    float ia_a;                 /* A/U 相电流，A */
    float ib_a;                 /* B/V 相电流，A */
    float ic_a;                 /* C/W 相电流，A */
    bool valid;                 /* 样本是否有效 */
} CurrentSensorSample;

void current_sensor_set_default_config(CurrentSensorConfig *config);
void current_sensor_bind_drv8301_gain(CurrentSensorConfig *config, float drv8301_gain_v_v);
void current_sensor_recalculate_amp_per_count(CurrentSensorConfig *config);
CurrentSensorSample current_sensor_convert_raw(const CurrentSensorConfig *config,
                                               uint16_t raw_a,
                                               uint16_t raw_b,
                                               uint16_t raw_c);
float axis0_current_sensor_vbus_from_raw(const CurrentSensorConfig *config, uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_CURRENT_SENSOR_H */
