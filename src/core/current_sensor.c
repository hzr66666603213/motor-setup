#include "core/current_sensor.h"

/*
 * current_sensor.c
 *
 * 采样换算 mock 实现。
 * 当前使用线性比例，便于搭建框架。
 * 移植时应根据运放增益、采样电阻、ADC 参考电压和温度传感器曲线修正。
 */

void current_sensor_raw_to_phase(const FocConfig *config,
                                 const HalAdcPhaseRaw *raw,
                                 float *ia_a,
                                 float *ib_a,
                                 float *ic_a)
{
    /*
     * 三电阻采样常见换算：
     * phase_current_A = (adc_count - zero_offset_count) * A_per_count。
     * zero_offset_count 应由电流采样零偏校准得到。
     */
    *ia_a = ((float)raw->u - config->current_offset_u_count) * config->current_sample_gain_a_per_count;
    *ib_a = ((float)raw->v - config->current_offset_v_count) * config->current_sample_gain_a_per_count;
    *ic_a = ((float)raw->w - config->current_offset_w_count) * config->current_sample_gain_a_per_count;
}

float current_sensor_vbus_from_raw(uint16_t raw)
{
    /* mock：假设 4095 count 对应 60 V。真实硬件按母线分压比例计算。 */
    return (float)raw * (60.0f / 4095.0f);
}

float current_sensor_temperature_from_raw(uint16_t raw)
{
    /* mock：假设 4095 count 对应 150 degC。真实硬件应查表或 Steinhart-Hart。 */
    return (float)raw * (150.0f / 4095.0f);
}
