#include "drivers/current_sensor.h"

/*
 * current_sensor.c
 *
 * Axis0 电流采样换算 skeleton。
 * 默认比例只用于框架学习，不能作为真实 ODrive v3.6 电流标定。
 */

void current_sensor_set_default_config(CurrentSensorConfig *config)
{
    config->offset_a_count = 2048.0f;
    config->offset_b_count = 2048.0f;
    config->offset_c_count = 2048.0f;
    config->adc_ref_voltage_v = 3.3f;
    config->adc_full_scale_count = 4095.0f;
    config->shunt_resistance_ohm = 0.0005f;
    config->drv8301_gain_v_v = 10.0f;
    config->vbus_scale_v_per_count = 60.0f / 4095.0f;
    config->two_shunt_mode = true;
    current_sensor_recalculate_amp_per_count(config);
}

void current_sensor_bind_drv8301_gain(CurrentSensorConfig *config, float drv8301_gain_v_v)
{
    /*
     * DRV8301 的 shunt amplifier gain 和 amp_per_count 必须绑定。
     * 如果 DRV 寄存器配置了 gain=10/20/40/80 V/V，这里必须同步更新。
     */
    if (drv8301_gain_v_v > 0.0f) {
        config->drv8301_gain_v_v = drv8301_gain_v_v;
        current_sensor_recalculate_amp_per_count(config);
    }
}

void current_sensor_recalculate_amp_per_count(CurrentSensorConfig *config)
{
    if (config->adc_full_scale_count > 0.0f &&
        config->drv8301_gain_v_v > 0.0f &&
        config->shunt_resistance_ohm > 0.0f) {
        config->amp_per_count = config->adc_ref_voltage_v /
                                config->adc_full_scale_count /
                                config->drv8301_gain_v_v /
                                config->shunt_resistance_ohm;
    }
}

CurrentSensorSample current_sensor_convert_raw(const CurrentSensorConfig *config,
                                               uint16_t raw_a,
                                               uint16_t raw_b,
                                               uint16_t raw_c)
{
    CurrentSensorSample sample;
    sample.ia_a = ((float)raw_a - config->offset_a_count) * config->amp_per_count;
    sample.ib_a = ((float)raw_b - config->offset_b_count) * config->amp_per_count;
    if (config->two_shunt_mode) {
        /*
         * ODrive v3.6 Axis0 第一版明确 two-shunt mode。
         * raw_c 在该模式下无效，不能假装成 ADC 中点；第三相只能由 KCL 推算。
         */
        sample.ic_a = -sample.ia_a - sample.ib_a;
    } else {
        sample.ic_a = ((float)raw_c - config->offset_c_count) * config->amp_per_count;
    }
    sample.valid = true;
    return sample;
}

float axis0_current_sensor_vbus_from_raw(const CurrentSensorConfig *config, uint16_t raw)
{
    return (float)raw * config->vbus_scale_v_per_count;
}
