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
    config->amp_per_count = 0.01f;
    config->two_shunt_mode = true;
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
        sample.ic_a = -sample.ia_a - sample.ib_a;
    } else {
        sample.ic_a = ((float)raw_c - config->offset_c_count) * config->amp_per_count;
    }
    sample.valid = true;
    return sample;
}

float axis0_current_sensor_vbus_from_raw(uint16_t raw)
{
    /* TODO：按 ODrive v3.6 VBUS_S 分压比例修正。 */
    return (float)raw * (60.0f / 4095.0f);
}
