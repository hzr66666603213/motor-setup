#include "hal/hal_adc.h"

/*
 * hal_adc.c
 *
 * ADC mock/stub 实现。
 * 当前返回固定中点值，用于框架搭建和单元测试。
 * 移植时应替换为 STM32 ADC 注入通道或 DMA 缓冲读取。
 */

bool hal_adc_init(void)
{
    return true;
}

bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw)
{
    /* 12-bit ADC 中点，模拟零电流附近的采样结果。 */
    raw->u = 2048u;
    raw->v = 2048u;
    raw->w = 2048u;
    return true;
}

uint16_t hal_adc_get_vbus_raw(void)
{
    /* mock 值不代表真实硬件比例，换算在 current_sensor.c 中完成。 */
    return 2048u;
}

uint16_t hal_adc_get_mos_temperature_raw(void)
{
    return 1200u;
}

uint16_t hal_adc_get_motor_temperature_raw(void)
{
    return 1200u;
}

bool hal_adc_samples_valid(void)
{
    return true;
}
