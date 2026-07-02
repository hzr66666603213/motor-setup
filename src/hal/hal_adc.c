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
    /* ODrive v3.6 Axis0 two-shunt mode 下第三相 raw 无效，不伪造 ADC 中点。 */
    raw->w = 0u;
    return true;
}

bool hal_adc_get_snapshot(HalAdcSnapshot *snapshot)
{
    static uint32_t s_mock_seq = 0u;
    if (snapshot == 0) {
        return false;
    }

    snapshot->raw_u = 2048u;
    snapshot->raw_v = 2048u;
    snapshot->raw_w = 0u;
    snapshot->raw_vbus = 2048u;
    snapshot->raw_mos_temp = 1200u;
    snapshot->seq = ++s_mock_seq;
    snapshot->valid = true;
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

bool hal_adc_set_m0_rank_order(HalAdcM0RankOrder order)
{
    return order == HAL_ADC_M0_ORDER_PC0_PC1 ||
           order == HAL_ADC_M0_ORDER_PC1_PC0;
}

HalAdcM0RankOrder hal_adc_get_m0_rank_order(void)
{
    return HAL_ADC_M0_ORDER_PC0_PC1;
}

void hal_adc_stm32f405_on_injected_complete(void *hadc)
{
    (void)hadc;
    /* 兼容 mock：真实 STM32F405 后端才在 ADC injected complete 中递增 seq。 */
}
