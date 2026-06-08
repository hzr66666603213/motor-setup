#include "hal/hal_adc.h"

/*
 * hal_adc_mock.c
 *
 * PC/Simulink/早期框架测试使用的 ADC mock 后端。
 * 该文件不访问任何真实 ADC 外设，只返回接近 12-bit ADC 中点的安全样本。
 * 真实上板工程不要编译本文件，应改用 src/hal/stm32f405/hal_adc_stm32f405.c。
 */

bool hal_adc_init(void)
{
    return true;
}

bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw)
{
    if (raw == 0) {
        return false;
    }

    raw->u = 2048u;
    raw->v = 2048u;
    raw->w = 0u; /* two-shunt 模式下第三相原始采样无效，不伪造为中点。 */
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

void hal_adc_stm32f405_on_injected_complete(void)
{
    /* mock 后端不依赖真实 ADC conversion complete 回调。 */
}
