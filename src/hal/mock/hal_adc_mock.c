#include "hal/hal_adc.h"

/*
 * hal_adc_mock.c
 *
 * PC/Simulink/早期框架测试使用的 ADC mock 后端。
 * 该文件不访问任何真实 ADC 外设，只返回接近 12-bit ADC 中点的安全样本。
 *
 * 重要语义：
 * - hal_adc_get_snapshot() 只复制最近一次快照，不主动递增 seq；
 * - hal_adc_stm32f405_on_injected_complete() 用来模拟一次 injected ADC 转换完成；
 * - board/ISR 通过 seq 是否变化判断本周期是否拿到了新样本。
 */

static HalAdcSnapshot s_mock_snapshot;

bool hal_adc_init(void)
{
    s_mock_snapshot.raw_u = 2048u;
    s_mock_snapshot.raw_v = 2048u;
    s_mock_snapshot.raw_w = 0u;
    s_mock_snapshot.raw_vbus = 2048u;
    s_mock_snapshot.raw_mos_temp = 1200u;
    s_mock_snapshot.seq = 0u;
    s_mock_snapshot.valid = false;
    return true;
}

bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw)
{
    if (raw == 0) {
        return false;
    }

    raw->u = s_mock_snapshot.raw_u;
    raw->v = s_mock_snapshot.raw_v;
    raw->w = 0u; /* two-shunt 模式下第三相原始采样无效，不伪造成中点。 */
    return s_mock_snapshot.valid;
}

bool hal_adc_get_snapshot(HalAdcSnapshot *snapshot)
{
    if (snapshot == 0) {
        return false;
    }

    *snapshot = s_mock_snapshot;
    return true;
}

uint16_t hal_adc_get_vbus_raw(void)
{
    return s_mock_snapshot.valid ? s_mock_snapshot.raw_vbus : 0u;
}

uint16_t hal_adc_get_mos_temperature_raw(void)
{
    return s_mock_snapshot.valid ? s_mock_snapshot.raw_mos_temp : 0u;
}

uint16_t hal_adc_get_motor_temperature_raw(void)
{
    return 1200u;
}

bool hal_adc_samples_valid(void)
{
    return s_mock_snapshot.valid;
}

void hal_adc_stm32f405_on_injected_complete(void *hadc)
{
    (void)hadc;
    /*
     * 模拟一次 ADC injected conversion complete。
     * 功率级是否使能不影响 ADC seq；只要 TIM1/ADC trigger 在运行，真实后端也应由
     * HAL_ADCEx_InjectedConvCpltCallback() 调用对应函数递增 seq。
     */
    s_mock_snapshot.raw_u = 2048u;
    s_mock_snapshot.raw_v = 2048u;
    s_mock_snapshot.raw_w = 0u;
    s_mock_snapshot.raw_vbus = 2048u;
    s_mock_snapshot.raw_mos_temp = 1200u;
    s_mock_snapshot.seq += 1u;
    s_mock_snapshot.valid = true;
}
