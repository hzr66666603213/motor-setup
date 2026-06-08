#include "hal/hal_adc.h"

/*
 * hal_adc_stm32f405.c
 *
 * ODrive v3.6 / STM32F405 ADC 真实后端入口。
 *
 * 重要说明：
 * - 这里提供的是可接入 CubeMX 的真实 HAL 读取路径，不再返回固定 2048；
 * - 具体 ADC instance、injected rank、触发源和采样时刻必须在 CubeMX 中按
 *   ODrive v3.6 原理图和 TIM1 PWM 采样窗口核对；
 * - ODrive v3.6 Axis0 第一阶段按 two-shunt 处理：raw_w=0，第三相电流在
 *   current_sensor 中用 ic=-ia-ib 推算，不能伪造第三相 ADC 中点。
 *
 * 默认映射建议：
 * - M0_SO1：ADC2 injected rank 1；
 * - M0_SO2：ADC2 injected rank 2；
 * - VBUS_S：ADC1 injected/regular rank 1；
 * - M0_TEMP：ADC1 injected/regular rank 2。
 *
 * 这些 rank 是工程接入点，不是最终硬件真理。首次上板前必须用示波器/串口
 * dump 原始 count 核对通道和比例。
 */

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static volatile HalAdcSnapshot s_snapshot;
static volatile bool s_adc_started = false;

bool hal_adc_init(void)
{
    /*
     * injected ADC 通常由 TIM1 TRGO/CC 触发。这里启动 injected conversion，
     * 具体触发配置由 CubeMX 生成的 MX_ADCx_Init() 决定。
     */
    const HAL_StatusTypeDef ok1 = HAL_ADCEx_InjectedStart(&hadc1);
    const HAL_StatusTypeDef ok2 = HAL_ADCEx_InjectedStart(&hadc2);
    s_snapshot.valid = false;
    s_snapshot.seq = 0u;
    s_adc_started = (ok1 == HAL_OK) && (ok2 == HAL_OK);
    return s_adc_started;
}

/*
 * 可由 HAL_ADCEx_InjectedConvCpltCallback() 调用，也可在 PWM ADC 完成中断中调用。
 * 为避免把 CubeMX 回调函数强塞进框架，这里只提供一个后端内部可见的更新函数。
 */
static void update_snapshot_from_injected_adc(void)
{
    HalAdcSnapshot next;
    next.raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    next.raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    next.raw_w = 0u;
    next.raw_vbus = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    next.raw_mos_temp = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    next.seq = s_snapshot.seq + 1u;
    next.valid = s_adc_started;
    s_snapshot = next;
}

bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw)
{
    HalAdcSnapshot snap;
    if (!hal_adc_get_snapshot(&snap) || raw == 0) {
        return false;
    }

    raw->u = snap.raw_u;
    raw->v = snap.raw_v;
    raw->w = 0u;
    return snap.valid;
}

bool hal_adc_get_snapshot(HalAdcSnapshot *snapshot)
{
    if (snapshot == 0 || !s_adc_started) {
        return false;
    }

    /*
     * 简化接入方式：读取时刷新 injected ADC 结果并递增 seq。
     * 更严格的量产写法是在 ADC injected complete ISR 中调用更新函数，
     * 电流环 ISR 只读取已完成快照并检查 seq 是否变化。
     */
    update_snapshot_from_injected_adc();
    snapshot->raw_u = s_snapshot.raw_u;
    snapshot->raw_v = s_snapshot.raw_v;
    snapshot->raw_w = s_snapshot.raw_w;
    snapshot->raw_vbus = s_snapshot.raw_vbus;
    snapshot->raw_mos_temp = s_snapshot.raw_mos_temp;
    snapshot->seq = s_snapshot.seq;
    snapshot->valid = s_snapshot.valid;
    return snapshot->valid;
}

uint16_t hal_adc_get_vbus_raw(void)
{
    HalAdcSnapshot snap;
    return hal_adc_get_snapshot(&snap) ? snap.raw_vbus : 0u;
}

uint16_t hal_adc_get_mos_temperature_raw(void)
{
    HalAdcSnapshot snap;
    return hal_adc_get_snapshot(&snap) ? snap.raw_mos_temp : 0u;
}

uint16_t hal_adc_get_motor_temperature_raw(void)
{
    /* 第一版未接电机温度，返回 0 并由保护配置决定是否启用。 */
    return 0u;
}

bool hal_adc_samples_valid(void)
{
    return s_adc_started;
}
