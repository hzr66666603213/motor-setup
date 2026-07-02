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
static uint32_t s_mock_seqlock;
static HalAdcM0RankOrder s_mock_rank_order = HAL_ADC_M0_ORDER_PC0_PC1;

static void hal_adc_mock_publish(const HalAdcSnapshot *snapshot)
{
    s_mock_seqlock++;
    s_mock_snapshot = *snapshot;
    s_mock_seqlock++;
}

static void hal_adc_mock_demux(HalAdcM0RankOrder order,
                               uint16_t rank1,
                               uint16_t rank2,
                               uint16_t rank3,
                               uint16_t rank4,
                               HalAdcSnapshot *snapshot)
{
    if (order == HAL_ADC_M0_ORDER_PC1_PC0) {
        snapshot->raw_pc0_m0_so1 = rank2;
        snapshot->raw_pc1_m0_so2 = rank1;
    } else {
        snapshot->raw_pc0_m0_so1 = rank1;
        snapshot->raw_pc1_m0_so2 = rank2;
    }
    snapshot->raw_pc2_m1_so2 = rank3;
    snapshot->raw_pc3_m1_so1 = rank4;
    snapshot->raw_u = snapshot->raw_pc0_m0_so1;
    snapshot->raw_v = snapshot->raw_pc1_m0_so2;
    snapshot->raw_w = 0u;
}

bool hal_adc_init(void)
{
    s_mock_snapshot.raw_u = 2048u;
    s_mock_snapshot.raw_v = 2048u;
    s_mock_snapshot.raw_w = 0u;
    s_mock_snapshot.raw_pc0_m0_so1 = 2048u;
    s_mock_snapshot.raw_pc1_m0_so2 = 2048u;
    s_mock_snapshot.raw_pc2_m1_so2 = 2048u;
    s_mock_snapshot.raw_pc3_m1_so1 = 2048u;
    s_mock_snapshot.raw_vbus = 2048u;
    s_mock_snapshot.raw_mos_temp = 1200u;
    s_mock_snapshot.seq = 0u;
    s_mock_snapshot.valid = false;
    s_mock_seqlock = 0u;
    s_mock_rank_order = HAL_ADC_M0_ORDER_PC0_PC1;
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

    uint32_t before = 0u;
    uint32_t after = 0u;
    do {
        before = s_mock_seqlock;
        *snapshot = s_mock_snapshot;
        after = s_mock_seqlock;
    } while ((before != after) || ((after & 1u) != 0u));
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
    HalAdcSnapshot next = s_mock_snapshot;
    hal_adc_mock_demux(s_mock_rank_order,
                       2048u,
                       2048u,
                       2048u,
                       2048u,
                       &next);
    next.raw_vbus = 2048u;
    next.raw_mos_temp = 1200u;
    next.seq += 1u;
    next.valid = true;
    hal_adc_mock_publish(&next);
}

bool hal_adc_set_m0_rank_order(HalAdcM0RankOrder order)
{
    if (order != HAL_ADC_M0_ORDER_PC0_PC1 &&
        order != HAL_ADC_M0_ORDER_PC1_PC0) {
        return false;
    }
    s_mock_rank_order = order;
    return true;
}

HalAdcM0RankOrder hal_adc_get_m0_rank_order(void)
{
    return s_mock_rank_order;
}
