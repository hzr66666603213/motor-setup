#include "hal/hal_adc.h"

/*
 * hal_adc_stm32f405.c
 *
 * ODrive v3.6 / STM32F405 ADC 真实后端入口。
 *
 * 设计要点：
 * - injected ADC 由 TIM1 同步触发，完成中断里只更新一个快照；
 * - hal_adc_get_snapshot() 只复制快照，不主动读取 ADC，也不递增 seq；
 * - seq 只有在 hadc1/hadc2 本周期数据都到齐后才递增，避免把半帧数据当作新样本；
 * - Axis0 第一阶段按 two-shunt 处理，raw_w=0，第三相电流在 current_sensor 中用 ic=-ia-ib 推算。
 *
 * CubeMX 接入点需要人工核对：
 * - M0_SO1：ADC2 injected rank 1；
 * - M0_SO2：ADC2 injected rank 2；
 * - VBUS_S：ADC1 injected rank 1；
 * - M0_TEMP：ADC1 injected rank 2；
 * - injected external trigger 建议来自 TIM1_CC4 或你的实际采样触发源。
 */

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static volatile HalAdcSnapshot s_snapshot;
static volatile uint32_t s_snapshot_seqlock;
static volatile bool s_adc_started = false;
static volatile HalAdcDiagnostics s_adc_diagnostics;
static volatile HalAdcM0RankOrder s_m0_rank_order = HAL_ADC_M0_ORDER_PC0_PC1;

static uint16_t s_pending_vbus_raw;
static uint16_t s_pending_mos_temp_raw;
static uint16_t s_pending_pc0_m0_so1_raw;
static uint16_t s_pending_pc1_m0_so2_raw;
static uint16_t s_pending_pc2_m1_so2_raw;
static uint16_t s_pending_pc3_m1_so1_raw;
static volatile bool s_pending_adc1_ready = false;
static volatile bool s_pending_adc2_ready = false;

static bool hal_adc_configure_adc2_rank_order(HalAdcM0RankOrder order)
{
    ADC_InjectionConfTypeDef cfg;
    HAL_StatusTypeDef status = HAL_OK;
    const uint32_t rank1_channel =
        (order == HAL_ADC_M0_ORDER_PC1_PC0) ? ADC_CHANNEL_11 : ADC_CHANNEL_10;
    const uint32_t rank2_channel =
        (order == HAL_ADC_M0_ORDER_PC1_PC0) ? ADC_CHANNEL_10 : ADC_CHANNEL_11;

    if (s_adc_started) {
        (void)HAL_ADCEx_InjectedStop_IT(&hadc2);
    }

    cfg.InjectedNbrOfConversion = 4;
    cfg.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    cfg.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    cfg.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    cfg.AutoInjectedConv = DISABLE;
    cfg.InjectedDiscontinuousConvMode = DISABLE;
    cfg.InjectedOffset = 0;

    cfg.InjectedChannel = rank1_channel;
    cfg.InjectedRank = ADC_INJECTED_RANK_1;
    status = HAL_ADCEx_InjectedConfigChannel(&hadc2, &cfg);
    cfg.InjectedChannel = rank2_channel;
    cfg.InjectedRank = ADC_INJECTED_RANK_2;
    if (status == HAL_OK) {
        status = HAL_ADCEx_InjectedConfigChannel(&hadc2, &cfg);
    }
    cfg.InjectedChannel = ADC_CHANNEL_12;
    cfg.InjectedRank = ADC_INJECTED_RANK_3;
    if (status == HAL_OK) {
        status = HAL_ADCEx_InjectedConfigChannel(&hadc2, &cfg);
    }
    cfg.InjectedChannel = ADC_CHANNEL_13;
    cfg.InjectedRank = ADC_INJECTED_RANK_4;
    if (status == HAL_OK) {
        status = HAL_ADCEx_InjectedConfigChannel(&hadc2, &cfg);
    }

    if (s_adc_started) {
        const HAL_StatusTypeDef restart = HAL_ADCEx_InjectedStart_IT(&hadc2);
        if (status == HAL_OK) {
            status = restart;
        }
        s_adc_diagnostics.injected_start_adc2_status = (uint32_t)restart;
        s_pending_adc2_ready = false;
    }

    return status == HAL_OK;
}

static void hal_adc_publish_snapshot(const HalAdcSnapshot *snapshot)
{
    s_snapshot_seqlock++;
    s_snapshot = *snapshot;
    s_snapshot_seqlock++;
}

static void hal_adc_demux_adc2(HalAdcM0RankOrder order,
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
    /*
     * 使用 IT 版本启动 injected conversion，确保 HAL_ADCEx_InjectedConvCpltCallback()
     * 会被调用。真实工程中还必须在 NVIC 里使能 ADC 中断，并确认 ADC injected
     * external trigger 已配置到 TIM1 的采样事件。
     */
    s_snapshot.valid = false;
    s_snapshot.seq = 0u;
    s_snapshot_seqlock = 0u;
    s_pending_adc1_ready = false;
    s_pending_adc2_ready = false;
    s_adc_diagnostics.irq_count = 0u;
    s_adc_diagnostics.adc1_callback_count = 0u;
    s_adc_diagnostics.adc2_callback_count = 0u;
    s_adc_diagnostics.snapshot_count = 0u;

    const HAL_StatusTypeDef ok1 = HAL_ADCEx_InjectedStart_IT(&hadc1);
    const HAL_StatusTypeDef ok2 = HAL_ADCEx_InjectedStart_IT(&hadc2);
    s_adc_diagnostics.injected_start_adc1_status = (uint32_t)ok1;
    s_adc_diagnostics.injected_start_adc2_status = (uint32_t)ok2;
    s_adc_started = (ok1 == HAL_OK) && (ok2 == HAL_OK);
    return s_adc_started;
}

void hal_adc_stm32f405_on_irq_enter(void)
{
    s_adc_diagnostics.irq_count++;
}

void hal_adc_stm32f405_on_injected_complete(void *hadc)
{
    ADC_HandleTypeDef *adc = (ADC_HandleTypeDef *)hadc;

    if (adc == &hadc1) {
        s_adc_diagnostics.adc1_callback_count++;
        s_pending_vbus_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        s_pending_mos_temp_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        s_pending_adc1_ready = true;
    } else if (adc == &hadc2) {
        s_adc_diagnostics.adc2_callback_count++;
        s_pending_pc0_m0_so1_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        s_pending_pc1_m0_so2_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        s_pending_pc2_m1_so2_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
        s_pending_pc3_m1_so1_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);
        s_pending_adc2_ready = true;
    } else {
        return;
    }

    /*
     * HAL 通常分别回调 hadc1 和 hadc2。只有两边都完成后，才发布一帧完整快照并递增 seq。
     * 这样 board_axis0_read_phase_current_raw() 不会读到“电流是新帧、VBUS 是旧帧”的半帧。
     */
    if (s_pending_adc1_ready && s_pending_adc2_ready) {
        HalAdcSnapshot next = {0};
        hal_adc_demux_adc2((HalAdcM0RankOrder)s_m0_rank_order,
                           s_pending_pc0_m0_so1_raw,
                           s_pending_pc1_m0_so2_raw,
                           s_pending_pc2_m1_so2_raw,
                           s_pending_pc3_m1_so1_raw,
                           &next);
        next.raw_vbus = s_pending_vbus_raw;
        next.raw_mos_temp = s_pending_mos_temp_raw;
        next.seq = s_snapshot.seq + 1u;
        next.valid = s_adc_started;
        hal_adc_publish_snapshot(&next);
        s_adc_diagnostics.snapshot_count++;

        s_pending_adc1_ready = false;
        s_pending_adc2_ready = false;
    }
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

    uint32_t before = 0u;
    uint32_t after = 0u;
    do {
        before = s_snapshot_seqlock;
        *snapshot = s_snapshot;
        after = s_snapshot_seqlock;
    } while ((before != after) || ((after & 1u) != 0u));
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
    /* 第一阶段未接电机温度，返回 0，由保护配置决定是否启用。 */
    return 0u;
}

bool hal_adc_samples_valid(void)
{
    return s_adc_started && s_snapshot.valid;
}

void hal_adc_get_diagnostics(HalAdcDiagnostics *diagnostics)
{
    if (diagnostics == 0) {
        return;
    }

    *diagnostics = s_adc_diagnostics;
}

bool hal_adc_set_m0_rank_order(HalAdcM0RankOrder order)
{
    if (order != HAL_ADC_M0_ORDER_PC0_PC1 &&
        order != HAL_ADC_M0_ORDER_PC1_PC0) {
        return false;
    }

    if (!hal_adc_configure_adc2_rank_order(order)) {
        return false;
    }

    s_m0_rank_order = order;
    return true;
}

HalAdcM0RankOrder hal_adc_get_m0_rank_order(void)
{
    return (HalAdcM0RankOrder)s_m0_rank_order;
}
