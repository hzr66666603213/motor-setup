#include "control/current_sensor_admission.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint16_t abs_i16_to_u16(int16_t v)
{
    return (v < 0) ? (uint16_t)(-(int32_t)v) : (uint16_t)v;
}

static uint16_t max3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

#define CURRENT_SENSOR_ADMISSION_WAIT_TIMEOUT_SNAPSHOTS 2000u

static void fail_now(CurrentSensorAdmission *adm, uint32_t flag)
{
    if (adm == NULL ||
        adm->state == CURRENT_SENSOR_ADMISSION_COMPLETE ||
        adm->state == CURRENT_SENSOR_ADMISSION_FAIL) {
        return;
    }
    adm->result.fail_flags |= flag;
    adm->state = CURRENT_SENSOR_ADMISSION_FAIL;
    adm->run_once_locked = true;
}

static void raw_acc_push(CurrentSensorAdmissionRawAccumulator *acc,
                         uint16_t raw0,
                         uint16_t raw1)
{
    if (acc->count == 0u) {
        acc->min0 = raw0;
        acc->max0 = raw0;
        acc->min1 = raw1;
        acc->max1 = raw1;
    }
    if (raw0 < acc->min0) { acc->min0 = raw0; }
    if (raw0 > acc->max0) { acc->max0 = raw0; }
    if (raw1 < acc->min1) { acc->min1 = raw1; }
    if (raw1 > acc->max1) { acc->max1 = raw1; }
    acc->count++;
    acc->sum0 += (uint32_t)raw0;
    acc->sum1 += (uint32_t)raw1;
}

static void post_acc_push(CurrentSensorAdmissionPostAccumulator *acc,
                          int16_t d0,
                          int16_t d1,
                          uint16_t phase_metric)
{
    if (acc->count == 0u) {
        acc->min0 = d0;
        acc->max0 = d0;
        acc->min1 = d1;
        acc->max1 = d1;
    }
    if (d0 < acc->min0) { acc->min0 = d0; }
    if (d0 > acc->max0) { acc->max0 = d0; }
    if (d1 < acc->min1) { acc->min1 = d1; }
    if (d1 > acc->max1) { acc->max1 = d1; }

    const int16_t iu = (int16_t)(-(int32_t)d0 - (int32_t)d1);
    const uint16_t ad0 = abs_i16_to_u16(d0);
    const uint16_t ad1 = abs_i16_to_u16(d1);
    if (ad0 > acc->max_abs_delta_pc0) { acc->max_abs_delta_pc0 = ad0; }
    if (ad1 > acc->max_abs_delta_pc1) { acc->max_abs_delta_pc1 = ad1; }
    if (phase_metric > acc->phase_metric_max_counts) {
        acc->phase_metric_max_counts = phase_metric;
    }

    if (phase_metric > 6u) {
        acc->current_above_6++;
        if (acc->current_above_6 > acc->longest_above_6) {
            acc->longest_above_6 = acc->current_above_6;
        }
    } else {
        acc->current_above_6 = 0u;
    }
    if (phase_metric > 8u) {
        acc->current_above_8++;
        if (acc->current_above_8 > acc->longest_above_8) {
            acc->longest_above_8 = acc->current_above_8;
        }
    } else {
        acc->current_above_8 = 0u;
    }
    if (phase_metric > 10u) {
        acc->current_above_10++;
        if (acc->current_above_10 > acc->longest_above_10) {
            acc->longest_above_10 = acc->current_above_10;
        }
    } else {
        acc->current_above_10 = 0u;
    }

    acc->count++;
    acc->sum0 += (int64_t)d0;
    acc->sum1 += (int64_t)d1;
    acc->sum_u += (int64_t)iu;
    acc->sum0_sq += (int64_t)d0 * (int64_t)d0;
    acc->sum1_sq += (int64_t)d1 * (int64_t)d1;
}

CurrentSensorAdmissionConfig current_sensor_admission_default_config(void)
{
    CurrentSensorAdmissionConfig cfg;
    cfg.dc_cal_discard_samples = 128u;
    cfg.dc_cal_collect_samples = 256u;
    cfg.post_discard_samples = 256u;
    cfg.post_collect_samples = 512u;
    cfg.current_amp_per_count = 0.020142f;
    cfg.std_limit_counts = 3.0f;
    cfg.recenter_live_zero_offset = false;
    cfg.live_zero_shift_limit_counts = 4u;
    cfg.raw_adc_min = 100u;
    cfg.raw_adc_max = 3995u;
    cfg.hard_delta_limit_counts = 20u;
    cfg.hard_phase_limit_counts = 17u;
    cfg.above_6_limit = 8u;
    cfg.above_8_limit = 4u;
    cfg.above_10_limit = 1u;
    return cfg;
}

void current_sensor_admission_init(CurrentSensorAdmission *adm,
                                   const CurrentSensorAdmissionConfig *cfg)
{
    if (adm == NULL) {
        return;
    }
    memset(adm, 0, sizeof(*adm));
    adm->cfg = (cfg != NULL) ? *cfg : current_sensor_admission_default_config();
    adm->state = CURRENT_SENSOR_ADMISSION_IDLE;
}

bool current_sensor_admission_request_start(CurrentSensorAdmission *adm)
{
    if (adm == NULL || adm->state != CURRENT_SENSOR_ADMISSION_IDLE ||
        adm->run_once_locked) {
        return false;
    }
    adm->state = CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE;
    adm->seq_initialized = false;
    adm->offsets_valid = false;
    adm->finalized = false;
    memset(&adm->dc, 0, sizeof(adm->dc));
    memset(&adm->post, 0, sizeof(adm->post));
    memset(&adm->result, 0, sizeof(adm->result));
    return true;
}

CurrentSensorAdmissionMainAction
current_sensor_admission_service_main(CurrentSensorAdmission *adm)
{
    CurrentSensorAdmissionMainAction action = {0};
    if (adm == NULL) {
        return action;
    }
    action.request_enable_dc_cal =
        adm->state == CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE;
    action.request_disable_dc_cal =
        adm->state == CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK;
    action.complete = adm->state == CURRENT_SENSOR_ADMISSION_COMPLETE;
    action.failed = adm->state == CURRENT_SENSOR_ADMISSION_FAIL;
    return action;
}

bool current_sensor_admission_ack_dc_cal_enabled(CurrentSensorAdmission *adm)
{
    if (adm == NULL ||
        adm->state != CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE) {
        return false;
    }
    adm->state = CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_DISCARD;
    return true;
}

bool current_sensor_admission_ack_dc_cal_disabled(CurrentSensorAdmission *adm)
{
    if (adm == NULL ||
        adm->state != CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK ||
        adm->dc.count == 0u) {
        return false;
    }
    const uint32_t n = adm->dc.count;
    adm->result.dc_cal_offset_pc0 =
        (uint16_t)((adm->dc.sum0 + (n / 2u)) / n);
    adm->result.dc_cal_offset_pc1 =
        (uint16_t)((adm->dc.sum1 + (n / 2u)) / n);
    adm->result.dc_cal_samples = n;
    adm->offsets_valid = true;
    adm->result.post_cal_ack_observed_seq = adm->last_seq;
    adm->state = CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_DISCARD;
    return true;
}

void current_sensor_admission_fast_isr(CurrentSensorAdmission *adm,
                                       const CurrentSensorAdmissionFastInput *in)
{
    if (adm == NULL || in == NULL ||
        adm->state == CURRENT_SENSOR_ADMISSION_IDLE ||
        adm->state == CURRENT_SENSOR_ADMISSION_COMPLETE ||
        adm->state == CURRENT_SENSOR_ADMISSION_FAIL) {
        return;
    }

    if (!in->snapshot_valid) {
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_TORN);
        adm->result.torn_count++;
        return;
    }

    if (!adm->seq_initialized) {
        adm->seq_initialized = true;
        adm->last_seq = in->adc_seq;
        adm->result.producer_first_seq = in->adc_seq;
        adm->result.producer_last_seq = in->adc_seq;
        adm->result.producer_seq_span = 1u;
    } else {
        const uint32_t expected = adm->last_seq + 1u;
        if (in->adc_seq == adm->last_seq) {
            adm->result.producer_duplicate_count++;
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_ADC_DUPLICATE);
            return;
        }
        if (in->adc_seq != expected) {
            adm->result.producer_gap_count++;
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_ADC_SEQ_GAP);
            return;
        }
        adm->last_seq = in->adc_seq;
        adm->result.producer_last_seq = in->adc_seq;
        adm->result.producer_seq_span =
            adm->result.producer_last_seq - adm->result.producer_first_seq + 1u;
    }
    adm->result.observed_snapshot_count++;
    adm->result.total_snapshot_count = adm->result.observed_snapshot_count;

    if (in->adc_true_unpaired) {
        adm->result.runtime_true_unpaired_count++;
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_ADC_UNPAIRED);
        return;
    }
    if (in->adc_torn) {
        adm->result.torn_count++;
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_TORN);
        return;
    }
    if (in->adc_generation_mismatch) {
        adm->result.generation_mismatch_count++;
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_GENERATION_MISMATCH);
        return;
    }
    if (!in->nfault_raw_high) {
        adm->result.nfault_runtime_asserted_count++;
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_NFAULT);
        return;
    }
    if (in->raw_pc0 <= adm->cfg.raw_adc_min ||
        in->raw_pc0 >= adm->cfg.raw_adc_max ||
        in->raw_pc1 <= adm->cfg.raw_adc_min ||
        in->raw_pc1 >= adm->cfg.raw_adc_max) {
        fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_RAW_SATURATION);
        return;
    }

    switch (adm->state) {
    case CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE:
        adm->result.wait_dc_cal_enable_snapshot_count++;
        adm->result.transition_wait_snapshot_count++;
        if (adm->result.wait_dc_cal_enable_snapshot_count >
            CURRENT_SENSOR_ADMISSION_WAIT_TIMEOUT_SNAPSHOTS) {
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_DC_CAL_ACK_TIMEOUT);
        }
        break;
    case CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_DISCARD:
        adm->result.dc_cal_discard_count++;
        adm->result.discard_snapshot_count++;
        if (adm->result.dc_cal_discard_count >= adm->cfg.dc_cal_discard_samples) {
            adm->state = CURRENT_SENSOR_ADMISSION_COLLECT_DC_CAL;
        }
        break;
    case CURRENT_SENSOR_ADMISSION_COLLECT_DC_CAL:
        raw_acc_push(&adm->dc, in->raw_pc0, in->raw_pc1);
        adm->result.analysis_snapshot_count++;
        if (adm->dc.count >= adm->cfg.dc_cal_collect_samples) {
            adm->result.dc_cal_collect_last_seq = in->adc_seq;
            adm->state = CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK;
        }
        break;
    case CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK:
        adm->result.wait_post_cal_ack_snapshot_count++;
        adm->result.transition_wait_snapshot_count++;
        if (adm->result.wait_post_cal_ack_snapshot_count >
            CURRENT_SENSOR_ADMISSION_WAIT_TIMEOUT_SNAPSHOTS) {
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_POST_CAL_ACK_TIMEOUT);
        }
        break;
    case CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_DISCARD:
        if (adm->result.post_cal_discard_count == 0u) {
            adm->result.post_cal_discard_first_seq = in->adc_seq;
        }
        adm->result.post_cal_discard_count++;
        adm->result.discard_snapshot_count++;
        if (adm->result.post_cal_discard_count >= adm->cfg.post_discard_samples) {
            adm->state = CURRENT_SENSOR_ADMISSION_COLLECT_POST_CAL;
        }
        break;
    case CURRENT_SENSOR_ADMISSION_COLLECT_POST_CAL: {
        if (!adm->offsets_valid) {
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_SAMPLE_COUNT);
            return;
        }
        const int16_t d0 = (int16_t)((int32_t)in->raw_pc0 -
                                     (int32_t)adm->result.dc_cal_offset_pc0);
        const int16_t d1 = (int16_t)((int32_t)in->raw_pc1 -
                                     (int32_t)adm->result.dc_cal_offset_pc1);
        const int16_t iu = (int16_t)(-(int32_t)d0 - (int32_t)d1);
        const uint16_t phase_metric =
            max3_u16(abs_i16_to_u16(d0), abs_i16_to_u16(d1), abs_i16_to_u16(iu));
        if (abs_i16_to_u16(d0) > adm->cfg.hard_delta_limit_counts ||
            abs_i16_to_u16(d1) > adm->cfg.hard_delta_limit_counts) {
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_RAW_OUTLIER);
            return;
        }
        if (phase_metric > adm->cfg.hard_phase_limit_counts) {
            fail_now(adm, CURRENT_SENSOR_ADMISSION_FAIL_PHASE_OUTLIER);
            return;
        }
        post_acc_push(&adm->post, d0, d1, phase_metric);
        adm->result.analysis_snapshot_count++;
        if (adm->post.count >= adm->cfg.post_collect_samples) {
            adm->result.post_samples = adm->post.count;
            adm->result.scheduled_snapshot_count =
                (uint32_t)adm->cfg.dc_cal_discard_samples +
                (uint32_t)adm->cfg.dc_cal_collect_samples +
                (uint32_t)adm->cfg.post_discard_samples +
                (uint32_t)adm->cfg.post_collect_samples;
            adm->state = CURRENT_SENSOR_ADMISSION_COMPLETE;
            adm->run_once_locked = true;
        }
        break;
    }
    default:
        break;
    }
}

void current_sensor_admission_finalize(CurrentSensorAdmission *adm)
{
    if (adm == NULL || adm->finalized) {
        return;
    }
    adm->result.dc_cal_samples = adm->dc.count;
    adm->result.post_samples = adm->post.count;
    adm->result.scheduled_snapshot_count =
        (uint32_t)adm->cfg.dc_cal_discard_samples +
        (uint32_t)adm->cfg.dc_cal_collect_samples +
        (uint32_t)adm->cfg.post_discard_samples +
        (uint32_t)adm->cfg.post_collect_samples;
    adm->result.analysis_snapshot_count = adm->dc.count + adm->post.count;
    adm->result.discard_snapshot_count =
        adm->result.dc_cal_discard_count + adm->result.post_cal_discard_count;
    adm->result.dc_cal_pc0_min = adm->dc.min0;
    adm->result.dc_cal_pc0_max = adm->dc.max0;
    adm->result.dc_cal_pc1_min = adm->dc.min1;
    adm->result.dc_cal_pc1_max = adm->dc.max1;
    adm->result.live_zero_offset_pc0 = adm->result.dc_cal_offset_pc0;
    adm->result.live_zero_offset_pc1 = adm->result.dc_cal_offset_pc1;
    if (adm->dc.count > 0u) {
        const double inv_dc = 1.0 / (double)adm->dc.count;
        adm->result.dc_cal_pc0_mean = (float)((double)adm->dc.sum0 * inv_dc);
        adm->result.dc_cal_pc1_mean = (float)((double)adm->dc.sum1 * inv_dc);
    }
    adm->result.max_abs_delta_pc0 = adm->post.max_abs_delta_pc0;
    adm->result.max_abs_delta_pc1 = adm->post.max_abs_delta_pc1;
    adm->result.phase_metric_max_counts = adm->post.phase_metric_max_counts;
    adm->result.longest_above_6_counts = adm->post.longest_above_6;
    adm->result.longest_above_8_counts = adm->post.longest_above_8;
    adm->result.longest_above_10_counts = adm->post.longest_above_10;

    if (adm->post.count > 0u) {
        const double inv = 1.0 / (double)adm->post.count;
        const double mean0 = (double)adm->post.sum0 * inv;
        const double mean1 = (double)adm->post.sum1 * inv;
        double var0 = ((double)adm->post.sum0_sq * inv) - mean0 * mean0;
        double var1 = ((double)adm->post.sum1_sq * inv) - mean1 * mean1;
        if (var0 < 0.0) { var0 = 0.0; }
        if (var1 < 0.0) { var1 = 0.0; }
        int32_t shift0 = 0;
        int32_t shift1 = 0;
        if (adm->cfg.recenter_live_zero_offset) {
            shift0 = (int32_t)(mean0 + ((mean0 >= 0.0) ? 0.5 : -0.5));
            shift1 = (int32_t)(mean1 + ((mean1 >= 0.0) ? 0.5 : -0.5));
            int32_t live0 = (int32_t)adm->result.dc_cal_offset_pc0 + shift0;
            int32_t live1 = (int32_t)adm->result.dc_cal_offset_pc1 + shift1;
            if (live0 < 0) { live0 = 0; }
            if (live0 > 4095) { live0 = 4095; }
            if (live1 < 0) { live1 = 0; }
            if (live1 > 4095) { live1 = 4095; }
            adm->result.live_zero_offset_pc0 = (uint16_t)live0;
            adm->result.live_zero_offset_pc1 = (uint16_t)live1;
        }
        adm->result.live_zero_shift_pc0_counts = (int16_t)shift0;
        adm->result.live_zero_shift_pc1_counts = (int16_t)shift1;
        const double residual0 = mean0 - (double)shift0;
        const double residual1 = mean1 - (double)shift1;
        const double residual_u = -(residual0 + residual1);
        adm->result.iv_mean_a = (float)residual0 * adm->cfg.current_amp_per_count;
        adm->result.iw_mean_a = (float)residual1 * adm->cfg.current_amp_per_count;
        adm->result.iu_mean_a = (float)residual_u * adm->cfg.current_amp_per_count;
        adm->result.pc0_std_counts = sqrtf((float)var0);
        adm->result.pc1_std_counts = sqrtf((float)var1);
    }

    if (adm->cfg.recenter_live_zero_offset) {
        const int32_t shift0 = adm->result.live_zero_shift_pc0_counts;
        const int32_t shift1 = adm->result.live_zero_shift_pc1_counts;
        const int32_t shift_u = -(shift0 + shift1);
        const int32_t limit = (int32_t)adm->cfg.live_zero_shift_limit_counts;
        adm->result.mean_pass =
            shift0 >= -limit && shift0 <= limit &&
            shift1 >= -limit && shift1 <= limit &&
            shift_u >= -limit && shift_u <= limit &&
            fabsf(adm->result.iv_mean_a) <= 0.03f &&
            fabsf(adm->result.iw_mean_a) <= 0.03f &&
            fabsf(adm->result.iu_mean_a) <= 0.04f;
    } else {
        adm->result.mean_pass =
            fabsf(adm->result.iv_mean_a) <= 0.03f &&
            fabsf(adm->result.iw_mean_a) <= 0.03f &&
            fabsf(adm->result.iu_mean_a) <= 0.04f;
    }
    adm->result.std_pass =
        adm->result.pc0_std_counts <= adm->cfg.std_limit_counts &&
        adm->result.pc1_std_counts <= adm->cfg.std_limit_counts;
    adm->result.consecutive_pass =
        adm->result.longest_above_6_counts <= adm->cfg.above_6_limit &&
        adm->result.longest_above_8_counts <= adm->cfg.above_8_limit &&
        adm->result.longest_above_10_counts <= adm->cfg.above_10_limit;
    adm->result.hard_outlier_pass =
        adm->result.max_abs_delta_pc0 <= adm->cfg.hard_delta_limit_counts &&
        adm->result.max_abs_delta_pc1 <= adm->cfg.hard_delta_limit_counts &&
        adm->result.phase_metric_max_counts <= adm->cfg.hard_phase_limit_counts;
    adm->result.adc_pipeline_pass =
        adm->result.producer_gap_count == 0u &&
        adm->result.producer_duplicate_count == 0u &&
        adm->result.runtime_true_unpaired_count == 0u &&
        adm->result.torn_count == 0u &&
        adm->result.generation_mismatch_count == 0u &&
        adm->result.nfault_runtime_asserted_count == 0u;
    adm->result.drv_runtime_pass = true;

    if (adm->post.count != adm->cfg.post_collect_samples ||
        adm->dc.count != adm->cfg.dc_cal_collect_samples ||
        adm->result.dc_cal_discard_count != adm->cfg.dc_cal_discard_samples ||
        adm->result.post_cal_discard_count != adm->cfg.post_discard_samples) {
        adm->result.fail_flags |= CURRENT_SENSOR_ADMISSION_FAIL_SAMPLE_COUNT;
    }
    if (!adm->result.mean_pass) {
        adm->result.fail_flags |= CURRENT_SENSOR_ADMISSION_FAIL_MEAN;
    }
    if (!adm->result.std_pass) {
        adm->result.fail_flags |= CURRENT_SENSOR_ADMISSION_FAIL_STD;
    }
    if (!adm->result.consecutive_pass) {
        adm->result.fail_flags |= CURRENT_SENSOR_ADMISSION_FAIL_CONSECUTIVE;
    }
    adm->result.admission_pass =
        adm->state == CURRENT_SENSOR_ADMISSION_COMPLETE &&
        adm->result.fail_flags == CURRENT_SENSOR_ADMISSION_FAIL_NONE &&
        adm->result.mean_pass &&
        adm->result.std_pass &&
        adm->result.consecutive_pass &&
        adm->result.hard_outlier_pass &&
        adm->result.adc_pipeline_pass &&
        adm->result.drv_runtime_pass;
    adm->finalized = true;
}

CurrentSensorAdmissionResult
current_sensor_admission_get_result(const CurrentSensorAdmission *adm)
{
    CurrentSensorAdmissionResult result = {0};
    if (adm != NULL) {
        result = adm->result;
    }
    return result;
}

CurrentSensorAdmissionPreflightVerdict current_sensor_admission_evaluate_preflight(
    const CurrentSensorAdmissionResult *result,
    uint32_t worst_admission_cycles,
    uint32_t worst_adc_callback_cycles,
    uint32_t cpu_hz,
    bool sample_count_pass,
    bool adc_pipeline_pass,
    bool drv_runtime_pass,
    bool final_safe)
{
    CurrentSensorAdmissionPreflightVerdict verdict = {0};
    const uint32_t clock_hz = (cpu_hz != 0u) ? cpu_hz : 168000000u;
    const uint32_t target_5us_cycles = clock_hz / 200000u;
    const uint32_t callback_50us_cycles = clock_hz / 20000u;
    verdict.target_5us_met = worst_admission_cycles < target_5us_cycles;
    verdict.deadline_pass =
        worst_adc_callback_cycles < callback_50us_cycles &&
        adc_pipeline_pass;
    if (result != NULL) {
        verdict.functional_pass =
            result->admission_pass &&
            sample_count_pass &&
            result->mean_pass &&
            result->std_pass &&
            result->consecutive_pass &&
            result->hard_outlier_pass &&
            adc_pipeline_pass &&
            drv_runtime_pass &&
            final_safe;
    }
    verdict.preflight_pass =
        verdict.functional_pass &&
        verdict.deadline_pass;
    return verdict;
}

const char *current_sensor_admission_state_name(CurrentSensorAdmissionState state)
{
    switch (state) {
    case CURRENT_SENSOR_ADMISSION_IDLE: return "IDLE";
    case CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_ENABLE: return "WAIT_DC_CAL_ENABLE";
    case CURRENT_SENSOR_ADMISSION_WAIT_DC_CAL_DISCARD: return "WAIT_DC_CAL_DISCARD";
    case CURRENT_SENSOR_ADMISSION_COLLECT_DC_CAL: return "COLLECT_DC_CAL";
    case CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK: return "WAIT_POST_CAL_ACK";
    case CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_DISCARD: return "WAIT_POST_CAL_DISCARD";
    case CURRENT_SENSOR_ADMISSION_COLLECT_POST_CAL: return "COLLECT_POST_CAL";
    case CURRENT_SENSOR_ADMISSION_COMPLETE: return "COMPLETE";
    case CURRENT_SENSOR_ADMISSION_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}
