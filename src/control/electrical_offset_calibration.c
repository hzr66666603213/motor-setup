#include "control/electrical_offset_calibration.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define EO_PI_F 3.14159265358979323846f
#define EO_TWO_PI_F (2.0f * EO_PI_F)
#define EO_THREE_F 3.0f

ElectricalOffsetCalibrationConfig electrical_offset_calibration_default_config(void)
{
    ElectricalOffsetCalibrationConfig cfg;
    cfg.encoder_cpr = 4096;
    cfg.encoder_direction = 1;
    cfg.pole_pairs = 7;
    cfg.max_offset_spread_deg = 8.0f;
    cfg.expected_delta_tolerance_counts = 20.0f;
    cfg.alignment_voltage_limit_v = 0.40f;
    return cfg;
}

float electrical_offset_wrap_0_2pi(float x)
{
    while (x >= EO_TWO_PI_F) {
        x -= EO_TWO_PI_F;
    }
    while (x < 0.0f) {
        x += EO_TWO_PI_F;
    }
    return x;
}

float electrical_offset_angle_distance_rad(float a, float b)
{
    float d = electrical_offset_wrap_0_2pi(a) - electrical_offset_wrap_0_2pi(b);
    while (d > EO_PI_F) {
        d -= EO_TWO_PI_F;
    }
    while (d < -EO_PI_F) {
        d += EO_TWO_PI_F;
    }
    return fabsf(d);
}

static float eo_wrap_minus_pi_pi(float x)
{
    while (x > EO_PI_F) {
        x -= EO_TWO_PI_F;
    }
    while (x < -EO_PI_F) {
        x += EO_TWO_PI_F;
    }
    return x;
}

float electrical_offset_count_delta(float now, float reference, int32_t cpr)
{
    if (cpr <= 0) {
        return 0.0f;
    }
    float d = now - reference;
    const float half = (float)cpr * 0.5f;
    while (d > half) {
        d -= (float)cpr;
    }
    while (d < -half) {
        d += (float)cpr;
    }
    return d;
}

static bool eo_best_equivalent_delta(const ElectricalOffsetCalibrationConfig *cfg,
                                     float command_delta_rad,
                                     float measured_delta_counts,
                                     float *best_command_delta_rad,
                                     float *best_error_counts)
{
    if (cfg == 0 ||
        cfg->encoder_cpr <= 0 ||
        cfg->encoder_direction == 0 ||
        cfg->pole_pairs <= 0 ||
        !isfinite(command_delta_rad) ||
        !isfinite(measured_delta_counts)) {
        return false;
    }

    bool found = false;
    float best_cmd = 0.0f;
    float best_err = 1.0e30f;
    for (int n = -1; n <= 1; ++n) {
        const float candidate_cmd =
            command_delta_rad + ((float)n * EO_TWO_PI_F);
        if (fabsf(candidate_cmd) < 1.0e-6f) {
            continue;
        }
        const float expected_counts =
            candidate_cmd * (float)cfg->encoder_cpr /
            (EO_TWO_PI_F * (float)cfg->encoder_direction *
             (float)cfg->pole_pairs);
        const float err = fabsf(measured_delta_counts - expected_counts);
        if (!found || err < best_err) {
            found = true;
            best_err = err;
            best_cmd = candidate_cmd;
        }
    }

    if (best_command_delta_rad != 0) {
        *best_command_delta_rad = best_cmd;
    }
    if (best_error_counts != 0) {
        *best_error_counts = best_err;
    }
    return found;
}

const char *electrical_offset_state_name(ElectricalOffsetBringupState state)
{
    switch (state) {
    case ELECTRICAL_OFFSET_STATE_IDLE: return "IDLE";
    case ELECTRICAL_OFFSET_STATE_PREFLIGHT_INIT: return "PREFLIGHT_INIT";
    case ELECTRICAL_OFFSET_STATE_PREFLIGHT_ADMISSION: return "PREFLIGHT_ADMISSION";
    case ELECTRICAL_OFFSET_STATE_PREFLIGHT_HANDOFF: return "PREFLIGHT_HANDOFF";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_PRE_ENABLE: return "ALIGNMENT_DRV_PRE_ENABLE";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_WAIT_NFAULT: return "ALIGNMENT_DRV_WAIT_NFAULT";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_SETTLE: return "ALIGNMENT_DRV_SETTLE";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_CONFIGURE: return "ALIGNMENT_DRV_CONFIGURE";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_DRV_VERIFY: return "ALIGNMENT_DRV_VERIFY";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_CURRENT_OFFSET_REVALIDATE: return "ALIGNMENT_CURRENT_OFFSET_REVALIDATE";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_REQUEST: return "ALIGNMENT_ZERO_COMMAND_REQUEST";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_WAIT_ACK: return "ALIGNMENT_ZERO_COMMAND_WAIT_ACK";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_ZERO_COMMAND_SETTLE: return "ALIGNMENT_ZERO_COMMAND_SETTLE";
    case ELECTRICAL_OFFSET_STATE_ALIGNMENT_PRE_GATE_CHECK: return "ALIGNMENT_PRE_GATE_CHECK";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP: return "ALIGN_POINT_0_RAMP";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_HOLD: return "ALIGN_POINT_0_HOLD";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_SAMPLE: return "ALIGN_POINT_0_SAMPLE";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP_DOWN: return "ALIGN_POINT_0_RAMP_DOWN";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_RAMP: return "ALIGN_POINT_POS120_RAMP";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_HOLD: return "ALIGN_POINT_POS120_HOLD";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_SAMPLE: return "ALIGN_POINT_POS120_SAMPLE";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_POS120_RAMP_DOWN: return "ALIGN_POINT_POS120_RAMP_DOWN";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_RAMP: return "ALIGN_POINT_NEG120_RAMP";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_HOLD: return "ALIGN_POINT_NEG120_HOLD";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_SAMPLE: return "ALIGN_POINT_NEG120_SAMPLE";
    case ELECTRICAL_OFFSET_STATE_ALIGN_POINT_NEG120_RAMP_DOWN: return "ALIGN_POINT_NEG120_RAMP_DOWN";
    case ELECTRICAL_OFFSET_STATE_CALCULATE_OFFSET: return "CALCULATE_OFFSET";
    case ELECTRICAL_OFFSET_STATE_COMPLETE: return "COMPLETE";
    case ELECTRICAL_OFFSET_STATE_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}

const char *electrical_offset_failure_name(ElectricalOffsetFailure failure)
{
    switch (failure) {
    case ELECTRICAL_OFFSET_FAILURE_NONE: return "none";
    case ELECTRICAL_OFFSET_FAILURE_ADMISSION_REJECTED: return "electrical_offset_admission_rejected";
    case ELECTRICAL_OFFSET_FAILURE_ADMISSION_DEADLINE_MISS: return "electrical_offset_admission_deadline_miss";
    case ELECTRICAL_OFFSET_FAILURE_ADMISSION_HANDOFF_FAILED: return "electrical_offset_admission_handoff_failed";
    case ELECTRICAL_OFFSET_FAILURE_HOOK_OVERLAP: return "electrical_offset_hook_overlap";
    case ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID: return "electrical_offset_adc_pipeline_invalid";
    case ELECTRICAL_OFFSET_FAILURE_DRV_RUNTIME_FAULT: return "electrical_offset_drv_runtime_fault";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OVERCURRENT: return "electrical_offset_alignment_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OVERSPEED: return "electrical_offset_alignment_overspeed";
    case ELECTRICAL_OFFSET_FAILURE_ENCODER_DELTA_INVALID: return "electrical_offset_encoder_delta_invalid";
    case ELECTRICAL_OFFSET_FAILURE_SPREAD_INVALID: return "electrical_offset_spread_invalid";
    case ELECTRICAL_OFFSET_FAILURE_CALLBACK_OVERRUN: return "electrical_offset_callback_overrun";
    case ELECTRICAL_OFFSET_FAILURE_STATE_TIMEOUT: return "electrical_offset_state_timeout";
    case ELECTRICAL_OFFSET_FAILURE_STATE_TRANSITION_INVALID: return "electrical_offset_state_transition_invalid";
    case ELECTRICAL_OFFSET_FAILURE_STALE_FAULT_LATCHED: return "electrical_offset_stale_fault_latched";
    case ELECTRICAL_OFFSET_FAILURE_PRE_ALIGNMENT_GATE_FAILED: return "electrical_offset_pre_alignment_gate_failed";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DISPATCH_NOT_READY: return "electrical_offset_alignment_dispatch_not_ready";
    case ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY: return "electrical_offset_drv_not_ready";
    case ELECTRICAL_OFFSET_FAILURE_OFFSETS_INVALID: return "electrical_offset_offsets_invalid";
    case ELECTRICAL_OFFSET_FAILURE_DC_CAL_NOT_CLEAR: return "electrical_offset_dc_cal_not_clear";
    case ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO: return "electrical_offset_zero_command_not_zero";
    case ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_APPLIED: return "electrical_offset_zero_command_not_applied";
    case ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_HANDOFF_TIMEOUT: return "electrical_offset_zero_command_handoff_timeout";
    case ELECTRICAL_OFFSET_FAILURE_CCR_NOT_SAFE: return "electrical_offset_ccr_not_safe";
    case ELECTRICAL_OFFSET_FAILURE_PENDING_VOLTAGE_UPDATE: return "electrical_offset_pending_voltage_update";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC0_OVERCURRENT: return "electrical_offset_alignment_raw_pc0_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC1_OVERCURRENT: return "electrical_offset_alignment_raw_pc1_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT: return "electrical_offset_alignment_phase_soft_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_HARD_OVERCURRENT: return "electrical_offset_alignment_phase_hard_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DQ_OVERCURRENT: return "electrical_offset_alignment_dq_overcurrent";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ADC_SATURATION: return "electrical_offset_alignment_adc_saturation";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OFFSET_SHIFT: return "electrical_offset_alignment_offset_shift";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_VOLTAGE_SCALING_INVALID: return "electrical_offset_alignment_voltage_scaling_invalid";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CURRENT_MODEL_INVALID: return "electrical_offset_alignment_current_model_invalid";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CORE_OVERRUN: return "electrical_offset_alignment_core_overrun";
    case ELECTRICAL_OFFSET_FAILURE_ADC_CALLBACK_OVERRUN: return "electrical_offset_adc_callback_overrun";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_TIMING_SCOPE_INVALID: return "electrical_offset_alignment_timing_scope_invalid";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PROFILE_OVERHEAD: return "electrical_offset_alignment_profile_overhead";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_INTERRUPT_PREEMPTED: return "electrical_offset_alignment_interrupt_preempted";
    case ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ENTRY_INIT_OVERRUN: return "electrical_offset_alignment_entry_init_overrun";
    default: return "electrical_offset_unknown_failure";
    }
}

const char *electrical_offset_pwm_zero_classification_name(
    ElectricalOffsetPwmZeroClassification classification)
{
    switch (classification) {
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_UNKNOWN:
        return "UNKNOWN";
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ACTIVE_OFFSET_SHIFT:
        return "PWM_ACTIVE_OFFSET_SHIFT";
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_ADC_SWITCHING_EDGE_CONTAMINATION:
        return "ADC_SWITCHING_EDGE_CONTAMINATION";
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ZERO_STATE_INVALID:
        return "PWM_ZERO_STATE_INVALID";
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_CURRENT_RECONSTRUCTION_INVALID:
        return "CURRENT_RECONSTRUCTION_INVALID";
    case ELECTRICAL_OFFSET_PWM_ZERO_CLASS_POSSIBLE_REAL_UNINTENDED_CURRENT:
        return "POSSIBLE_REAL_UNINTENDED_CURRENT";
    default:
        return "UNKNOWN";
    }
}

const char *electrical_offset_trigger_sweep_classification_name(
    ElectricalOffsetTriggerSweepClassification classification)
{
    switch (classification) {
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_TOO_CLOSE_TO_SWITCH_EDGE:
        return "TRIGGER_TOO_CLOSE_TO_SWITCH_EDGE";
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_COMMON_MODE_SETTLING_LONG:
        return "PWM_COMMON_MODE_SETTLING_LONGER_THAN_EXPECTED";
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_ACTIVE_OFFSET_INDEPENDENT:
        return "PWM_ACTIVE_OFFSET_INDEPENDENT_OF_TRIGGER";
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_NO_VALID_LOW_SIDE_WINDOW:
        return "NO_VALID_LOW_SIDE_WINDOW";
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_EVENT_CONFIG_INVALID:
        return "TRIGGER_EVENT_CONFIGURATION_INVALID";
    case ELECTRICAL_OFFSET_TRIGGER_SWEEP_INCONCLUSIVE:
    default:
        return "INCONCLUSIVE";
    }
}

uint32_t electrical_offset_dwt_elapsed(uint32_t start, uint32_t end)
{
    return end - start;
}

ElectricalOffsetTimingVerdict electrical_offset_timing_evaluate(
    const ElectricalOffsetTimingInput *input)
{
    ElectricalOffsetTimingVerdict out;
    memset(&out, 0, sizeof(out));
    out.pass = false;
    out.failure = ELECTRICAL_OFFSET_FAILURE_NONE;
    if (input == 0 || input->cpu_hz == 0u) {
        out.timing_scope_invalid = true;
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_TIMING_SCOPE_INVALID;
        return out;
    }

    const uint32_t core_20us_cycles = input->cpu_hz / 50000u;
    const uint32_t callback_50us_cycles = input->cpu_hz / 20000u;
    out.alignment_core_overrun =
        input->alignment_fast_hook_core_cycles >= core_20us_cycles;
    out.adc_callback_overrun =
        input->adc_callback_total_cycles >= callback_50us_cycles;
    out.old_main_scope_would_overrun =
        input->main_service_cycles >= core_20us_cycles;
    out.entry_init_overrun =
        input->alignment_state_transition_once_cycles >= core_20us_cycles;
    out.timing_scope_invalid = !input->scope_valid;
    out.interrupt_preempted = input->interrupt_preempted;
    out.profile_overhead =
        input->detailed_timing_enabled &&
        input->main_service_cycles > input->alignment_fast_hook_core_cycles;

    if (out.timing_scope_invalid) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_TIMING_SCOPE_INVALID;
    } else if (out.alignment_core_overrun) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CORE_OVERRUN;
    } else if (out.adc_callback_overrun) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ADC_CALLBACK_OVERRUN;
    } else {
        out.pass = true;
    }
    return out;
}

int electrical_offset_format_signed_milli(char *buf,
                                          size_t len,
                                          int32_t milli,
                                          bool always_sign)
{
    if (buf == 0 || len == 0u) {
        return -1;
    }
    const char *sign = "";
    if (milli < 0) {
        sign = "-";
    } else if (always_sign) {
        sign = "+";
    }
    uint32_t mag = (milli < 0) ? (uint32_t)(-milli) : (uint32_t)milli;
    return snprintf(buf,
                    len,
                    "%s%lu.%03lu",
                    sign,
                    (unsigned long)(mag / 1000u),
                    (unsigned long)(mag % 1000u));
}

ElectricalOffsetBringupState electrical_offset_handoff_next_state(bool handoff_pass,
                                                                  bool same_iteration)
{
    if (!handoff_pass) {
        return ELECTRICAL_OFFSET_STATE_FAIL;
    }
    (void)same_iteration;
    return ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP;
}

ElectricalOffsetPreAlignmentGateResult electrical_offset_pre_alignment_gate_evaluate(
    const ElectricalOffsetPreAlignmentGateInput *input)
{
    ElectricalOffsetPreAlignmentGateResult out;
    memset(&out, 0, sizeof(out));
    if (input == 0) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_PRE_ALIGNMENT_GATE_FAILED;
        return out;
    }

    out.admission_pass =
        input->admission_preflight_pass &&
        input->admission_disabled_ack &&
        !input->admission_active &&
        input->admission_hook_calls_after_disable == 0u &&
        input->overlap_count == 0u;
    out.handoff_pass =
        input->admission_handoff_pass &&
        input->snapshots_after_admission_disable >= 2u;
    out.offsets_pass = input->offset_pc0_valid && input->offset_pc1_valid;
    out.dc_cal_clear_pass = input->dc_cal_bits_clear;
    out.adc_pass =
        input->producer_gap_count == 0u &&
        input->producer_duplicate_count == 0u &&
        input->true_unpaired_count == 0u &&
        input->torn_count == 0u &&
        input->generation_mismatch_count == 0u;
    out.drv_pass =
        input->gate_enabled &&
        !input->nfault_asserted &&
        input->drv_ready;
    out.dispatch_pass =
        input->alignment_dispatch_enabled &&
        input->alignment_active &&
        !input->admission_active &&
        input->overlap_count == 0u;
    out.command_flag_zero_pass = input->command_flag_zero;
    out.v_alpha_zero_pass = input->v_alpha_zero;
    out.v_beta_zero_pass = input->v_beta_zero;
    out.voltage_magnitude_zero_pass = input->voltage_magnitude_zero;
    out.modulation_command_zero_pass = input->modulation_command_zero;
    out.last_applied_command_zero_pass = input->last_applied_command_zero;
    out.voltage_command_pending_clear_pass =
        input->voltage_command_pending_clear;
    out.voltage_command_seq_stable_pass = input->voltage_command_seq_stable;
    out.pwm_shadow_safe_pass = input->pwm_shadow_safe;
    out.pwm_active_safe_pass = input->pwm_active_safe;
    out.ccr1_safe_pass = input->ccr1_safe;
    out.ccr2_safe_pass = input->ccr2_safe;
    out.ccr3_safe_pass = input->ccr3_safe;
    out.ccr_safe_for_moe_off_pass = input->ccr_safe_for_moe_off;
    out.ccr_safe_for_moe_enable_pass = input->ccr_safe_for_moe_enable;
    out.ccr_alignment_start_ready_pass = input->ccr_alignment_start_ready;
    out.moe_still_off_pass = input->moe_off;
    out.software_zero_command_pass =
        out.command_flag_zero_pass &&
        out.v_alpha_zero_pass &&
        out.v_beta_zero_pass &&
        out.voltage_magnitude_zero_pass &&
        out.modulation_command_zero_pass;
    out.applied_zero_command_pass =
        out.last_applied_command_zero_pass &&
        out.voltage_command_pending_clear_pass &&
        out.voltage_command_seq_stable_pass &&
        out.pwm_shadow_safe_pass &&
        out.pwm_active_safe_pass &&
        out.ccr1_safe_pass &&
        out.ccr2_safe_pass &&
        out.ccr3_safe_pass &&
        out.ccr_safe_for_moe_off_pass &&
        out.moe_still_off_pass;
    out.zero_command_pass =
        out.software_zero_command_pass &&
        out.applied_zero_command_pass;
    out.pass =
        out.admission_pass &&
        out.handoff_pass &&
        out.offsets_pass &&
        out.dc_cal_clear_pass &&
        out.adc_pass &&
        out.drv_pass &&
        out.dispatch_pass &&
        out.zero_command_pass &&
        out.moe_still_off_pass;

    if (out.pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_NONE;
    } else if (!out.admission_pass) {
        out.failure = (input->overlap_count != 0u)
                          ? ELECTRICAL_OFFSET_FAILURE_HOOK_OVERLAP
                          : ELECTRICAL_OFFSET_FAILURE_ADMISSION_REJECTED;
    } else if (!out.handoff_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ADMISSION_HANDOFF_FAILED;
    } else if (!out.offsets_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_OFFSETS_INVALID;
    } else if (!out.dc_cal_clear_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_DC_CAL_NOT_CLEAR;
    } else if (!out.adc_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID;
    } else if (!out.drv_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY;
    } else if (!out.dispatch_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DISPATCH_NOT_READY;
    } else if (!out.software_zero_command_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO;
    } else if (!out.voltage_command_pending_clear_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_PENDING_VOLTAGE_UPDATE;
    } else if (!out.ccr1_safe_pass ||
               !out.ccr2_safe_pass ||
               !out.ccr3_safe_pass ||
               !out.ccr_safe_for_moe_off_pass ||
               !out.pwm_active_safe_pass ||
               !out.pwm_shadow_safe_pass ||
               !out.moe_still_off_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_CCR_NOT_SAFE;
    } else if (!out.applied_zero_command_pass) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_APPLIED;
    } else {
        out.failure = ELECTRICAL_OFFSET_FAILURE_PRE_ALIGNMENT_GATE_FAILED;
    }

    return out;
}

static float eo_point_offset(const ElectricalOffsetCalibrationConfig *cfg,
                             float theta_cmd,
                             float encoder_count)
{
    const float theta_m =
        EO_TWO_PI_F * encoder_count / (float)cfg->encoder_cpr;
    return electrical_offset_wrap_0_2pi(
        theta_cmd -
        ((float)cfg->encoder_direction * (float)cfg->pole_pairs * theta_m));
}

bool electrical_offset_calibration_evaluate(
    const ElectricalOffsetCalibrationConfig *config,
    const ElectricalOffsetCalibrationPoint points[ELECTRICAL_OFFSET_CAL_POINT_COUNT],
    float alignment_voltage_v,
    ElectricalOffsetCalibrationResult *result)
{
    ElectricalOffsetCalibrationConfig def =
        electrical_offset_calibration_default_config();
    const ElectricalOffsetCalibrationConfig *cfg =
        (config != 0) ? config : &def;

    if (result == 0) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if (points == 0 ||
        cfg->encoder_cpr <= 0 ||
        cfg->encoder_direction == 0 ||
        cfg->pole_pairs <= 0 ||
        !isfinite(alignment_voltage_v) ||
        alignment_voltage_v > cfg->alignment_voltage_limit_v) {
        result->fail_flags |= ELECTRICAL_OFFSET_CAL_FAIL_VOLTAGE_LIMIT;
        return false;
    }

    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    for (uint32_t i = 0u; i < ELECTRICAL_OFFSET_CAL_POINT_COUNT; ++i) {
        result->points[i] = points[i];
        if (!points[i].valid ||
            !isfinite(points[i].theta_cmd_rad) ||
            !isfinite(points[i].encoder_count_mean)) {
            result->fail_flags |= ELECTRICAL_OFFSET_CAL_FAIL_POINT_INVALID;
            continue;
        }
        result->point_offset_rad[i] =
            eo_point_offset(cfg, points[i].theta_cmd_rad, points[i].encoder_count_mean);
        sin_sum += sinf(result->point_offset_rad[i]);
        cos_sum += cosf(result->point_offset_rad[i]);
    }

    if ((result->fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_POINT_INVALID) != 0u) {
        return false;
    }

    result->electrical_offset_rad =
        electrical_offset_wrap_0_2pi(atan2f(sin_sum, cos_sum));
    result->electrical_offset_deg =
        result->electrical_offset_rad * 180.0f / EO_PI_F;

    for (uint32_t i = 0u; i < ELECTRICAL_OFFSET_CAL_POINT_COUNT; ++i) {
        const float spread_rad =
            electrical_offset_angle_distance_rad(result->point_offset_rad[i],
                                                 result->electrical_offset_rad);
        const float spread_deg = spread_rad * 180.0f / EO_PI_F;
        if (spread_deg > result->max_offset_spread_deg) {
            result->max_offset_spread_deg = spread_deg;
        }
    }
    if (result->max_offset_spread_deg > cfg->max_offset_spread_deg) {
        result->fail_flags |= ELECTRICAL_OFFSET_CAL_FAIL_SPREAD;
    }

    result->delta_count_0_to_pos120 =
        electrical_offset_count_delta(points[1].encoder_count_mean,
                                      points[0].encoder_count_mean,
                                      cfg->encoder_cpr);
    result->delta_count_0_to_neg120 =
        electrical_offset_count_delta(points[2].encoder_count_mean,
                                      points[0].encoder_count_mean,
                                      cfg->encoder_cpr);
    result->expected_delta_count =
        (float)cfg->encoder_cpr /
        ((float)cfg->pole_pairs * EO_THREE_F);

    const float pos_command_delta =
        eo_wrap_minus_pi_pi(points[1].theta_cmd_rad - points[0].theta_cmd_rad);
    const float neg_command_delta =
        eo_wrap_minus_pi_pi(points[2].theta_cmd_rad - points[0].theta_cmd_rad);
    float pos_equiv_command_delta = 0.0f;
    float neg_equiv_command_delta = 0.0f;
    float pos_error_counts = 0.0f;
    float neg_error_counts = 0.0f;
    const bool pos_candidate_ok =
        eo_best_equivalent_delta(cfg,
                                 pos_command_delta,
                                 result->delta_count_0_to_pos120,
                                 &pos_equiv_command_delta,
                                 &pos_error_counts);
    const bool neg_candidate_ok =
        eo_best_equivalent_delta(cfg,
                                 neg_command_delta,
                                 result->delta_count_0_to_neg120,
                                 &neg_equiv_command_delta,
                                 &neg_error_counts);
    const bool pos_ok =
        pos_candidate_ok &&
        pos_error_counts <= cfg->expected_delta_tolerance_counts;
    const bool neg_ok =
        neg_candidate_ok &&
        neg_error_counts <= cfg->expected_delta_tolerance_counts;
    result->encoder_direction_ok = pos_ok && neg_ok;
    if (!result->encoder_direction_ok) {
        result->fail_flags |= ELECTRICAL_OFFSET_CAL_FAIL_DIRECTION;
    }

    result->pole_pairs_est_pos =
        (fabsf(result->delta_count_0_to_pos120) > 0.001f)
            ? (fabsf(pos_equiv_command_delta) * (float)cfg->encoder_cpr /
               (EO_TWO_PI_F * fabsf(result->delta_count_0_to_pos120)))
            : 0.0f;
    result->pole_pairs_est_neg =
        (fabsf(result->delta_count_0_to_neg120) > 0.001f)
            ? (fabsf(neg_equiv_command_delta) * (float)cfg->encoder_cpr /
               (EO_TWO_PI_F * fabsf(result->delta_count_0_to_neg120)))
            : 0.0f;
    result->pole_pairs_est_average =
        0.5f * (result->pole_pairs_est_pos + result->pole_pairs_est_neg);
    result->pole_pairs_ok =
        fabsf(result->pole_pairs_est_average - (float)cfg->pole_pairs) <= 0.75f;
    if (!result->pole_pairs_ok) {
        result->fail_flags |= ELECTRICAL_OFFSET_CAL_FAIL_POLE_PAIRS;
    }

    result->electrical_offset_valid = result->fail_flags == ELECTRICAL_OFFSET_CAL_FAIL_NONE;
    return result->electrical_offset_valid;
}

void electrical_offset_alignment_protection_reset(
    ElectricalOffsetAlignmentProtectionState *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
    }
}

static void eo_mark_first_trip(ElectricalOffsetAlignmentProtectionResult *out,
                               const char *channel,
                               float current)
{
    if (out->first_trip_channel == 0) {
        out->first_trip_channel = channel;
        out->first_trip_current_a = current;
    }
}

ElectricalOffsetAlignmentProtectionResult electrical_offset_alignment_protection_update(
    ElectricalOffsetAlignmentProtectionState *state,
    const ElectricalOffsetAlignmentProtectionInput *input)
{
    ElectricalOffsetAlignmentProtectionResult out;
    memset(&out, 0, sizeof(out));
    out.failure = ELECTRICAL_OFFSET_FAILURE_NONE;

    if (state == 0 || input == 0 ||
        input->current_amp_per_count <= 0.0f ||
        input->soft_limit_a <= 0.0f ||
        input->phase_emergency_limit_a <= 0.0f ||
        input->raw_hard_limit_counts <= 0 ||
        input->soft_consecutive_required == 0u ||
        input->raw_min_safe_count >= input->raw_max_safe_count ||
        !input->current_finite ||
        !isfinite(input->iu_a) ||
        !isfinite(input->iv_a) ||
        !isfinite(input->iw_a) ||
        !isfinite(input->id_a) ||
        !isfinite(input->iq_a) ||
        !isfinite(input->phase_abs_a)) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_CURRENT_SENSOR_INVALID;
        out.trip = true;
        out.immediate_trip = true;
        out.first_trip_channel = "MODEL";
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CURRENT_MODEL_INVALID;
        return out;
    }

    if (!input->offset_valid) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_CURRENT_OFFSET_INVALID;
        out.trip = true;
        out.immediate_trip = true;
        out.first_trip_channel = "OFFSET";
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OFFSET_SHIFT;
        return out;
    }

    if (!input->nfault_ok) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_NFAULT;
        out.trip = true;
        out.immediate_trip = true;
        out.first_trip_channel = "NFAULT";
        out.failure = ELECTRICAL_OFFSET_FAILURE_DRV_RUNTIME_FAULT;
        return out;
    }

    if (input->raw_pc0 <= input->raw_min_safe_count ||
        input->raw_pc0 >= input->raw_max_safe_count ||
        input->raw_pc1 <= input->raw_min_safe_count ||
        input->raw_pc1 >= input->raw_max_safe_count) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_ADC_SATURATION;
        out.trip = true;
        out.immediate_trip = true;
        out.first_trip_channel = "ADC";
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ADC_SATURATION;
        return out;
    }

    if (input->delta_pc0_counts > input->raw_hard_limit_counts ||
        input->delta_pc0_counts < -input->raw_hard_limit_counts) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RAW_PC0_HARD;
        out.trip = true;
        out.immediate_trip = true;
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC0_OVERCURRENT;
        eo_mark_first_trip(&out, "RAW_PC0",
                           (float)input->delta_pc0_counts * input->current_amp_per_count);
    }
    if (input->delta_pc1_counts > input->raw_hard_limit_counts ||
        input->delta_pc1_counts < -input->raw_hard_limit_counts) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RAW_PC1_HARD;
        out.trip = true;
        out.immediate_trip = true;
        if (out.failure == ELECTRICAL_OFFSET_FAILURE_NONE) {
            out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC1_OVERCURRENT;
        }
        eo_mark_first_trip(&out, "RAW_PC1",
                           (float)input->delta_pc1_counts * input->current_amp_per_count);
    }

    const bool phase_hard =
        fabsf(input->iu_a) > input->phase_emergency_limit_a ||
        fabsf(input->iv_a) > input->phase_emergency_limit_a ||
        fabsf(input->iw_a) > input->phase_emergency_limit_a ||
        fabsf(input->id_a) > input->phase_emergency_limit_a ||
        fabsf(input->iq_a) > input->phase_emergency_limit_a ||
        input->phase_abs_a > input->phase_emergency_limit_a;
    if (phase_hard) {
        out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_PHASE_HARD;
        out.trip = true;
        out.immediate_trip = true;
        if (out.failure == ELECTRICAL_OFFSET_FAILURE_NONE) {
            out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_HARD_OVERCURRENT;
        }
    }

    const bool iu_soft = fabsf(input->iu_a) > input->soft_limit_a;
    const bool iv_soft = fabsf(input->iv_a) > input->soft_limit_a;
    const bool iw_soft = fabsf(input->iw_a) > input->soft_limit_a;
    const bool id_soft = fabsf(input->id_a) > input->soft_limit_a;
    const bool iq_soft = fabsf(input->iq_a) > input->soft_limit_a;
    const bool phase_soft = input->phase_abs_a > input->soft_limit_a;

    if (iu_soft) { out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IU; eo_mark_first_trip(&out, "IU", input->iu_a); }
    if (iv_soft) { out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IV; eo_mark_first_trip(&out, "IV", input->iv_a); }
    if (iw_soft) { out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_RECONSTRUCTED_IW; eo_mark_first_trip(&out, "IW", input->iw_a); }
    if (id_soft || iq_soft) { out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_DQ; }
    if (phase_soft) { out.source_mask |= ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_PHASE_SOFT; }

    if (out.immediate_trip) {
        state->hard_consecutive_count++;
    } else {
        state->hard_consecutive_count = 0u;
    }
    if (state->hard_consecutive_count > state->hard_consecutive_max) {
        state->hard_consecutive_max = state->hard_consecutive_count;
    }

    if (phase_soft || id_soft || iq_soft) {
        state->soft_consecutive_count++;
    } else {
        state->soft_consecutive_count = 0u;
    }
    if (state->soft_consecutive_count > state->soft_consecutive_max) {
        state->soft_consecutive_max = state->soft_consecutive_count;
    }
    out.soft_consecutive_count = state->soft_consecutive_count;
    out.hard_consecutive_count = state->hard_consecutive_count;

    if (!out.trip &&
        state->soft_consecutive_count >= input->soft_consecutive_required) {
        out.trip = true;
        out.soft_trip = true;
        out.failure = ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT;
    }

    if (!out.trip) {
        out.failure = ELECTRICAL_OFFSET_FAILURE_NONE;
    }
    if (out.first_trip_channel == 0) {
        out.first_trip_channel = "NONE";
    }
    return out;
}

ElectricalOffsetPwmZeroStartResult electrical_offset_pwm_zero_start_evaluate(
    const ElectricalOffsetPwmZeroStartInput *input)
{
    ElectricalOffsetPwmZeroStartResult out;
    memset(&out, 0, sizeof(out));
    if (input == 0) {
        return out;
    }

    out.command_zero =
        input->commanded_v_alpha_zero &&
        input->commanded_v_beta_zero &&
        input->applied_v_alpha_zero &&
        input->applied_v_beta_zero;
    out.duties_equal = input->duty_u_eq_v && input->duty_v_eq_w;
    out.ccrs_equal = input->ccr1_eq_ccr2 && input->ccr2_eq_ccr3;
    out.preload_complete =
        input->preload_ack &&
        input->tim_updates_after_preload &&
        input->pending_voltage_update_clear;
    out.safe_to_enable_moe =
        out.command_zero &&
        out.duties_equal &&
        out.ccrs_equal &&
        out.preload_complete &&
        input->dc_cal_bits_clear &&
        input->nfault_ok &&
        input->gate_enabled &&
        input->moe_off_before_enable;
    return out;
}

ElectricalOffsetPwmZeroClassification electrical_offset_pwm_zero_classify(
    const ElectricalOffsetPwmZeroClassifyInput *input)
{
    if (input == 0) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_UNKNOWN;
    }
    if (!input->ccrs_equal || !input->line_to_line_zero) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ZERO_STATE_INVALID;
    }
    if (!input->reconstruction_consistent && input->raw_within_limits) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_CURRENT_RECONSTRUCTION_INVALID;
    }
    if (input->near_switch_edge) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_ADC_SWITCHING_EDGE_CONTAMINATION;
    }
    if (input->stable_pc0_pc1_shift &&
        input->speed_near_zero &&
        input->nfault_ok) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ACTIVE_OFFSET_SHIFT;
    }
    if (input->current_ramp_like || input->encoder_motion) {
        return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_POSSIBLE_REAL_UNINTENDED_CURRENT;
    }
    return ELECTRICAL_OFFSET_PWM_ZERO_CLASS_UNKNOWN;
}

int32_t electrical_offset_reconstructed_iu_counts(int32_t iv_counts,
                                                  int32_t iw_counts)
{
    return -(iv_counts + iw_counts);
}

float electrical_offset_common_mode_shift_counts(float delta_pc0_counts,
                                                 float delta_pc1_counts)
{
    return 0.5f * (delta_pc0_counts + delta_pc1_counts);
}

float electrical_offset_differential_shift_counts(float delta_pc0_counts,
                                                  float delta_pc1_counts)
{
    return 0.5f * (delta_pc0_counts - delta_pc1_counts);
}

static uint32_t eo_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

uint32_t electrical_offset_min_phase_edge_distance_counts(uint32_t sample_count,
                                                         uint32_t ccr1,
                                                         uint32_t ccr2,
                                                         uint32_t ccr3)
{
    uint32_t min_counts = eo_abs_diff_u32(sample_count, ccr1);
    uint32_t d = eo_abs_diff_u32(sample_count, ccr2);
    if (d < min_counts) {
        min_counts = d;
    }
    d = eo_abs_diff_u32(sample_count, ccr3);
    if (d < min_counts) {
        min_counts = d;
    }
    return min_counts;
}

ElectricalOffsetTriggerSweepTiming electrical_offset_trigger_sweep_timing(
    const ElectricalOffsetTriggerSweepTimingInput *input)
{
    ElectricalOffsetTriggerSweepTiming out;
    memset(&out, 0, sizeof(out));
    if (input == 0 || input->arr == 0u || input->pclk2_hz == 0u ||
        input->apb2_prescaler == 0u) {
        return out;
    }

    const uint32_t tim_multiplier = (input->apb2_prescaler == 1u) ? 1u : 2u;
    out.tim_input_clock_hz = input->pclk2_hz * tim_multiplier;
    out.tim_counter_clock_hz =
        out.tim_input_clock_hz / (input->tim_prescaler + 1u);
    if (out.tim_counter_clock_hz == 0u) {
        return out;
    }
    out.timer_tick_ns = 1000000000.0f / (float)out.tim_counter_clock_hz;
    out.pwm_frequency_hz =
        (float)out.tim_counter_clock_hz / (2.0f * (float)(input->arr + 1u));
    out.deadtime_counts = input->deadtime_counts;
    out.deadtime_us =
        ((float)input->deadtime_counts * out.timer_tick_ns) / 1000.0f;

    const uint32_t start = input->phase_ccr + input->deadtime_counts;
    const uint32_t end =
        (input->arr > input->deadtime_counts) ? (input->arr - input->deadtime_counts) : 0u;
    out.low_side_window_start_count = start;
    out.low_side_window_end_count = (end > start) ? end : start;
    out.low_side_window_width_counts =
        (out.low_side_window_end_count > out.low_side_window_start_count)
            ? (out.low_side_window_end_count - out.low_side_window_start_count)
            : 0u;
    out.low_side_window_width_us =
        ((float)out.low_side_window_width_counts * out.timer_tick_ns) / 1000.0f;
    return out;
}

static bool eo_sweep_candidate_duplicate(
    const ElectricalOffsetTriggerSweepCandidate *candidates,
    uint32_t count,
    uint32_t trigger_count)
{
    for (uint32_t i = 0u; i < count; ++i) {
        if (candidates[i].programmed_trigger_count == trigger_count) {
            return true;
        }
    }
    return false;
}

static void eo_sweep_fill_candidate(ElectricalOffsetTriggerSweepCandidate *out,
                                    uint32_t index,
                                    float guard_us,
                                    uint32_t guard_counts,
                                    uint32_t trigger_count,
                                    const ElectricalOffsetTriggerSweepTiming *timing,
                                    bool baseline,
                                    bool quiet_center)
{
    memset(out, 0, sizeof(*out));
    out->candidate_index = index;
    out->requested_guard_time_us = guard_us;
    out->requested_guard_counts = guard_counts;
    out->programmed_trigger_count = trigger_count;
    out->trigger_direction = 1u; /* TIM1 OC4REF rising edge in PWM1 center-aligned mode occurs on down-count. */
    out->distance_to_nearest_switch_edge_counts =
        (trigger_count > timing->low_side_window_start_count)
            ? (trigger_count - timing->low_side_window_start_count)
            : (timing->low_side_window_start_count - trigger_count);
    out->distance_to_nearest_switch_edge_us =
        ((float)out->distance_to_nearest_switch_edge_counts *
         timing->timer_tick_ns) / 1000.0f;
    out->distance_to_deadtime_end_counts =
        (trigger_count > timing->low_side_window_start_count)
            ? (trigger_count - timing->low_side_window_start_count)
            : 0u;
    out->expected_low_side_state =
        trigger_count >= timing->low_side_window_start_count &&
        trigger_count <= timing->low_side_window_end_count;
    out->candidate_valid = out->expected_low_side_state;
    out->is_baseline = baseline;
    out->is_quiet_center = quiet_center;
}

uint32_t electrical_offset_trigger_sweep_generate_candidates(
    const ElectricalOffsetTriggerSweepTiming *timing,
    const float *guard_times_us,
    uint32_t guard_count,
    uint32_t baseline_trigger_count,
    ElectricalOffsetTriggerSweepCandidate *candidates,
    uint32_t max_candidates)
{
    if (timing == 0 || guard_times_us == 0 || candidates == 0 ||
        max_candidates == 0u || timing->tim_counter_clock_hz == 0u) {
        return 0u;
    }

    uint32_t count = 0u;
    if (count < max_candidates) {
        const bool valid =
            baseline_trigger_count >= timing->low_side_window_start_count &&
            baseline_trigger_count <= timing->low_side_window_end_count;
        (void)valid;
        eo_sweep_fill_candidate(&candidates[count],
                                count,
                                0.0f,
                                0u,
                                baseline_trigger_count,
                                timing,
                                true,
                                false);
        count++;
    }

    for (uint32_t i = 0u; i < guard_count && count < max_candidates; ++i) {
        const float guard_us = guard_times_us[i];
        const uint32_t guard_counts =
            (uint32_t)((guard_us * 1000.0f / timing->timer_tick_ns) + 0.5f);
        const uint32_t trigger =
            timing->low_side_window_start_count + guard_counts;
        if (trigger > timing->low_side_window_end_count ||
            eo_sweep_candidate_duplicate(candidates, count, trigger)) {
            continue;
        }
        eo_sweep_fill_candidate(&candidates[count],
                                count,
                                guard_us,
                                guard_counts,
                                trigger,
                                timing,
                                false,
                                false);
        count++;
    }

    if (count < max_candidates) {
        const uint32_t center =
            timing->low_side_window_start_count +
            (timing->low_side_window_width_counts / 2u);
        if (!eo_sweep_candidate_duplicate(candidates, count, center)) {
            const uint32_t guard_counts =
                (center > timing->low_side_window_start_count)
                    ? (center - timing->low_side_window_start_count)
                    : 0u;
            const float guard_us =
                ((float)guard_counts * timing->timer_tick_ns) / 1000.0f;
            eo_sweep_fill_candidate(&candidates[count],
                                    count,
                                    guard_us,
                                    guard_counts,
                                    center,
                                    timing,
                                    false,
                                    true);
            count++;
        }
    }

    return count;
}

uint32_t electrical_offset_trigger_sweep_generate_fixed_candidates(
    const ElectricalOffsetTriggerSweepTiming *timing,
    uint32_t trigger_count,
    uint32_t repeat_count,
    ElectricalOffsetTriggerSweepCandidate *candidates,
    uint32_t max_candidates)
{
    if (timing == 0 || candidates == 0 || max_candidates == 0u ||
        timing->tim_counter_clock_hz == 0u) {
        return 0u;
    }

    uint32_t count = 0u;
    const uint32_t guard_counts =
        (trigger_count > timing->low_side_window_start_count)
            ? (trigger_count - timing->low_side_window_start_count)
            : 0u;
    const float guard_us =
        ((float)guard_counts * timing->timer_tick_ns) / 1000.0f;
    const uint32_t repeats =
        (repeat_count < max_candidates) ? repeat_count : max_candidates;
    for (uint32_t i = 0u; i < repeats; ++i) {
        eo_sweep_fill_candidate(&candidates[count],
                                count,
                                guard_us,
                                guard_counts,
                                trigger_count,
                                timing,
                                false,
                                false);
        count++;
    }
    return count;
}

bool electrical_offset_trigger_sweep_candidate_result_valid(
    const ElectricalOffsetTriggerSweepCandidate *candidate,
    const ElectricalOffsetTriggerSweepCandidateResult *result)
{
    if (candidate == 0 || result == 0) {
        return false;
    }
    return candidate->candidate_valid &&
           result->candidate_valid &&
           !result->raw_hard_trip &&
           !result->emergency_trip &&
           !result->nfault_trip &&
           !result->producer_error &&
           !result->adc_saturation &&
           fabsf(result->delta_pc0_mean_counts) <= 2.0f &&
           fabsf(result->delta_pc1_mean_counts) <= 2.0f &&
           result->pc0_std_counts <= 3.0f &&
           result->pc1_std_counts <= 3.0f &&
           fabsf(result->iu_mean_a) <= 0.05f &&
           fabsf(result->iv_mean_a) <= 0.05f &&
           fabsf(result->iw_mean_a) <= 0.05f &&
           result->maximum_soft_consecutive_count < 4u &&
           result->encoder_delta_counts >= -2 &&
           result->encoder_delta_counts <= 2;
}

int32_t electrical_offset_trigger_sweep_recommend_candidate(
    const ElectricalOffsetTriggerSweepCandidate *candidates,
    const ElectricalOffsetTriggerSweepCandidateResult *results,
    uint32_t count)
{
    if (candidates == 0 || results == 0) {
        return -1;
    }
    int32_t best = -1;
    float best_score = 0.0f;
    for (uint32_t i = 0u; i < count; ++i) {
        if (!electrical_offset_trigger_sweep_candidate_result_valid(&candidates[i],
                                                                    &results[i])) {
            continue;
        }
        const float std_sum = results[i].pc0_std_counts + results[i].pc1_std_counts;
        const float edge_bonus = 0.001f * results[i].distance_to_edge_us;
        const float center_bonus = -0.0001f * fabsf(results[i].center_distance_counts);
        const float score =
            results[i].reconstructed_zero_current_error_a +
            0.001f * std_sum - edge_bonus - center_bonus;
        if (best < 0 || score < best_score) {
            best = (int32_t)i;
            best_score = score;
        }
    }
    return best;
}

ElectricalOffsetTriggerSweepClassification
electrical_offset_trigger_sweep_classify(bool baseline_bad,
                                         bool improves_with_guard,
                                         bool any_valid,
                                         bool all_valid_shifted,
                                         bool event_config_invalid)
{
    if (event_config_invalid) {
        return ELECTRICAL_OFFSET_TRIGGER_SWEEP_EVENT_CONFIG_INVALID;
    }
    if (!any_valid) {
        return ELECTRICAL_OFFSET_TRIGGER_SWEEP_NO_VALID_LOW_SIDE_WINDOW;
    }
    if (baseline_bad && improves_with_guard) {
        return ELECTRICAL_OFFSET_TRIGGER_SWEEP_TOO_CLOSE_TO_SWITCH_EDGE;
    }
    if (improves_with_guard) {
        return ELECTRICAL_OFFSET_TRIGGER_SWEEP_COMMON_MODE_SETTLING_LONG;
    }
    if (all_valid_shifted) {
        return ELECTRICAL_OFFSET_TRIGGER_SWEEP_ACTIVE_OFFSET_INDEPENDENT;
    }
    return ELECTRICAL_OFFSET_TRIGGER_SWEEP_INCONCLUSIVE;
}

void electrical_offset_alpha_beta_to_phase(float v_alpha,
                                           float v_beta,
                                           float *vu,
                                           float *vv,
                                           float *vw,
                                           float *max_line_to_line)
{
    const float u = v_alpha;
    const float v = -0.5f * v_alpha + 0.86602540378f * v_beta;
    const float w = -0.5f * v_alpha - 0.86602540378f * v_beta;
    if (vu != 0) { *vu = u; }
    if (vv != 0) { *vv = v; }
    if (vw != 0) { *vw = w; }
    if (max_line_to_line != 0) {
        float uv = fabsf(u - v);
        const float vw_ll = fabsf(v - w);
        const float wu = fabsf(w - u);
        if (vw_ll > uv) { uv = vw_ll; }
        if (wu > uv) { uv = wu; }
        *max_line_to_line = uv;
    }
}

float electrical_offset_expected_phase_current_phase_resistance(
    float alpha_beta_voltage_v,
    float phase_resistance_ohm)
{
    if (phase_resistance_ohm <= 0.0f || !isfinite(alpha_beta_voltage_v)) {
        return 0.0f;
    }
    return fabsf(alpha_beta_voltage_v) / phase_resistance_ohm;
}

float electrical_offset_expected_phase_current_line_line_resistance(
    float alpha_beta_voltage_v,
    float line_line_resistance_ohm)
{
    if (line_line_resistance_ohm <= 0.0f || !isfinite(alpha_beta_voltage_v)) {
        return 0.0f;
    }
    return fabsf(alpha_beta_voltage_v) / (0.5f * line_line_resistance_ohm);
}
