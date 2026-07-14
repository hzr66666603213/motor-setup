#include "control/fixed_rotor_current_test.h"

#include "foc/foc_math.h"

#include <math.h>
#include <string.h>

#define FIXED_ROTOR_PI_F 3.14159265358979323846f

static bool fr_finite(float x)
{
    return isfinite(x) != 0;
}

static uint32_t fr_ticks_from_ms(uint32_t ms, float dt_s)
{
    const float ticks = ((float)ms * 0.001f) / dt_s;
    return (uint32_t)(ticks + 0.5f);
}

static int64_t fr_abs_i64(int64_t x)
{
    return (x < 0) ? -x : x;
}

static void fr_stage_stats_reset(FixedRotorCurrentStageStats *stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->settling_time_ms = -1.0f;
}

static void fr_stage_stats_update(FixedRotorCurrentStageStats *stats,
                                  const FixedRotorCurrentLogRecord *rec,
                                  float dt_s)
{
    const float n = (float)stats->sample_count;
    const float id_error = rec->id_ref_a - rec->id_a;
    const float iq_abs = fabsf(rec->iq_a);
    const float phase_abs = fmaxf(fabsf(rec->iu_a),
                                  fmaxf(fabsf(rec->iv_a), fabsf(rec->iw_a)));
    const float vector_v = sqrtf((rec->vd_v * rec->vd_v) + (rec->vq_v * rec->vq_v));

    stats->id_ref_mean = (stats->id_ref_mean * n + rec->id_ref_a) / (n + 1.0f);
    const float old_id_mean = stats->id_mean;
    stats->id_mean = (stats->id_mean * n + rec->id_a) / (n + 1.0f);
    stats->id_std += (rec->id_a - old_id_mean) * (rec->id_a - stats->id_mean);
    stats->id_error_mean = (stats->id_error_mean * n + id_error) / (n + 1.0f);
    if (fabsf(id_error) > stats->id_error_peak) {
        stats->id_error_peak = fabsf(id_error);
    }
    const float old_iq_mean = stats->iq_mean;
    stats->iq_mean = (stats->iq_mean * n + rec->iq_a) / (n + 1.0f);
    stats->iq_std += (rec->iq_a - old_iq_mean) * (rec->iq_a - stats->iq_mean);
    if (iq_abs > stats->iq_peak) {
        stats->iq_peak = iq_abs;
    }
    if (phase_abs > stats->phase_current_peak) {
        stats->phase_current_peak = phase_abs;
    }
    if (vector_v > stats->voltage_vector_peak) {
        stats->voltage_vector_peak = vector_v;
    }
    if (rec->saturation_active) {
        stats->saturation_count++;
    }
    stats->sample_count++;

    if (rec->id_ref_a > 0.001f) {
        const float overshoot = ((rec->id_a - rec->id_ref_a) / rec->id_ref_a) * 100.0f;
        if (overshoot > stats->overshoot_percent) {
            stats->overshoot_percent = overshoot;
        }
        if (stats->settling_time_ms < 0.0f && fabsf(id_error) <= 0.01f) {
            stats->settling_time_ms = (float)stats->sample_count * dt_s * 1000.0f;
        }
    }
    stats->steady_state_error_a = stats->id_error_mean;
    if (iq_abs > stats->cross_axis_iq_peak_a) {
        stats->cross_axis_iq_peak_a = iq_abs;
    }
}

static void fr_stage_stats_finalize(FixedRotorCurrentStageStats *stats)
{
    if (stats->sample_count > 1u) {
        stats->id_std = sqrtf(stats->id_std / (float)(stats->sample_count - 1u));
        stats->iq_std = sqrtf(stats->iq_std / (float)(stats->sample_count - 1u));
    } else {
        stats->id_std = 0.0f;
        stats->iq_std = 0.0f;
    }
}

static FixedRotorCurrentStageStats *fr_stats_for_state(FixedRotorCurrentTest *test,
                                                       FixedRotorCurrentTestState state)
{
    switch (state) {
    case FIXED_ROTOR_STATE_ENABLE_ZERO:
        return &test->enable_zero_stats;
    case FIXED_ROTOR_STATE_HOLD_ID_0P05:
        return &test->hold_0p05_stats;
    case FIXED_ROTOR_STATE_HOLD_ID_0P10:
        return &test->hold_0p10_stats;
    case FIXED_ROTOR_STATE_HOLD_ZERO:
        return &test->hold_zero_stats;
    default:
        return 0;
    }
}

static void fr_finalize_all_stats(FixedRotorCurrentTest *test)
{
    fr_stage_stats_finalize(&test->enable_zero_stats);
    fr_stage_stats_finalize(&test->hold_0p05_stats);
    fr_stage_stats_finalize(&test->hold_0p10_stats);
    fr_stage_stats_finalize(&test->hold_zero_stats);
}

FixedRotorCurrentTestConfig fixed_rotor_current_test_default_config(void)
{
    FixedRotorCurrentTestConfig cfg;
    cfg.phase_resistance_ohm = 3.20f;
    cfg.phase_inductance_h = 0.00066f;
    cfg.bandwidth_hz = 100.0f;
    cfg.voltage_limit_v = 1.00f;
    cfg.kaw = 2.0f * FIXED_ROTOR_PI_F * 100.0f;
    cfg.integrator_limit_v = 1.00f;
    cfg.dt_s = 0.00005f;
    cfg.id_ref_max_a = 0.10f;
    cfg.id_ramp_rate_a_per_s = 0.10f;
    cfg.phase_current_limit_a = 0.25f;
    cfg.dq_current_limit_a = 0.20f;
    cfg.iq_deviation_limit_a = 0.05f;
    cfg.iq_deviation_limit_ticks = fr_ticks_from_ms(50u, cfg.dt_s);
    cfg.tracking_error_limit_a = 0.10f;
    cfg.tracking_error_limit_ticks = fr_ticks_from_ms(50u, cfg.dt_s);
    cfg.saturation_limit_ticks = fr_ticks_from_ms(20u, cfg.dt_s);
    cfg.encoder_motion_limit_counts = 32;
    cfg.vbus_min_v = 7.0f;
    cfg.vbus_max_v = 13.0f;
    cfg.log_decimation = 20u;
    cfg.preflight_ticks = fr_ticks_from_ms(500u, cfg.dt_s);
    cfg.enable_zero_ticks = fr_ticks_from_ms(500u, cfg.dt_s);
    cfg.hold_0p05_ticks = fr_ticks_from_ms(500u, cfg.dt_s);
    cfg.hold_0p10_ticks = fr_ticks_from_ms(500u, cfg.dt_s);
    cfg.hold_zero_ticks = fr_ticks_from_ms(500u, cfg.dt_s);
    cfg.control_time_limit_us = 20.0f;
    return cfg;
}

void fixed_rotor_current_test_init(FixedRotorCurrentTest *test,
                                   const FixedRotorCurrentTestConfig *config)
{
    if (test == 0) {
        return;
    }

    memset(test, 0, sizeof(*test));
    test->config = (config != 0) ? *config : fixed_rotor_current_test_default_config();
    current_controller_init(&test->controller,
                            0.0f,
                            0.0f,
                            test->config.voltage_limit_v);
    current_controller_tune_from_rl(&test->controller,
                                    test->config.phase_resistance_ohm,
                                    test->config.phase_inductance_h,
                                    test->config.bandwidth_hz,
                                    test->config.voltage_limit_v);
    current_controller_set_antiwindup(&test->controller,
                                      test->config.kaw,
                                      test->config.integrator_limit_v);
    fixed_rotor_current_test_reset(test);
}

void fixed_rotor_current_test_reset(FixedRotorCurrentTest *test)
{
    if (test == 0) {
        return;
    }

    test->state = FIXED_ROTOR_STATE_PREFLIGHT;
    test->result = FIXED_ROTOR_RESULT_RUNNING;
    test->fault_code = FIXED_ROTOR_FAULT_NONE;
    test->control_tick_seq = 0u;
    test->voltage_command_seq = 0u;
    test->missed_control_tick_count = 0u;
    test->duplicate_control_tick_count = 0u;
    test->worst_case_control_cycles = 0u;
    test->worst_case_control_time_us = 0.0f;
    test->state_ticks = 0u;
    test->last_adc_seq = 0u;
    test->have_last_adc_seq = false;
    test->theta_latched = false;
    test->theta_test_rad = 0.0f;
    test->encoder_start_count = 0;
    test->encoder_motion_max_counts = 0;
    test->id_ref_a = 0.0f;
    test->iq_deviation_ticks = 0u;
    test->tracking_error_ticks = 0u;
    test->saturation_ticks = 0u;
    test->log_count = 0u;
    test->log_dropped = 0u;
    current_controller_reset(&test->controller);
    fr_stage_stats_reset(&test->enable_zero_stats);
    fr_stage_stats_reset(&test->hold_0p05_stats);
    fr_stage_stats_reset(&test->hold_0p10_stats);
    fr_stage_stats_reset(&test->hold_zero_stats);
}

static void fr_fail(FixedRotorCurrentTest *test, uint32_t fault_code)
{
    test->fault_code |= fault_code;
    test->state = FIXED_ROTOR_STATE_FAIL;
    test->result = FIXED_ROTOR_RESULT_FAIL;
    test->id_ref_a = 0.0f;
    current_controller_reset(&test->controller);
    fr_finalize_all_stats(test);
}

void fixed_rotor_current_test_request_start(FixedRotorCurrentTest *test)
{
    if (test == 0 ||
        test->result != FIXED_ROTOR_RESULT_RUNNING ||
        test->fault_code != FIXED_ROTOR_FAULT_NONE) {
        return;
    }

    test->state = FIXED_ROTOR_STATE_ENABLE_ZERO;
    test->state_ticks = 0u;
    test->control_tick_seq = 0u;
    test->voltage_command_seq = 0u;
    test->missed_control_tick_count = 0u;
    test->duplicate_control_tick_count = 0u;
    test->last_adc_seq = 0u;
    test->have_last_adc_seq = false;
    test->theta_latched = false;
    test->theta_test_rad = 0.0f;
    test->encoder_start_count = 0;
    test->encoder_motion_max_counts = 0;
    test->id_ref_a = 0.0f;
    test->iq_deviation_ticks = 0u;
    test->tracking_error_ticks = 0u;
    test->saturation_ticks = 0u;
    test->log_count = 0u;
    test->log_dropped = 0u;
    current_controller_reset(&test->controller);
    fr_stage_stats_reset(&test->enable_zero_stats);
    fr_stage_stats_reset(&test->hold_0p05_stats);
    fr_stage_stats_reset(&test->hold_0p10_stats);
    fr_stage_stats_reset(&test->hold_zero_stats);
}

void fixed_rotor_current_test_force_fault(FixedRotorCurrentTest *test,
                                          uint32_t fault_code)
{
    if (test == 0 || fault_code == FIXED_ROTOR_FAULT_NONE) {
        return;
    }
    if (test->result == FIXED_ROTOR_RESULT_PASS ||
        test->result == FIXED_ROTOR_RESULT_FAIL) {
        test->fault_code |= fault_code;
        return;
    }
    fr_fail(test, fault_code);
}

void fixed_rotor_current_test_note_execution_time(FixedRotorCurrentTest *test,
                                                  uint32_t cycles,
                                                  float time_us)
{
    if (test == 0) {
        return;
    }

    if (!fr_finite(time_us)) {
        fr_fail(test, FIXED_ROTOR_FAULT_NAN_INF);
        return;
    }

    if (time_us > test->worst_case_control_time_us) {
        test->worst_case_control_time_us = time_us;
        test->worst_case_control_cycles = cycles;
    }

    if (test->result == FIXED_ROTOR_RESULT_RUNNING &&
        test->config.control_time_limit_us > 0.0f &&
        time_us > test->config.control_time_limit_us) {
        fr_fail(test, FIXED_ROTOR_FAULT_CONTROL_TIME);
    }
}

static void fr_advance_state(FixedRotorCurrentTest *test,
                             FixedRotorCurrentTestState next)
{
    test->state = next;
    test->state_ticks = 0u;
    if (next == FIXED_ROTOR_STATE_COMPLETE) {
        test->id_ref_a = 0.0f;
        test->result = FIXED_ROTOR_RESULT_PASS;
        current_controller_reset(&test->controller);
        fr_finalize_all_stats(test);
    }
}

static bool fr_valid_inputs(const FixedRotorCurrentTestInput *in)
{
    return (in != 0) &&
           fr_finite(in->theta_test_rad) &&
           fr_finite(in->iv_a) &&
           fr_finite(in->iw_a) &&
           fr_finite(in->vbus_v);
}

static uint32_t fr_input_faults(const FixedRotorCurrentTest *test,
                                const FixedRotorCurrentTestInput *in,
                                float iu,
                                float id,
                                float iq)
{
    uint32_t fault = 0u;
    if (!fr_valid_inputs(in)) {
        fault |= FIXED_ROTOR_FAULT_NAN_INF;
    }
    if (!in->adc_valid) {
        fault |= FIXED_ROTOR_FAULT_ADC_SEQ_GAP;
    }
    if (!in->encoder_valid || !fr_finite(in->theta_test_rad)) {
        fault |= FIXED_ROTOR_FAULT_ENCODER;
    }
    if (!in->nfault_ok) {
        fault |= FIXED_ROTOR_FAULT_NFAULT;
    }
    if (!in->drv_ok || in->fault_active) {
        fault |= FIXED_ROTOR_FAULT_DRV;
    }
    if (!in->m1_safe) {
        fault |= FIXED_ROTOR_FAULT_M1;
    }
    if (!in->pwm_ccr_ok) {
        fault |= FIXED_ROTOR_FAULT_PWM_CCR;
    }
    if ((in->vbus_v < test->config.vbus_min_v) ||
        (in->vbus_v > test->config.vbus_max_v)) {
        fault |= FIXED_ROTOR_FAULT_VBUS;
    }
    if ((fabsf(iu) > test->config.phase_current_limit_a) ||
        (fabsf(in->iv_a) > test->config.phase_current_limit_a) ||
        (fabsf(in->iw_a) > test->config.phase_current_limit_a)) {
        fault |= FIXED_ROTOR_FAULT_PHASE_CURRENT_LIMIT;
    }
    if ((fabsf(id) > test->config.dq_current_limit_a) ||
        (fabsf(iq) > test->config.dq_current_limit_a)) {
        fault |= FIXED_ROTOR_FAULT_DQ_CURRENT_LIMIT;
    }
    if (test->theta_latched) {
        const int64_t motion = fr_abs_i64(in->encoder_count - test->encoder_start_count);
        if (motion > test->config.encoder_motion_limit_counts) {
            fault |= FIXED_ROTOR_FAULT_ROTOR_MOVED;
        }
    }
    return fault;
}

static void fr_fill_record(FixedRotorCurrentTest *test,
                           const FixedRotorCurrentTestInput *in,
                           const CurrentControllerOutput *cc_out,
                           float iu,
                           float id,
                           float iq,
                           FixedRotorCurrentLogRecord *rec)
{
    rec->state = test->state;
    rec->control_tick_seq = test->control_tick_seq;
    rec->adc_seq = in->adc_seq;
    rec->voltage_command_seq = test->voltage_command_seq;
    rec->encoder_count = in->encoder_count;
    rec->theta_test_rad = test->theta_test_rad;
    rec->id_ref_a = test->id_ref_a;
    rec->iq_ref_a = 0.0f;
    rec->iu_a = iu;
    rec->iv_a = in->iv_a;
    rec->iw_a = in->iw_a;
    rec->id_a = id;
    rec->iq_a = iq;
    rec->error_d_a = test->id_ref_a - id;
    rec->error_q_a = -iq;
    rec->vd_unsat_v = cc_out->vd_unsat_v;
    rec->vq_unsat_v = cc_out->vq_unsat_v;
    rec->vd_v = cc_out->vd_v;
    rec->vq_v = cc_out->vq_v;
    rec->v_alpha_v = cc_out->v_alpha_v;
    rec->v_beta_v = cc_out->v_beta_v;
    rec->integrator_d_v = cc_out->integrator_d_v;
    rec->integrator_q_v = cc_out->integrator_q_v;
    rec->vbus_v = in->vbus_v;
    rec->saturation_active = cc_out->saturation_active;
    rec->fault_code = test->fault_code;
}

static void fr_log_record(FixedRotorCurrentTest *test,
                          const FixedRotorCurrentLogRecord *rec)
{
    if ((test->config.log_decimation == 0u) ||
        ((test->control_tick_seq % test->config.log_decimation) != 0u)) {
        return;
    }
    if (test->log_count < FIXED_ROTOR_CURRENT_TEST_LOG_CAPACITY) {
        test->log[test->log_count++] = *rec;
    } else {
        test->log_dropped++;
    }
}

void fixed_rotor_current_test_step(FixedRotorCurrentTest *test,
                                   const FixedRotorCurrentTestInput *input,
                                   FixedRotorCurrentTestOutput *output)
{
    if (output != 0) {
        memset(output, 0, sizeof(*output));
    }
    if (test == 0 || input == 0 || output == 0) {
        return;
    }

    output->state = test->state;
    output->result = test->result;

    if (test->result == FIXED_ROTOR_RESULT_PASS ||
        test->result == FIXED_ROTOR_RESULT_FAIL) {
        output->safe_shutdown_request = true;
        output->done = true;
        output->fault_code = test->fault_code;
        output->result = test->result;
        return;
    }

    test->control_tick_seq++;

    if (!test->theta_latched) {
        test->theta_test_rad = input->theta_test_rad;
        test->encoder_start_count = input->encoder_count;
        test->theta_latched = true;
    }
    const int64_t motion = fr_abs_i64(input->encoder_count - test->encoder_start_count);
    if (motion > test->encoder_motion_max_counts) {
        test->encoder_motion_max_counts = motion;
    }

    if (test->have_last_adc_seq) {
        if (input->adc_seq == test->last_adc_seq) {
            test->duplicate_control_tick_count++;
            fr_fail(test, FIXED_ROTOR_FAULT_ADC_DUPLICATE);
        } else if (input->adc_seq != (test->last_adc_seq + 1u)) {
            test->missed_control_tick_count++;
            fr_fail(test, FIXED_ROTOR_FAULT_ADC_SEQ_GAP);
        }
    }
    test->last_adc_seq = input->adc_seq;
    test->have_last_adc_seq = true;

    float iu = 0.0f;
    float i_alpha = 0.0f;
    float i_beta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    current_controller_clarke_vw(input->iv_a,
                                 input->iw_a,
                                 &iu,
                                 &i_alpha,
                                 &i_beta);
    current_controller_park(i_alpha, i_beta, test->theta_test_rad, &id, &iq);

    if (test->result != FIXED_ROTOR_RESULT_FAIL) {
        const uint32_t input_faults = fr_input_faults(test, input, iu, id, iq);
        if (input_faults != 0u) {
            fr_fail(test, input_faults);
        }
    }

    const bool power_state =
        (test->state != FIXED_ROTOR_STATE_PREFLIGHT) &&
        (test->state != FIXED_ROTOR_STATE_COMPLETE) &&
        (test->state != FIXED_ROTOR_STATE_FAIL);

    if (test->state == FIXED_ROTOR_STATE_RAMP_ID_0P05) {
        test->id_ref_a = current_controller_ramp_toward(test->id_ref_a,
                                                        0.05f,
                                                        test->config.id_ramp_rate_a_per_s,
                                                        test->config.dt_s);
        if (test->id_ref_a >= 0.05f) {
            fr_advance_state(test, FIXED_ROTOR_STATE_HOLD_ID_0P05);
        }
    } else if (test->state == FIXED_ROTOR_STATE_RAMP_ID_0P10) {
        test->id_ref_a = current_controller_ramp_toward(test->id_ref_a,
                                                        test->config.id_ref_max_a,
                                                        test->config.id_ramp_rate_a_per_s,
                                                        test->config.dt_s);
        if (test->id_ref_a >= test->config.id_ref_max_a) {
            fr_advance_state(test, FIXED_ROTOR_STATE_HOLD_ID_0P10);
        }
    } else if (test->state == FIXED_ROTOR_STATE_RAMP_ZERO) {
        test->id_ref_a = current_controller_ramp_toward(test->id_ref_a,
                                                        0.0f,
                                                        test->config.id_ramp_rate_a_per_s,
                                                        test->config.dt_s);
        if (test->id_ref_a <= 0.0f) {
            fr_advance_state(test, FIXED_ROTOR_STATE_HOLD_ZERO);
        }
    } else if (test->state == FIXED_ROTOR_STATE_PREFLIGHT ||
               test->state == FIXED_ROTOR_STATE_ENABLE_ZERO ||
               test->state == FIXED_ROTOR_STATE_HOLD_ZERO) {
        test->id_ref_a = 0.0f;
    }

    if (test->id_ref_a > test->config.id_ref_max_a) {
        test->id_ref_a = test->config.id_ref_max_a;
    }

    CurrentControllerInput cc_in;
    CurrentControllerOutput cc_out;
    cc_in.id_ref_a = test->id_ref_a;
    cc_in.iq_ref_a = 0.0f;
    cc_in.id_measured_a = id;
    cc_in.iq_measured_a = iq;
    cc_in.theta_rad = test->theta_test_rad;
    cc_in.vbus_v = input->vbus_v;
    cc_in.dt_s = test->config.dt_s;
    cc_in.enable = power_state && input->pwm_allowed &&
                   (test->result != FIXED_ROTOR_RESULT_FAIL);
    cc_in.fault_active = input->fault_active ||
                         (test->result == FIXED_ROTOR_RESULT_FAIL);
    current_controller_update_dq(&test->controller, &cc_in, &cc_out);
    if (cc_in.enable && !cc_out.valid) {
        fr_fail(test, FIXED_ROTOR_FAULT_NAN_INF);
    }
    if (cc_in.enable && test->result != FIXED_ROTOR_RESULT_FAIL) {
        test->voltage_command_seq++;
    }

    FixedRotorCurrentLogRecord rec;
    fr_fill_record(test, input, &cc_out, iu, id, iq, &rec);
    FixedRotorCurrentStageStats *stats = fr_stats_for_state(test, test->state);
    if (stats != 0) {
        fr_stage_stats_update(stats, &rec, test->config.dt_s);
    }
    fr_log_record(test, &rec);

    if (fabsf(iq) > test->config.iq_deviation_limit_a) {
        test->iq_deviation_ticks++;
        if (test->iq_deviation_ticks > test->config.iq_deviation_limit_ticks) {
            fr_fail(test, FIXED_ROTOR_FAULT_IQ_DEVIATION);
        }
    } else {
        test->iq_deviation_ticks = 0u;
    }

    const bool stable_ref =
        (test->state == FIXED_ROTOR_STATE_HOLD_ID_0P05) ||
        (test->state == FIXED_ROTOR_STATE_HOLD_ID_0P10) ||
        (test->state == FIXED_ROTOR_STATE_HOLD_ZERO);
    if (stable_ref &&
        fabsf(test->id_ref_a - id) > test->config.tracking_error_limit_a) {
        test->tracking_error_ticks++;
        if (test->tracking_error_ticks > test->config.tracking_error_limit_ticks) {
            fr_fail(test, FIXED_ROTOR_FAULT_TRACKING);
        }
    } else {
        test->tracking_error_ticks = 0u;
    }

    if (cc_out.saturation_active) {
        test->saturation_ticks++;
        if (test->saturation_ticks > test->config.saturation_limit_ticks) {
            fr_fail(test, FIXED_ROTOR_FAULT_SATURATION);
        }
    } else {
        test->saturation_ticks = 0u;
    }

    if (test->result != FIXED_ROTOR_RESULT_FAIL) {
        test->state_ticks++;
        switch (test->state) {
        case FIXED_ROTOR_STATE_PREFLIGHT:
            if (test->state_ticks >= test->config.preflight_ticks) {
                fr_advance_state(test, FIXED_ROTOR_STATE_ENABLE_ZERO);
            }
            break;
        case FIXED_ROTOR_STATE_ENABLE_ZERO:
            if (test->state_ticks >= test->config.enable_zero_ticks) {
                fr_advance_state(test, FIXED_ROTOR_STATE_RAMP_ID_0P05);
            }
            break;
        case FIXED_ROTOR_STATE_HOLD_ID_0P05:
            if (test->state_ticks >= test->config.hold_0p05_ticks) {
                fr_advance_state(test, FIXED_ROTOR_STATE_RAMP_ID_0P10);
            }
            break;
        case FIXED_ROTOR_STATE_HOLD_ID_0P10:
            if (test->state_ticks >= test->config.hold_0p10_ticks) {
                fr_advance_state(test, FIXED_ROTOR_STATE_RAMP_ZERO);
            }
            break;
        case FIXED_ROTOR_STATE_HOLD_ZERO:
            if (test->state_ticks >= test->config.hold_zero_ticks) {
                fr_advance_state(test, FIXED_ROTOR_STATE_COMPLETE);
            }
            break;
        default:
            break;
        }
    }

    output->state = test->state;
    output->result = test->result;
    output->fault_code = test->fault_code;
    output->id_ref_a = test->id_ref_a;
    output->iq_ref_a = 0.0f;
    output->vd_v = (test->result == FIXED_ROTOR_RESULT_FAIL) ? 0.0f : cc_out.vd_v;
    output->vq_v = (test->result == FIXED_ROTOR_RESULT_FAIL) ? 0.0f : cc_out.vq_v;
    output->v_alpha_v = (test->result == FIXED_ROTOR_RESULT_FAIL) ? 0.0f : cc_out.v_alpha_v;
    output->v_beta_v = (test->result == FIXED_ROTOR_RESULT_FAIL) ? 0.0f : cc_out.v_beta_v;
    output->power_stage_request =
        (test->state != FIXED_ROTOR_STATE_PREFLIGHT) &&
        (test->state != FIXED_ROTOR_STATE_COMPLETE) &&
        (test->state != FIXED_ROTOR_STATE_FAIL);
    output->pwm_output_request = output->power_stage_request;
    output->safe_shutdown_request =
        (test->state == FIXED_ROTOR_STATE_COMPLETE) ||
        (test->state == FIXED_ROTOR_STATE_FAIL);
    output->done = output->safe_shutdown_request;
}

void fixed_rotor_current_test_fast_isr(FixedRotorCurrentTest *test,
                                       const FixedRotorCurrentTestInput *input,
                                       FixedRotorCurrentTestOutput *output)
{
    fixed_rotor_current_test_step(test, input, output);
}

void fixed_rotor_current_test_service_main(const FixedRotorCurrentTest *test,
                                           FixedRotorCurrentTestOutput *output)
{
    if (output != 0) {
        memset(output, 0, sizeof(*output));
    }
    if (test == 0 || output == 0) {
        return;
    }

    output->state = test->state;
    output->result = test->result;
    output->fault_code = test->fault_code;
    output->id_ref_a = test->id_ref_a;
    output->iq_ref_a = 0.0f;
    output->safe_shutdown_request =
        (test->result == FIXED_ROTOR_RESULT_PASS) ||
        (test->result == FIXED_ROTOR_RESULT_FAIL) ||
        (test->state == FIXED_ROTOR_STATE_COMPLETE) ||
        (test->state == FIXED_ROTOR_STATE_FAIL);
    output->done = output->safe_shutdown_request;
}

const char *fixed_rotor_current_test_state_name(FixedRotorCurrentTestState state)
{
    switch (state) {
    case FIXED_ROTOR_STATE_PREFLIGHT: return "PREFLIGHT";
    case FIXED_ROTOR_STATE_ENABLE_ZERO: return "ENABLE_ZERO";
    case FIXED_ROTOR_STATE_RAMP_ID_0P05: return "RAMP_ID_0P05";
    case FIXED_ROTOR_STATE_HOLD_ID_0P05: return "HOLD_ID_0P05";
    case FIXED_ROTOR_STATE_RAMP_ID_0P10: return "RAMP_ID_0P10";
    case FIXED_ROTOR_STATE_HOLD_ID_0P10: return "HOLD_ID_0P10";
    case FIXED_ROTOR_STATE_RAMP_ZERO: return "RAMP_ZERO";
    case FIXED_ROTOR_STATE_HOLD_ZERO: return "HOLD_ZERO";
    case FIXED_ROTOR_STATE_COMPLETE: return "COMPLETE";
    case FIXED_ROTOR_STATE_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}

const char *fixed_rotor_current_test_result_name(FixedRotorCurrentTestResult result)
{
    switch (result) {
    case FIXED_ROTOR_RESULT_NOT_RUN: return "NOT_RUN";
    case FIXED_ROTOR_RESULT_RUNNING: return "RUNNING";
    case FIXED_ROTOR_RESULT_PASS: return "PASS";
    case FIXED_ROTOR_RESULT_FAIL: return "FAIL";
    default: return "UNKNOWN";
    }
}

float fixed_rotor_current_test_kp(const FixedRotorCurrentTest *test)
{
    return (test != 0) ? test->controller.kp : 0.0f;
}

float fixed_rotor_current_test_ki(const FixedRotorCurrentTest *test)
{
    return (test != 0) ? test->controller.ki : 0.0f;
}

float fixed_rotor_current_test_ki_times_ts(const FixedRotorCurrentTest *test)
{
    return (test != 0) ? (test->controller.ki * test->config.dt_s) : 0.0f;
}
