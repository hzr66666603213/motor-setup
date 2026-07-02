#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "current_sample_pipeline.h"
#include "drivers/drv8301.h"
#include "motor_identification.h"

static void check_true(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void check_close(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL: %s actual=%f expected=%f\n", message, actual, expected);
        exit(1);
    }
}

static float deterministic_noise(uint32_t i)
{
    uint32_t x = i * 1103515245u + 12345u;
    x ^= x >> 11u;
    x *= 2654435761u;
    const int32_t centered = (int32_t)(x % 2001u) - 1000;
    return (float)centered / 1000.0f;
}

static void test_sample_pipeline_binds_previous_command(void)
{
    CurrentSamplePipeline pipe;
    current_sample_pipeline_init(&pipe, 0.20f, 0.0f, 0.19f, 0.0f);

    CurrentSampleAppliedVoltage bound = current_sample_pipeline_bind_sample(&pipe);
    check_close(bound.command_v_alpha, 0.20f, 0.0001f, "initial bound command");

    current_sample_pipeline_write_next(&pipe, 0.80f, 0.0f, 0.78f, 0.0f);
    bound = current_sample_pipeline_bind_sample(&pipe);
    check_close(bound.command_v_alpha, 0.20f, 0.0001f,
                "sample before PWM advance remains bound to previous command");

    current_sample_pipeline_advance_pwm_cycle(&pipe);
    bound = current_sample_pipeline_bind_sample(&pipe);
    check_close(bound.command_v_alpha, 0.80f, 0.0001f,
                "next PWM sample binds new command");
    check_close(bound.applied_v_alpha, 0.78f, 0.0001f, "applied voltage follows active command");
}

static void test_rank_order_demux_keeps_physical_identity(void)
{
    HalAdcSnapshot snap = {0};
    current_sample_pipeline_demux_adc2(HAL_ADC_M0_ORDER_PC0_PC1,
                                       1000u, 1100u, 1200u, 1300u, &snap);
    check_true(snap.raw_pc0_m0_so1 == 1000u, "ORDER_A PC0 identity");
    check_true(snap.raw_pc1_m0_so2 == 1100u, "ORDER_A PC1 identity");
    check_true(snap.raw_u == 1000u && snap.raw_v == 1100u, "ORDER_A raw aliases");

    current_sample_pipeline_demux_adc2(HAL_ADC_M0_ORDER_PC1_PC0,
                                       1100u, 1000u, 1200u, 1300u, &snap);
    check_true(snap.raw_pc0_m0_so1 == 1000u, "ORDER_B PC0 identity");
    check_true(snap.raw_pc1_m0_so2 == 1100u, "ORDER_B PC1 identity");
    check_true(snap.raw_pc2_m1_so2 == 1200u, "ORDER_B PC2 identity");
    check_true(snap.raw_pc3_m1_so1 == 1300u, "ORDER_B PC3 identity");
}

static void test_identification_quality_helpers(void)
{
    check_true(MOTOR_IDENT_RISE_SAMPLES == 30u, "rise array stores 30 samples");
    check_true(MOTOR_IDENT_FALL_SAMPLES == 80u, "fall array stores 80 samples");

    MotorIdentPulseQuality quality;
    memset(&quality, 0, sizeof(quality));
    quality.rise_count = MOTOR_IDENT_RISE_SAMPLES;
    for (uint32_t i = 0u; i < quality.rise_count; ++i) {
        quality.rise_delta_i[i] = 0.50f + 0.01f * (float)i;
        quality.beta_delta_i[i] = 0.0f;
    }
    quality.beta_delta_i[3] = 0.09f;
    check_true(motor_ident_beta_quality_finalize(&quality, 0.20f),
               "single beta spike below averaged ratio does not delete repeat");

    float slow_rise[MOTOR_IDENT_RISE_SAMPLES] = {0};
    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        slow_rise[i] = 0.01f * (float)(i + 1u);
    }
    check_true(!motor_ident_dynamics_too_fast(slow_rise, MOTOR_IDENT_RISE_SAMPLES),
               "60 percent first sample is not dynamics too fast");
    slow_rise[0] = 0.91f;
    slow_rise[1] = 1.00f;
    check_true(motor_ident_dynamics_too_fast(slow_rise, 2u),
               "90 percent first sample is dynamics too fast");

    float pulse[MOTOR_IDENT_RISE_SAMPLES] = {0};
    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        pulse[i] = (i < 10u) ? 0.10f : 0.90f;
    }
    check_true(!motor_ident_pulse_too_short(pulse, MOTOR_IDENT_RISE_SAMPLES, 1.0f),
               "pulse too short uses last valid rise point, not tenth point");

    MotorIdentClassification unreliable =
        motor_ident_classify(true, true, false, 5.0f, 20.0f);
    check_true(unreliable.status == MOTOR_IDENT_STATUS_UNRELIABLE,
               "one unreliable level classifies as unreliable");
    check_true(!unreliable.current_loop_enable_permitted,
               "unreliable identification does not permit current loop");

    MotorIdentClassification pass =
        motor_ident_classify(true, true, true, 5.0f, 20.0f);
    check_true(pass.status == MOTOR_IDENT_STATUS_PASS, "reliable levels pass");
    check_true(pass.current_loop_enable_permitted, "only pass permits current loop");
}

static void test_rl_fit_math(void)
{
    const float resistance = 3.2f;
    const float inductance = 150.0e-6f;
    const float tau = inductance / resistance;
    const float sample_period = 50.0e-6f;
    const float voltage = 0.60f;
    const float amplitude = voltage / resistance;
    float rise[MOTOR_IDENT_RISE_SAMPLES];

    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        const float t = ((float)i + 1.0f) * sample_period;
        rise[i] = amplitude * (1.0f - expf(-t / tau));
    }

    MotorIdentFitResult fit =
        motor_ident_fit_rl_rise(rise,
                                MOTOR_IDENT_RISE_SAMPLES,
                                sample_period,
                                resistance,
                                voltage);
    check_true(fit.valid, "known RL fit valid");
    check_close(fit.inductance_h, inductance, 20.0e-6f, "known RL inductance");
    check_true(fit.r_squared > 0.99f, "known RL fit r squared");
}

static void test_robust_inductance_fit_rejects_spike_and_tail_noise(void)
{
    const float resistance = 3.2f;
    const float inductance = 1.0e-3f;
    const float tau = inductance / resistance;
    const float sample_period = 50.0e-6f;
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    const float amplitude = voltage_step / resistance;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];

    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        const float t = ((float)i + 1.0f) * sample_period;
        float y = amplitude * (1.0f - expf(-t / tau));
        y += deterministic_noise(i) * 0.006f;
        rise[i] = roundf(y / amp_per_count) * amp_per_count;
        rise_std[i] = 0.012f;
    }
    rise[6] += 0.20f; /* single switching/noise spike must not define amplitude */

    const float start = rise[MOTOR_IDENT_RISE_SAMPLES - 1u];
    for (uint32_t i = 0u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
        const float t = (float)i * sample_period;
        float y = start * expf(-t / tau);
        y += deterministic_noise(100u + i) * 0.006f;
        if (i > 35u) {
            y += deterministic_noise(500u + i) * 0.018f; /* noisy floor */
        }
        fall[i] = roundf(y / amp_per_count) * amp_per_count;
        fall_std[i] = (i > 35u) ? 0.025f : 0.010f;
    }

    MotorIdentRobustResult robust;
    check_true(motor_ident_robust_fit_level(rise,
                                            MOTOR_IDENT_RISE_SAMPLES,
                                            rise_std,
                                            fall,
                                            MOTOR_IDENT_FALL_SAMPLES,
                                            fall_std,
                                            sample_period,
                                            resistance,
                                            voltage_step,
                                            amp_per_count,
                                            &robust),
               "robust fit runs");
    check_true(robust.rise_fit.valid, "nonlinear rise fit valid with spike");
    check_true(robust.fall_fit.valid, "adaptive fall fit valid with noisy tail");
    check_true(robust.fall_fit.end_index < MOTOR_IDENT_FALL_SAMPLES - 10u,
               "fall fit excludes long noisy tail");
    check_true(robust.rise_fit.fitted_amplitude_a < robust.rise_peak_a,
               "single peak must not become fitted amplitude");
    check_close(robust.rise_fit.fitted_inductance_h, inductance, 150.0e-6f,
                "robust rise inductance near true value");
    check_close(robust.arx_smoothed_fixed_r.inductance_h, inductance, 150.0e-6f,
                "smoothed fixed-R ARX inductance near true value");
    check_true(robust.fused_inductance_h > 0.0f, "fused inductance produced");
}

static void test_robust_inductance_fit_noiseless_accuracy(void)
{
    const float resistance = 3.2f;
    const float inductance = 1.0e-3f;
    const float tau = inductance / resistance;
    const float sample_period = 50.0e-6f;
    const float voltage_step = 0.80f;
    const float amplitude = voltage_step / resistance;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];

    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        const float t = ((float)i + 1.0f) * sample_period;
        rise[i] = amplitude * (1.0f - expf(-t / tau));
        rise_std[i] = 0.0f;
    }
    const float start = rise[MOTOR_IDENT_RISE_SAMPLES - 1u];
    for (uint32_t i = 0u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
        const float t = (float)i * sample_period;
        fall[i] = start * expf(-t / tau);
        fall_std[i] = 0.0f;
    }

    MotorIdentRobustResult robust;
    check_true(motor_ident_robust_fit_level(rise,
                                            MOTOR_IDENT_RISE_SAMPLES,
                                            rise_std,
                                            fall,
                                            MOTOR_IDENT_FALL_SAMPLES,
                                            fall_std,
                                            sample_period,
                                            resistance,
                                            voltage_step,
                                            0.0201416f,
                                            &robust),
               "noiseless robust fit runs");
    check_close(robust.rise_fit.fitted_inductance_h, inductance, 50.0e-6f,
                "noiseless rise error under 5 percent");
    check_close(robust.fall_fit.fitted_inductance_h, inductance, 50.0e-6f,
                "noiseless fall error under 5 percent");
    check_close(robust.arx_fixed_r.inductance_h, inductance, 50.0e-6f,
                "noiseless ARX fixed-R error under 5 percent");
}

static void make_rl_curves(float inductance,
                           float voltage_step,
                           float amp_per_count,
                           float std_a,
                           float *rise,
                           float *rise_std,
                           float *fall,
                           float *fall_std)
{
    const float resistance = 3.2f;
    const float tau = inductance / resistance;
    const float sample_period = 50.0e-6f;
    const float amplitude = voltage_step / resistance;
    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        const float t = ((float)i + 1.0f) * sample_period;
        rise[i] = amplitude * (1.0f - expf(-t / tau));
        if (amp_per_count > 0.0f) {
            rise[i] = roundf(rise[i] / amp_per_count) * amp_per_count;
        }
        rise_std[i] = std_a;
    }
    const float start = rise[MOTOR_IDENT_RISE_SAMPLES - 1u];
    for (uint32_t i = 0u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
        const float t = (float)i * sample_period;
        fall[i] = start * expf(-t / tau);
        if (amp_per_count > 0.0f) {
            fall[i] = roundf(fall[i] / amp_per_count) * amp_per_count;
        }
        fall_std[i] = std_a;
    }
}

static MotorIdentRobustResult run_monotonic_case(float *rise,
                                                 float *rise_std,
                                                 float *fall,
                                                 float *fall_std,
                                                 float inductance,
                                                 float voltage_step,
                                                 float amp_per_count)
{
    MotorIdentRobustResult robust;
    const float resistance = 3.2f;
    const float sample_period = 50.0e-6f;
    check_true(motor_ident_robust_fit_level(rise,
                                            MOTOR_IDENT_RISE_SAMPLES,
                                            rise_std,
                                            fall,
                                            MOTOR_IDENT_FALL_SAMPLES,
                                            fall_std,
                                            sample_period,
                                            resistance,
                                            voltage_step,
                                            amp_per_count,
                                            &robust),
               "monotonic test robust fit runs");
    check_true(robust.rise_fit.valid, "monotonic test rise fit valid");
    check_true(robust.fall_fit.valid, "monotonic test fall fit valid");
    (void)inductance;
    return robust;
}

static void test_monotonic_ignores_rise_plateau_bounce(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(600.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    for (uint32_t i = 18u; i < MOTOR_IDENT_RISE_SAMPLES; i += 3u) {
        rise[i] -= 0.018f;
    }
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           600.0e-6f, voltage_step, amp_per_count);
    check_true(robust.monotonic_rise_ok,
               "rise plateau mild bounce after 90 percent is ignored");
}

static void test_monotonic_ignores_fall_noise_tail(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(600.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    for (uint32_t i = 25u; i < MOTOR_IDENT_FALL_SAMPLES; i += 4u) {
        fall[i] += 0.060f;
        fall_std[i] = 0.030f;
    }
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           600.0e-6f, voltage_step, amp_per_count);
    check_true(robust.fall_fit.end_index < 25u,
               "adaptive fall window excludes noisy tail");
    check_true(robust.monotonic_fall_ok,
               "fall noise tail does not fail monotonicity");
}

static void test_monotonic_rejects_clear_reverse_in_window(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.010f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(1.2e-3f, voltage_step, amp_per_count, 0.001f,
                   rise, rise_std, fall, fall_std);
    rise[7] = rise[6] - 0.050f;
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           1.2e-3f, voltage_step, amp_per_count);
    check_true(!robust.monotonic_rise_ok,
               "clear reverse step inside active rise window fails");
    check_true(robust.rise_monotonic.max_violation_counts > 1.0f,
               "clear reverse reports max violation in ADC counts");
}

static void test_monotonic_violation_ratio_threshold(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.010f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(4.0e-3f, voltage_step, amp_per_count, 0.001f,
                   rise, rise_std, fall, fall_std);
    for (uint32_t i = 4u; i <= 20u; i += 4u) {
        rise[i] = rise[i - 1u] - 0.008f;
    }
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           4.0e-3f, voltage_step, amp_per_count);
    check_true(robust.rise_monotonic.violation_ratio > 0.15f,
               "violation ratio exceeds threshold");
    check_true(!robust.monotonic_rise_ok,
               "too many small reverse steps fail");
}

static void test_monotonic_max_violation_threshold(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.010f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(4.0e-3f, voltage_step, amp_per_count, 0.001f,
                   rise, rise_std, fall, fall_std);
    rise[8] = rise[7] - 0.030f;
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           4.0e-3f, voltage_step, amp_per_count);
    check_true(robust.rise_monotonic.violation_ratio <= 0.15f,
               "single large reverse keeps violation ratio below threshold");
    check_true(!robust.monotonic_rise_ok,
               "single large reverse fails max violation threshold");
}

static void test_fall_window_skips_first_switching_transient(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(700.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    fall[0] += 0.090f;
    fall_std[0] = 0.012f;
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           700.0e-6f, voltage_step, amp_per_count);
    check_true(robust.fall_fit.start_index == 1u,
               "fall window skips first sample when it carries switching transient");
    check_true(robust.fall_fit.r_squared >= 0.95f,
               "first-sample transient skip restores fall R2");
}

static void test_fall_window_truncates_three_sigma_tail(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(700.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    for (uint32_t i = 13u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
        fall[i] = ((i & 1u) ? 0.010f : -0.010f);
        fall_std[i] = 0.030f;
    }
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           700.0e-6f, voltage_step, amp_per_count);
    check_true(robust.fall_fit.end_index < 13u,
               "fall window truncates samples after entering 3sigma floor");
}

static void test_fall_window_is_deterministic_with_single_outlier(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(700.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    fall[6] += 0.030f;
    MotorIdentRobustResult a =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           700.0e-6f, voltage_step, amp_per_count);
    MotorIdentRobustResult b =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           700.0e-6f, voltage_step, amp_per_count);
    check_true(a.fall_fit.start_index == b.fall_fit.start_index &&
               a.fall_fit.end_index == b.fall_fit.end_index,
               "fall window is deterministic with a single outlier");
    check_true(a.fall_fit.point_count >= 5u,
               "single outlier does not collapse window below minimum points");
}

static void test_fall_window_quantized_rl_reaches_r2_gate(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(700.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    MotorIdentRobustResult robust =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           700.0e-6f, voltage_step, amp_per_count);
    check_true(robust.fall_fit.r_squared >= 0.95f,
               "true quantized RL fall reaches R2 gate");
}

static void test_fall_window_model_mismatch_remains_unreliable(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];
    make_rl_curves(700.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    for (uint32_t i = 0u; i < 10u; ++i) {
        fall[i] = 0.20f - 0.012f * (float)i + ((i & 1u) ? 0.055f : -0.055f);
    }
    MotorIdentRobustResult robust;
    check_true(motor_ident_robust_fit_level(rise,
                                            MOTOR_IDENT_RISE_SAMPLES,
                                            rise_std,
                                            fall,
                                            MOTOR_IDENT_FALL_SAMPLES,
                                            fall_std,
                                            50.0e-6f,
                                            3.2f,
                                            voltage_step,
                                            amp_per_count,
                                            &robust),
               "model mismatch robust fit call returns");
    check_true(!robust.reliable,
               "obvious model mismatch is not promoted to reliable");
}

static void test_low_count_quantization_stays_diagnostic_only(void)
{
    const float amp_per_count = 0.0201416f;
    const float voltage_step_9_counts = 3.2f * amp_per_count * 9.0f;
    const float voltage_step_10_counts = 3.2f * amp_per_count * 10.0f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];

    make_rl_curves(650.0e-6f, voltage_step_9_counts, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    MotorIdentRobustResult nine =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           650.0e-6f, voltage_step_9_counts, amp_per_count);
    check_true(nine.rise_fit.valid && nine.fall_fit.valid,
               "9-count waveform remains fit-diagnosable");

    make_rl_curves(650.0e-6f, voltage_step_10_counts, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    MotorIdentRobustResult ten =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           650.0e-6f, voltage_step_10_counts, amp_per_count);
    check_true(ten.rise_fit.valid && ten.fall_fit.valid,
               "10-count waveform remains fit-diagnosable");
    check_true(ten.rise_peak_a > nine.rise_peak_a,
               "10-count command produces larger quantized peak than 9-count command");
    check_true((ten.rise_peak_a - nine.rise_peak_a) >= 0.5f * amp_per_count,
               "one-count-scale peak change is visible in low-count diagnostics");
}

static void test_fit_window_one_sample_sensitivity_is_visible(void)
{
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise[MOTOR_IDENT_RISE_SAMPLES];
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    float fall[MOTOR_IDENT_FALL_SAMPLES];
    float fall_std[MOTOR_IDENT_FALL_SAMPLES];

    make_rl_curves(650.0e-6f, voltage_step, amp_per_count, 0.010f,
                   rise, rise_std, fall, fall_std);
    MotorIdentRobustResult base =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           650.0e-6f, voltage_step, amp_per_count);
    fall[base.fall_fit.start_index] += amp_per_count;
    MotorIdentRobustResult shifted =
        run_monotonic_case(rise, rise_std, fall, fall_std,
                           650.0e-6f, voltage_step, amp_per_count);
    check_true(shifted.fall_fit.valid, "one-count fall-window perturbation still fits");
    check_true(fabsf(shifted.fall_fit.fitted_inductance_h -
                     base.fall_fit.fitted_inductance_h) > 5.0e-6f,
               "one ADC count at fit window edge is observable");
}

static void make_repeat_curves(float *rise_repeats,
                               float *fall_repeats,
                               uint32_t repeat_count,
                               float inductance,
                               float voltage_step,
                               float amp_per_count,
                               float rank_bias_counts)
{
    const float resistance = 3.2f;
    const float tau = inductance / resistance;
    const float sample_period = 50.0e-6f;
    const float amplitude = voltage_step / resistance;
    for (uint32_t rep = 0u; rep < repeat_count; ++rep) {
        const float rank_bias =
            ((rep & 1u) == 0u ? rank_bias_counts : -rank_bias_counts) *
            amp_per_count;
        for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
            const float t = ((float)i + 1.0f) * sample_period;
            float y = amplitude * (1.0f - expf(-t / tau));
            y += deterministic_noise(rep * 131u + i) * 0.35f * amp_per_count;
            y += rank_bias;
            rise_repeats[rep * MOTOR_IDENT_RISE_SAMPLES + i] =
                roundf(y / amp_per_count) * amp_per_count;
        }
        const float start =
            rise_repeats[rep * MOTOR_IDENT_RISE_SAMPLES +
                         (MOTOR_IDENT_RISE_SAMPLES - 1u)];
        for (uint32_t i = 0u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
            const float t = (float)i * sample_period;
            float y = start * expf(-t / tau);
            y += deterministic_noise(rep * 197u + 500u + i) *
                 0.35f * amp_per_count;
            y += rank_bias;
            fall_repeats[rep * MOTOR_IDENT_FALL_SAMPLES + i] =
                roundf(y / amp_per_count) * amp_per_count;
        }
    }
}

static void average_repeats(const float *repeats,
                            uint32_t repeat_count,
                            uint32_t sample_count,
                            float *mean)
{
    for (uint32_t i = 0u; i < sample_count; ++i) {
        float sum = 0.0f;
        for (uint32_t rep = 0u; rep < repeat_count; ++rep) {
            sum += repeats[rep * sample_count + i];
        }
        mean[i] = sum / (float)repeat_count;
    }
}

static void test_repeat_arx_stabilizes_rank_biased_quantized_data(void)
{
    enum { repeat_count = 32u };
    const float voltage_step = 0.80f;
    const float amp_per_count = 0.0201416f;
    float rise_repeats[repeat_count * MOTOR_IDENT_RISE_SAMPLES];
    float fall_repeats[repeat_count * MOTOR_IDENT_FALL_SAMPLES];
    float rise_mean[MOTOR_IDENT_RISE_SAMPLES];
    float fall_mean[MOTOR_IDENT_FALL_SAMPLES];
    float std[MOTOR_IDENT_FALL_SAMPLES];

    make_repeat_curves(rise_repeats, fall_repeats, repeat_count,
                       650.0e-6f, voltage_step, amp_per_count, 0.75f);
    MotorIdentArxFit repeat_fit;
    check_true(motor_ident_fit_repeated_arx_fixed_r(rise_repeats,
                                                    repeat_count,
                                                    MOTOR_IDENT_RISE_SAMPLES,
                                                    fall_repeats,
                                                    MOTOR_IDENT_FALL_SAMPLES,
                                                    50.0e-6f,
                                                    3.2f,
                                                    voltage_step,
                                                    &repeat_fit),
               "joint repeat fixed-R ARX fits");
    MotorIdentArxFit single_fit;
    check_true(motor_ident_fit_repeated_arx_fixed_r(rise_repeats,
                                                    1u,
                                                    MOTOR_IDENT_RISE_SAMPLES,
                                                    fall_repeats,
                                                    MOTOR_IDENT_FALL_SAMPLES,
                                                    50.0e-6f,
                                                    3.2f,
                                                    voltage_step,
                                                    &single_fit),
               "single repeat fixed-R ARX comparison fits");
    average_repeats(rise_repeats, repeat_count, MOTOR_IDENT_RISE_SAMPLES,
                    rise_mean);
    average_repeats(fall_repeats, repeat_count, MOTOR_IDENT_FALL_SAMPLES,
                    fall_mean);
    for (uint32_t i = 0u; i < MOTOR_IDENT_FALL_SAMPLES; ++i) {
        std[i] = amp_per_count;
    }
    float rise_std[MOTOR_IDENT_RISE_SAMPLES];
    for (uint32_t i = 0u; i < MOTOR_IDENT_RISE_SAMPLES; ++i) {
        rise_std[i] = amp_per_count;
    }
    MotorIdentRobustResult averaged =
        run_monotonic_case(rise_mean, rise_std, fall_mean, std,
                           650.0e-6f, voltage_step, amp_per_count);
    const float repeat_error =
        fabsf(repeat_fit.inductance_h - 650.0e-6f);
    const float single_error =
        fabsf(single_fit.inductance_h - 650.0e-6f);
    check_true(repeat_fit.valid, "joint repeat ARX marked valid");
    check_true(repeat_error < 120.0e-6f,
               "joint repeat fitting stays bounded on rank-biased quantized data");
    check_true(repeat_error <= single_error + 5.0e-6f,
               "joint repeat fitting is more stable than one representative repeat");
    check_true(averaged.arx_fixed_r.valid,
               "single averaged curve still produces a comparison ARX fit");
    check_true(fabsf(repeat_fit.inductance_h - averaged.arx_fixed_r.inductance_h) <
               180.0e-6f,
               "joint repeat and averaged curve stay in the same inductance band");
}

static void test_drv8301_gate_reset_preserves_config(void)
{
    const uint16_t control1 =
        (uint16_t)(drv8301_default_control1() |
                   ((uint16_t)DRV8301_GATE_CURRENT_0P7A << 0u));
    const uint16_t reset = drv8301_control1_with_gate_reset(control1);
    check_true((reset & (1u << 2)) != 0u, "gate reset bit set");
    check_true((reset & (uint16_t)~(1u << 2)) == (control1 & (uint16_t)~(1u << 2)),
               "gate reset helper preserves config bits");
}

int main(void)
{
    test_sample_pipeline_binds_previous_command();
    test_rank_order_demux_keeps_physical_identity();
    test_identification_quality_helpers();
    test_rl_fit_math();
    test_robust_inductance_fit_noiseless_accuracy();
    test_robust_inductance_fit_rejects_spike_and_tail_noise();
    test_monotonic_ignores_rise_plateau_bounce();
    test_monotonic_ignores_fall_noise_tail();
    test_monotonic_rejects_clear_reverse_in_window();
    test_monotonic_violation_ratio_threshold();
    test_monotonic_max_violation_threshold();
    test_fall_window_skips_first_switching_transient();
    test_fall_window_truncates_three_sigma_tail();
    test_fall_window_is_deterministic_with_single_outlier();
    test_fall_window_quantized_rl_reaches_r2_gate();
    test_fall_window_model_mismatch_remains_unreliable();
    test_low_count_quantization_stays_diagnostic_only();
    test_fit_window_one_sample_sensitivity_is_visible();
    test_repeat_arx_stabilizes_rank_biased_quantized_data();
    test_drv8301_gate_reset_preserves_config();
    printf("identification_pipeline_test PASS\n");
    return 0;
}
