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
    test_drv8301_gate_reset_preserves_config();
    printf("identification_pipeline_test PASS\n");
    return 0;
}
