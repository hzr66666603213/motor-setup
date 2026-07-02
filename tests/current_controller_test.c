#include "control/current_controller.h"
#include "control/fixed_rotor_current_test.h"

#include <math.h>
#include <stdio.h>

#define EPS 1.0e-4f

static int g_failures = 0;

static void check_true(int condition, const char *name)
{
    if (!condition) {
        printf("[FAIL] %s\n", name);
        g_failures++;
    } else {
        printf("[ OK ] %s\n", name);
    }
}

static int nearf(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

static CurrentController make_controller(void)
{
    CurrentController cc;
    current_controller_init(&cc, 0.0f, 0.0f, 1.0f);
    current_controller_tune_from_rl(&cc, 3.20f, 0.00066f, 100.0f, 1.0f);
    current_controller_set_antiwindup(&cc, 6.28318530718f * 100.0f, 1.0f);
    return cc;
}

static CurrentControllerOutput cc_step(CurrentController *cc,
                                       float id_ref,
                                       float iq_ref,
                                       float id,
                                       float iq,
                                       float theta)
{
    CurrentControllerInput in;
    CurrentControllerOutput out;
    in.id_ref_a = id_ref;
    in.iq_ref_a = iq_ref;
    in.id_measured_a = id;
    in.iq_measured_a = iq;
    in.theta_rad = theta;
    in.vbus_v = 11.7f;
    in.dt_s = 0.00005f;
    in.enable = true;
    in.fault_active = false;
    current_controller_update_dq(cc, &in, &out);
    return out;
}

static void test_math_and_pi_signs(void)
{
    float iu = 0.0f;
    float alpha = 0.0f;
    float beta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float va = 0.0f;
    float vb = 0.0f;
    CurrentController cc = make_controller();
    CurrentControllerOutput out;

    printf("\n== current_controller math and signs ==\n");

    current_controller_clarke_vw(-0.5f, -0.5f, &iu, &alpha, &beta);
    check_true(nearf(iu, 1.0f, EPS), "Clarke VW reconstructs iu");
    check_true(nearf(alpha, 1.0f, EPS), "Clarke alpha=iu");
    check_true(nearf(beta, 0.0f, EPS), "Clarke beta=(iv-iw)/sqrt3");

    current_controller_park(1.0f, 0.0f, 0.0f, &id, &iq);
    check_true(nearf(id, 1.0f, EPS), "Park id sign");
    check_true(nearf(iq, 0.0f, EPS), "Park iq sign");

    current_controller_inverse_park(1.0f, 0.0f, 0.0f, &va, &vb);
    check_true(nearf(va, 1.0f, EPS), "inverse Park alpha sign");
    check_true(nearf(vb, 0.0f, EPS), "inverse Park beta sign");

    out = cc_step(&cc, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f);
    check_true(out.vd_v > 0.0f, "positive id error produces positive vd");

    current_controller_reset(&cc);
    out = cc_step(&cc, 0.0f, 0.10f, 0.0f, 0.0f, 0.0f);
    check_true(out.vq_v > 0.0f, "positive iq error produces positive vq");

    current_controller_reset(&cc);
    out = cc_step(&cc, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    check_true(nearf(out.vd_v, 0.0f, EPS) && nearf(out.vq_v, 0.0f, EPS),
               "zero error produces zero output");
}

static void test_limit_antiwindup_and_reset(void)
{
    CurrentController cc = make_controller();
    CurrentControllerInput in;
    CurrentControllerOutput out;

    printf("\n== current_controller limit and antiwindup ==\n");

    out = cc_step(&cc, 10.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    check_true(sqrtf(out.vd_v * out.vd_v + out.vq_v * out.vq_v) <= 1.0001f,
               "circular voltage limit <=1V");
    check_true(out.saturation_active, "saturation flag set");

    for (int i = 0; i < 300; ++i) {
        out = cc_step(&cc, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    check_true(fabsf(cc.integrator_d_v) <= 1.0001f, "integrator clamp");

    for (int i = 0; i < 400; ++i) {
        out = cc_step(&cc, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f);
    }
    check_true(isfinite(out.vd_v) && fabsf(out.vd_v) <= 1.0001f,
               "antiwindup recovery stays finite");

    in.id_ref_a = 0.1f;
    in.iq_ref_a = 0.0f;
    in.id_measured_a = 0.0f;
    in.iq_measured_a = 0.0f;
    in.theta_rad = 0.0f;
    in.vbus_v = 11.7f;
    in.dt_s = 0.00005f;
    in.enable = false;
    in.fault_active = false;
    current_controller_update_dq(&cc, &in, &out);
    check_true(nearf(cc.integrator_d_v, 0.0f, EPS) &&
                   nearf(cc.integrator_q_v, 0.0f, EPS),
               "disable clears integrators");

    (void)cc_step(&cc, 0.1f, 0.1f, 0.0f, 0.0f, 0.0f);
    in.enable = true;
    in.fault_active = true;
    current_controller_update_dq(&cc, &in, &out);
    check_true(nearf(cc.integrator_d_v, 0.0f, EPS) &&
                   nearf(cc.integrator_q_v, 0.0f, EPS),
               "fault clears integrators");

    check_true(nearf(current_controller_ramp_toward(0.0f, 0.1f, 0.1f, 0.5f),
                     0.05f,
                     EPS),
               "reference ramp rate");
}

static FixedRotorCurrentTestInput nominal_input(uint32_t seq)
{
    FixedRotorCurrentTestInput in;
    in.time_us = (uint64_t)seq * 50ull;
    in.adc_seq = seq;
    in.encoder_count = 1000;
    in.theta_test_rad = 1.0f;
    in.iv_a = 0.0f;
    in.iw_a = 0.0f;
    in.vbus_v = 11.7f;
    in.adc_valid = true;
    in.encoder_valid = true;
    in.nfault_ok = true;
    in.drv_ok = true;
    in.m1_safe = true;
    in.pwm_ccr_ok = true;
    in.pwm_allowed = true;
    in.fault_active = false;
    return in;
}

static void test_fixed_rotor_protections(void)
{
    FixedRotorCurrentTest test;
    FixedRotorCurrentTestConfig cfg = fixed_rotor_current_test_default_config();
    FixedRotorCurrentTestOutput out;
    FixedRotorCurrentTestInput in;

    printf("\n== fixed rotor protections ==\n");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    in.iv_a = 0.30f;
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.result == FIXED_ROTOR_RESULT_FAIL) &&
                   ((out.fault_code & FIXED_ROTOR_FAULT_PHASE_CURRENT_LIMIT) != 0u),
               "phase current test limit abort");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    fixed_rotor_current_test_step(&test, &in, &out);
    in = nominal_input(2u);
    in.encoder_count += 33;
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ROTOR_MOVED) != 0u,
               "encoder movement >32 abort");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    fixed_rotor_current_test_step(&test, &in, &out);
    in = nominal_input(3u);
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ADC_SEQ_GAP) != 0u,
               "ADC seq gap abort");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    fixed_rotor_current_test_step(&test, &in, &out);
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ADC_DUPLICATE) != 0u,
               "duplicate tick no repeated integration");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    in.iv_a = NAN;
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_NAN_INF) != 0u,
               "NaN/Inf safe fail");

    fixed_rotor_current_test_init(&test, &cfg);
    in = nominal_input(1u);
    in.m1_safe = false;
    fixed_rotor_current_test_step(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_M1) != 0u,
               "M1 abnormal fail");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_note_execution_time(&test, 1600u, 9.52f);
    check_true(test.result == FIXED_ROTOR_RESULT_RUNNING &&
                   test.worst_case_control_cycles == 1600u,
               "control execution time records worst case");
    fixed_rotor_current_test_note_execution_time(&test, 4000u, 23.81f);
    check_true((test.result == FIXED_ROTOR_RESULT_FAIL) &&
                   ((test.fault_code & FIXED_ROTOR_FAULT_CONTROL_TIME) != 0u),
               "control execution timeout abort");

    check_true(out.safe_shutdown_request, "safe shutdown requested on fail");
}

static void test_fixed_rotor_full_sequence_without_plant(void)
{
    FixedRotorCurrentTest test;
    FixedRotorCurrentTestConfig cfg = fixed_rotor_current_test_default_config();
    FixedRotorCurrentTestOutput out = {0};
    FixedRotorCurrentTestInput in;

    printf("\n== fixed rotor sequence no plant ==\n");
    cfg.tracking_error_limit_ticks = 1000000u;
    cfg.saturation_limit_ticks = 1000000u;
    fixed_rotor_current_test_init(&test, &cfg);
    for (uint32_t seq = 1u; seq < 160000u && !out.done; ++seq) {
        in = nominal_input(seq);
        fixed_rotor_current_test_step(&test, &in, &out);
    }
    check_true(out.result == FIXED_ROTOR_RESULT_PASS, "sequence reaches COMPLETE");
    check_true(out.safe_shutdown_request, "safe shutdown requested on COMPLETE");
    check_true(test.hold_0p05_stats.sample_count > 0u &&
                   test.hold_0p10_stats.sample_count > 0u,
               "hold stage stats collected");
    check_true(test.voltage_command_seq > 0u &&
                   test.voltage_command_seq <= test.control_tick_seq,
               "voltage command sequence tracks accepted control ticks");
}

static void test_discrete_rl_current_loop(void)
{
    FixedRotorCurrentTestConfig cfg = fixed_rotor_current_test_default_config();
    CurrentController cc;
    CurrentControllerOutput out;
    float id = 0.0f;
    float iq = 0.0f;
    float delayed_vd = 0.0f;
    float delayed_vq = 0.0f;
    int saturation_count = 0;
    int finite_ok = 1;

    printf("\n== fixed rotor discrete RL sim ==\n");

    current_controller_init(&cc, 0.0f, 0.0f, cfg.voltage_limit_v);
    current_controller_tune_from_rl(&cc,
                                    cfg.phase_resistance_ohm,
                                    cfg.phase_inductance_h,
                                    cfg.bandwidth_hz,
                                    cfg.voltage_limit_v);
    current_controller_set_antiwindup(&cc, cfg.kaw, cfg.integrator_limit_v);

    for (int i = 0; i < 4000; ++i) {
        out = cc_step(&cc, 0.10f, 0.0f, id, iq, 0.0f);
        id += ((delayed_vd - cfg.phase_resistance_ohm * id) /
               cfg.phase_inductance_h) * cfg.dt_s;
        iq += ((delayed_vq - cfg.phase_resistance_ohm * iq) /
               cfg.phase_inductance_h) * cfg.dt_s;
        delayed_vd = out.vd_v;
        delayed_vq = out.vq_v;
        if (out.saturation_active) {
            saturation_count++;
        }
        if (!current_controller_is_finite_output(&out)) {
            finite_ok = 0;
        }
    }

    printf("rl_sim: id=% .6f iq=% .6f kp=% .6f ki=% .6f kiTs=% .6f sat=%d\n",
           id,
           iq,
           cc.kp,
           cc.ki,
           cc.ki * cfg.dt_s,
           saturation_count);
    check_true(fabsf(id - 0.10f) <= 0.005f, "id_ref=0.10A closed-loop stable");
    check_true(fabsf(iq) <= 0.001f, "iq_ref=0 keeps iq near 0");
    check_true(saturation_count == 0, "no sustained saturation in sim");
    check_true(finite_ok, "sim output no NaN/Inf");
    check_true(nearf(cc.kp, 0.41469f, 0.001f), "Kp from L*wc");
    check_true(nearf(cc.ki, 2010.62f, 0.5f), "Ki from R*wc");
    check_true(nearf(cc.ki * cfg.dt_s, 0.10053f, 0.001f), "Ki*Ts not double multiplied");
}

int main(void)
{
    test_math_and_pi_signs();
    test_limit_antiwindup_and_reset();
    test_fixed_rotor_protections();
    test_fixed_rotor_full_sequence_without_plant();
    test_discrete_rl_current_loop();

    if (g_failures != 0) {
        printf("\ncurrent_controller_test failed: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("\ncurrent_controller_test passed.\n");
    return 0;
}
