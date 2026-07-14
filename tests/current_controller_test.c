#include "control/current_controller.h"
#include "control/current_sensor_admission.h"
#include "control/current_sensor_noise_diagnostic.h"
#include "control/electrical_offset_calibration.h"
#include "control/fixed_rotor_current_test.h"
#include "control/rotating_dq_current_test.h"
#include "control/velocity_controller.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

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

static void test_velocity_overspeed_evidence(void)
{
    VelocityCountWindow window;
    VelocityOverspeedAnalysis analysis;

    printf("\n== velocity overspeed evidence ==\n");

    velocity_count_window_reset(&window);
    const int32_t acceleration[] = {2, 3, 4, 7, 22};
    for (uint32_t i = 0u; i < 5u; ++i) {
        (void)velocity_count_window_update_rpm(&window,
                                               acceleration[i],
                                               0.01f,
                                               4096u);
    }
    velocity_count_window_analyze_overspeed(&window,
                                            0.01f,
                                            4096u,
                                            25.0f,
                                            &analysis);
    check_true(analysis.evidence ==
                   VELOCITY_OVERSPEED_EVIDENCE_SHORT_ACCELERATION &&
                   analysis.chronological_delta_counts[0] == 2 &&
                   analysis.chronological_delta_counts[4] == 22,
               "same-direction history classifies short acceleration");
    (void)velocity_count_window_update_rpm(&window, 23, 0.01f, 4096u);
    velocity_count_window_analyze_overspeed(&window,
                                            0.01f,
                                            4096u,
                                            25.0f,
                                            &analysis);
    check_true(analysis.chronological_delta_counts[0] == 3 &&
                   analysis.chronological_delta_counts[4] == 23 &&
                   analysis.sum_counts == 59,
               "wrapped speed window exports chronological estimator inputs");

    velocity_count_window_reset(&window);
    const int32_t spike[] = {0, 1, 0, 1, 22};
    for (uint32_t i = 0u; i < 5u; ++i) {
        (void)velocity_count_window_update_rpm(&window,
                                               spike[i],
                                               0.01f,
                                               4096u);
    }
    velocity_count_window_analyze_overspeed(&window,
                                            0.01f,
                                            4096u,
                                            25.0f,
                                            &analysis);
    check_true(analysis.evidence ==
                   VELOCITY_OVERSPEED_EVIDENCE_ISOLATED_DELTA_SPIKE &&
                   analysis.newest_window_abs_fraction > 0.90f,
               "dominant final delta classifies isolated spike candidate");

    velocity_count_window_reset(&window);
    const int32_t sustained[] = {18, 18, 18, 18, 18};
    for (uint32_t i = 0u; i < 5u; ++i) {
        (void)velocity_count_window_update_rpm(&window,
                                               sustained[i],
                                               0.01f,
                                               4096u);
    }
    velocity_count_window_analyze_overspeed(&window,
                                            0.01f,
                                            4096u,
                                            25.0f,
                                            &analysis);
    check_true(analysis.evidence == VELOCITY_OVERSPEED_EVIDENCE_WINDOWED,
               "full-window overspeed remains a sustained fault");

    velocity_count_window_reset(&window);
    const int32_t ambiguous[] = {-8, 2, -8, 1, 22};
    for (uint32_t i = 0u; i < 5u; ++i) {
        (void)velocity_count_window_update_rpm(&window,
                                               ambiguous[i],
                                               0.01f,
                                               4096u);
    }
    velocity_count_window_analyze_overspeed(&window,
                                            0.01f,
                                            4096u,
                                            25.0f,
                                            &analysis);
    check_true(analysis.evidence == VELOCITY_OVERSPEED_EVIDENCE_INCONCLUSIVE,
               "mixed history does not force an overspeed diagnosis");
}

static void test_velocity_low_speed_coupling_and_run_guard(void)
{
    VelocityController controller;
    VelocityContinuousRunGuard guard;
    VelocityBreakawayProbe breakaway;
    VelocityBreakawayHandoff handoff;
    VelocityCountWindow handoff_window;

    printf("\n== velocity low-speed coupling and continuous guard ==\n");

    velocity_controller_init(&controller, 0.10f, 0.50f, 0.030f, 10.0f);
    for (uint32_t i = 0u; i < 100u; ++i) {
        (void)velocity_controller_update_gated(&controller,
                                               1.0f,
                                               0.0f,
                                               0.01f,
                                               false);
    }
    check_true(controller.integrator_a == 0.0f &&
                   controller.last_output_a > 0.0f,
               "unready low-speed estimator keeps P output but cannot integrate");
    (void)velocity_controller_update_gated(&controller,
                                           0.10f,
                                           0.0f,
                                           0.01f,
                                           true);
    check_true(controller.integrator_a > 0.0f,
               "ready velocity estimator enables the unchanged PI integrator");

    check_true(nearf(velocity_bounded_profile_target_rpm(
                         0.0f, 5.0f, 2.0f, 1.0f, 2.0f),
                     0.0f,
                     EPS) &&
                   nearf(velocity_bounded_profile_target_rpm(
                             1.0f, 5.0f, 2.0f, 1.0f, 2.0f),
                         1.0f,
                         EPS) &&
                   nearf(velocity_bounded_profile_target_rpm(
                             2.0f, 5.0f, 2.0f, 1.0f, 2.0f),
                         2.0f,
                         EPS) &&
                   nearf(velocity_bounded_profile_target_rpm(
                             3.5f, 5.0f, 2.0f, 1.0f, 2.0f),
                         2.0f,
                         EPS) &&
                   nearf(velocity_bounded_profile_target_rpm(
                             4.5f, 5.0f, 2.0f, 1.0f, 2.0f),
                         1.0f,
                         EPS) &&
                   nearf(velocity_bounded_profile_target_rpm(
                             5.0f, 5.0f, 2.0f, 1.0f, 2.0f),
                         0.0f,
                         EPS),
               "five-second bounded profile performs 0-to-2-to-0 rpm");

    check_true(nearf(velocity_hold_then_fall_target_rpm(
                         0.0f, 5.0f, 2.0f, 1.0f),
                     2.0f,
                     EPS) &&
                   nearf(velocity_hold_then_fall_target_rpm(
                             2.9f, 5.0f, 2.0f, 1.0f),
                         2.0f,
                         EPS) &&
                   nearf(velocity_hold_then_fall_target_rpm(
                             4.0f, 5.0f, 2.0f, 1.0f),
                         1.0f,
                         EPS) &&
                   nearf(velocity_hold_then_fall_target_rpm(
                             5.0f, 5.0f, 2.0f, 1.0f),
                         0.0f,
                         EPS),
               "handoff profile holds 2rpm then returns to zero in five seconds");

    check_true(nearf(velocity_hold_then_fall_target_rpm(
                          13.9f, 15.0f, 2.0f, 2.0f),
                     2.0f,
                     EPS) &&
                   nearf(velocity_hold_then_fall_target_rpm(
                             14.5f, 15.0f, 2.0f, 2.0f),
                         1.0f,
                         EPS) &&
                   nearf(velocity_hold_then_fall_target_rpm(
                             15.0f, 15.0f, 2.0f, 2.0f),
                         0.0f,
                         EPS),
                "fifteen-second handoff profile preserves a bounded final ramp to zero");

    {
        const uint32_t durations_s[] = {5u, 15u, 30u, 60u};
        RotatingDqCurrentTestConfig cfg =
            rotating_dq_current_test_default_config();
        cfg.single_direction_positive_only = true;
        cfg.enable_zero_diagnostic_only = false;
        cfg.iq_target_a = 0.030f;
        cfg.iq_ramp_rate_a_per_s = 4.0f;
        cfg.hold_zero_ticks = 5000u;

        for (size_t i = 0u;
             i < sizeof(durations_s) / sizeof(durations_s[0]);
             ++i) {
            cfg.iq_hold_ticks = durations_s[i] * 20000u;
            const uint32_t timeout_ms =
                rotating_dq_current_test_supervisor_timeout_ms(&cfg, 2000u);
            const uint32_t configured_ms = durations_s[i] * 1000u;
            check_true(timeout_ms > configured_ms,
                       "supervisor timeout cannot truncate configured run");
            check_true(timeout_ms <= configured_ms + 3000u,
                       "supervisor timeout margin remains bounded");
        }
    }

    velocity_controller_reset(&controller);
    velocity_controller_preload_output(&controller, 0.030f);
    check_true(nearf(controller.last_output_a, 0.030f, EPS) &&
                   nearf(controller.integrator_a, 0.0f, EPS),
               "handoff preload is continuously limited without integrator preload");

    velocity_count_window_reset(&handoff_window);
    (void)velocity_count_window_set_samples(&handoff_window, 5u);
    (void)velocity_count_window_update_rpm(&handoff_window,
                                           2,
                                           0.01f,
                                           4096u);
    velocity_controller_prepare_bumpless_handoff(&controller,
                                                  &handoff_window,
                                                  0.030f);
    check_true(nearf(controller.last_output_a, 0.030f, EPS) &&
                   nearf(controller.integrator_a, 0.0f, EPS) &&
                   handoff_window.sample_count == 0u,
               "bumpless handoff restarts post-breakaway speed feedback and preserves bounded torque");

    check_true(nearf(velocity_controller_apply_hold_direction_guard(
                         &controller, -0.010f, 2.0f, false),
                     0.0f,
                     EPS) &&
                   nearf(controller.last_output_a, 0.0f, EPS),
               "positive low-speed hold rejects quantization-driven reverse torque");
    check_true(nearf(velocity_controller_apply_hold_direction_guard(
                         &controller, -0.010f, 1.0f, true),
                     -0.010f,
                     EPS),
               "bounded reverse braking remains available during the fall phase");

    check_true(nearf(velocity_controller_apply_coulomb_feedforward(
                         &controller, 0.0f, 2.0f, 0.010f, true),
                     0.010f,
                     EPS) &&
                   nearf(controller.integrator_a, 0.0f, EPS),
               "hold friction feedforward adds bounded torque without touching the integrator");
    check_true(nearf(velocity_controller_apply_coulomb_feedforward(
                         &controller, 0.015f, 2.0f, 0.010f, true),
                     0.025f,
                     EPS),
               "PI plus friction feedforward remains inside the authorized 30mA total limit");
    check_true(nearf(velocity_controller_apply_coulomb_feedforward(
                         &controller, -0.006f, 1.0f, 0.010f, false),
                     -0.006f,
                     EPS),
               "fall phase disables friction feedforward for bounded braking");

    velocity_continuous_guard_init(&guard, 100u);
    check_true(!velocity_continuous_guard_start(&guard, 0u),
               "continuous mode cannot start without explicit arm");
    check_true(!velocity_continuous_guard_arm(&guard, false, 0u) &&
                   (guard.fault_latched &
                    VELOCITY_CONTINUOUS_FAULT_PREFLIGHT) != 0u,
               "failed preflight latches and blocks continuous mode");
    check_true(!velocity_continuous_guard_clear_fault(&guard, false) &&
                   velocity_continuous_guard_clear_fault(&guard, true),
               "latched continuous fault clears only with safe outputs");
    check_true(velocity_continuous_guard_arm(&guard, true, 10u) &&
                   velocity_continuous_guard_start(&guard, 10u) &&
                   velocity_continuous_guard_poll(&guard, true, 100u),
               "explicit arm and start enter guarded continuous mode");
    velocity_continuous_guard_heartbeat(&guard, 100u);
    check_true(velocity_continuous_guard_poll(&guard, true, 200u),
               "watchdog heartbeat keeps continuous mode alive");
    check_true(!velocity_continuous_guard_poll(&guard, true, 201u) &&
                   (guard.fault_latched &
                    VELOCITY_CONTINUOUS_FAULT_WATCHDOG) != 0u &&
                   !guard.running,
               "communication timeout latches fault and stops continuous mode");
    check_true(!velocity_continuous_guard_start(&guard, 202u),
               "continuous mode cannot automatically restart after fault");
    check_true(velocity_continuous_guard_clear_fault(&guard, true) &&
                   velocity_continuous_guard_arm(&guard, true, 300u) &&
                   velocity_continuous_guard_start(&guard, 300u) &&
                   !velocity_continuous_guard_poll(&guard, false, 301u) &&
                   (guard.fault_latched &
                    VELOCITY_CONTINUOUS_FAULT_RUNTIME) != 0u,
               "runtime safety loss latches shutdown until a new explicit start");

    velocity_breakaway_probe_init(&breakaway, 0.030f, 5u, 1, 2, 2u, 4u);
    check_true(velocity_breakaway_probe_start(&breakaway, 1000, 0u) &&
                   nearf(velocity_breakaway_probe_iq_ref(&breakaway),
                         0.030f,
                         EPS),
               "breakaway probe starts once with fixed 30mA current");
    (void)velocity_breakaway_probe_update(&breakaway, 1001, 0u);
    (void)velocity_breakaway_probe_update(&breakaway, 1002, 0u);
    check_true(breakaway.result == VELOCITY_BREAKAWAY_ACTIVE &&
                   breakaway.motion_candidate,
               "two legal same-direction events create a motion candidate");
    check_true(velocity_breakaway_probe_update(&breakaway, 1002, 0u) ==
                   VELOCITY_BREAKAWAY_PASS &&
                   velocity_breakaway_probe_iq_ref(&breakaway) == 0.0f &&
                   !velocity_breakaway_probe_start(&breakaway, 1002, 0u),
               "confirmed motion withdraws breakaway current and cannot restart");

    velocity_breakaway_probe_init(&breakaway, 0.030f, 3u, 1, 2, 2u, 4u);
    (void)velocity_breakaway_probe_start(&breakaway, 2000, 0u);
    (void)velocity_breakaway_probe_update(&breakaway, 2000, 0u);
    (void)velocity_breakaway_probe_update(&breakaway, 2000, 0u);
    check_true(velocity_breakaway_probe_update(&breakaway, 2000, 0u) ==
                   VELOCITY_BREAKAWAY_FAIL_NO_MOTION &&
                   velocity_breakaway_probe_iq_ref(&breakaway) == 0.0f,
               "breakaway timeout fails safe without automatic escalation");

    velocity_breakaway_probe_init(&breakaway, 0.030f, 5u, 1, 2, 2u, 4u);
    (void)velocity_breakaway_probe_start(&breakaway, 3000, 0u);
    check_true(velocity_breakaway_probe_update(&breakaway, 2999, 0u) ==
                   VELOCITY_BREAKAWAY_FAIL_REVERSE,
               "reverse movement immediately fails the breakaway probe");

    velocity_breakaway_probe_init(&breakaway, 0.030f, 5u, 1, 2, 2u, 4u);
    (void)velocity_breakaway_probe_start(&breakaway, 4000, 10u);
    check_true(velocity_breakaway_probe_update(&breakaway, 4001, 11u) ==
                   VELOCITY_BREAKAWAY_FAIL_ENCODER,
               "illegal encoder transition immediately fails the probe");

    check_true(nearf(controller.current_limit_a, 0.030f, EPS),
               "breakaway diagnostic does not alter the authorized 30mA continuous limit");

    velocity_breakaway_probe_init(&breakaway,
                                  0.030f,
                                  1000u,
                                  1,
                                  4,
                                  4u,
                                  4u);
    check_true(velocity_breakaway_probe_start(&breakaway, 5000, 0u),
               "four-count breakaway confirmation starts once");
    check_true(velocity_breakaway_probe_update(&breakaway, 5001, 0u) ==
                   VELOCITY_BREAKAWAY_ACTIVE &&
                   velocity_breakaway_probe_update(&breakaway, 5002, 0u) ==
                   VELOCITY_BREAKAWAY_ACTIVE,
               "two encoder counts cannot prematurely confirm reliable breakaway");
    check_true(velocity_breakaway_probe_update(&breakaway, 5003, 0u) ==
                   VELOCITY_BREAKAWAY_ACTIVE &&
                   velocity_breakaway_probe_update(&breakaway, 5004, 0u) ==
                   VELOCITY_BREAKAWAY_ACTIVE &&
                   velocity_breakaway_probe_update(&breakaway, 5004, 0u) ==
                   VELOCITY_BREAKAWAY_PASS,
               "four legal same-direction counts confirm reliable breakaway on the next coherent sample");

    velocity_breakaway_probe_init(&breakaway,
                                  0.030f,
                                  1000u,
                                  1,
                                  16,
                                  16u,
                                  4u);
    check_true(velocity_breakaway_probe_start(&breakaway, 6000, 0u),
               "sixteen-count momentum confirmation starts once");
    for (int32_t count = 1; count <= 15; ++count) {
        check_true(velocity_breakaway_probe_update(&breakaway,
                                                   6000 + count,
                                                   0u) ==
                       VELOCITY_BREAKAWAY_ACTIVE,
                   "breakaway stays active before all sixteen legal events arrive");
    }
    check_true(velocity_breakaway_probe_update(&breakaway, 6016, 0u) ==
                   VELOCITY_BREAKAWAY_ACTIVE &&
                   velocity_breakaway_probe_update(&breakaway, 6016, 0u) ==
                       VELOCITY_BREAKAWAY_PASS,
               "sixteen legal counts confirm momentum before the bounded handoff");

    velocity_breakaway_handoff_init(&handoff, 0.030f, 0.030f);
    check_true(velocity_breakaway_handoff_start(&handoff) &&
                   nearf(velocity_breakaway_handoff_update(
                             &handoff,
                             VELOCITY_BREAKAWAY_ACTIVE,
                             0.030f,
                             100u),
                         0.030f,
                         EPS),
               "handoff starts with the independently bounded 30mA pulse");
    check_true(nearf(velocity_breakaway_handoff_update(
                         &handoff,
                         VELOCITY_BREAKAWAY_PASS,
                         0.030f,
                         101u),
                     0.030f,
                     EPS) &&
                   velocity_breakaway_handoff_speed_pi_active(&handoff) &&
                   handoff.handoff_control_tick == 101u &&
                   handoff.handoff_count == 1u,
               "confirmed motion hands off without a 30mA current step");
    check_true(nearf(velocity_breakaway_handoff_update(
                         &handoff,
                         VELOCITY_BREAKAWAY_PASS,
                         0.015f,
                         102u),
                     0.015f,
                     EPS) &&
                   nearf(velocity_breakaway_handoff_update(
                             &handoff,
                             VELOCITY_BREAKAWAY_PASS,
                             0.050f,
                             103u),
                         0.030f,
                         EPS),
               "post-handoff speed PI command is clamped to authorized 30mA");
    check_true(!velocity_breakaway_handoff_start(&handoff),
               "successful handoff cannot automatically restart breakaway");

    velocity_breakaway_handoff_init(&handoff, 0.030f, 0.030f);
    (void)velocity_breakaway_handoff_start(&handoff);
    check_true(nearf(velocity_breakaway_handoff_update(
                         &handoff,
                         VELOCITY_BREAKAWAY_FAIL_NO_MOTION,
                         0.030f,
                         200u),
                     0.0f,
                     EPS) &&
                   handoff.state == VELOCITY_BREAKAWAY_HANDOFF_FAILED,
               "failed breakaway never hands control to speed PI");
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

static void fast_tick(FixedRotorCurrentTest *test,
                      uint32_t seq,
                      FixedRotorCurrentTestOutput *out)
{
    FixedRotorCurrentTestInput in = nominal_input(seq);
    fixed_rotor_current_test_fast_isr(test, &in, out);
}

static void test_fixed_rotor_protections(void)
{
    FixedRotorCurrentTest test;
    FixedRotorCurrentTestConfig cfg = fixed_rotor_current_test_default_config();
    FixedRotorCurrentTestOutput out;
    FixedRotorCurrentTestInput in;

    printf("\n== fixed rotor protections ==\n");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    in.iv_a = 0.30f;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.result == FIXED_ROTOR_RESULT_FAIL) &&
                   ((out.fault_code & FIXED_ROTOR_FAULT_PHASE_CURRENT_LIMIT) != 0u),
               "phase current test limit abort");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    in = nominal_input(2u);
    in.encoder_count += 33;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ROTOR_MOVED) != 0u,
               "encoder movement >32 abort");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    in = nominal_input(3u);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ADC_SEQ_GAP) != 0u,
               "ADC seq gap abort");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_ADC_DUPLICATE) != 0u,
               "duplicate tick no repeated integration");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    in.iv_a = NAN;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_NAN_INF) != 0u,
               "NaN/Inf safe fail");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    in.m1_safe = false;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
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

static void test_fixed_rotor_fast_isr_contract(void)
{
    FixedRotorCurrentTest test;
    FixedRotorCurrentTestConfig cfg = fixed_rotor_current_test_default_config();
    FixedRotorCurrentTestOutput out = {0};
    FixedRotorCurrentTestInput in;

    printf("\n== fixed rotor fast ISR contract ==\n");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_service_main(&test, &out);
    check_true(test.control_tick_seq == 0u &&
                   test.voltage_command_seq == 0u &&
                   !out.power_stage_request,
               "main-loop service does not update PI");

    fixed_rotor_current_test_request_start(&test);
    check_true(test.state == FIXED_ROTOR_STATE_ENABLE_ZERO &&
                   test.control_tick_seq == 0u &&
                   nearf(test.controller.integrator_d_v, 0.0f, EPS),
               "run_request arms ENABLE_ZERO and clears integrator");

    fast_tick(&test, 1u, &out);
    check_true(test.control_tick_seq == 1u, "one adc_seq produces one control tick");
    check_true(test.voltage_command_seq == 1u, "valid fast tick produces one voltage command");
    const float integrator_after_first = test.controller.integrator_d_v;
    in = nominal_input(1u);
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((test.fault_code & FIXED_ROTOR_FAULT_ADC_DUPLICATE) != 0u,
               "duplicate snapshot faults in fast path");
    check_true(nearf(test.controller.integrator_d_v, 0.0f, EPS) ||
                   nearf(test.controller.integrator_d_v, integrator_after_first, EPS),
               "duplicate snapshot does not keep integrating");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    fast_tick(&test, 1u, &out);
    in = nominal_input(2u);
    in.nfault_ok = false;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_NFAULT) != 0u &&
                   out.safe_shutdown_request,
               "nFAULT faults immediately in fast path");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    in = nominal_input(1u);
    in.drv_ok = false;
    fixed_rotor_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & FIXED_ROTOR_FAULT_DRV) != 0u,
               "slow DRV status is consumed as fast-path fault input");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    for (uint32_t seq = 1u; seq <= cfg.enable_zero_ticks; ++seq) {
        fast_tick(&test, seq, &out);
    }
    check_true(test.state == FIXED_ROTOR_STATE_RAMP_ID_0P05,
               "ENABLE_ZERO duration is based on 20kHz tick count");

    fixed_rotor_current_test_init(&test, &cfg);
    fixed_rotor_current_test_request_start(&test);
    test.config.log_decimation = 1u;
    test.config.tracking_error_limit_ticks = 1000000u;
    test.config.saturation_limit_ticks = 1000000u;
    for (uint32_t seq = 1u; seq <= (FIXED_ROTOR_CURRENT_TEST_LOG_CAPACITY + 10u); ++seq) {
        fast_tick(&test, seq, &out);
    }
    check_true(test.log_count == FIXED_ROTOR_CURRENT_TEST_LOG_CAPACITY &&
                   test.log_dropped > 0u,
               "ISR log buffer saturates without overflow");
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
    fixed_rotor_current_test_request_start(&test);
    for (uint32_t seq = 1u; seq < 160000u && !out.done; ++seq) {
        in = nominal_input(seq);
        fixed_rotor_current_test_fast_isr(&test, &in, &out);
    }
    check_true(out.result == FIXED_ROTOR_RESULT_PASS, "sequence reaches COMPLETE");
    check_true(out.safe_shutdown_request, "safe shutdown requested on COMPLETE");
    check_true(test.hold_0p05_stats.sample_count > 0u &&
                   test.hold_0p10_stats.sample_count > 0u,
               "hold stage stats collected");
    check_true(test.voltage_command_seq > 0u &&
                   test.voltage_command_seq <= test.control_tick_seq,
               "voltage command sequence tracks accepted control ticks");

    fixed_rotor_current_test_request_start(&test);
    check_true(test.result == FIXED_ROTOR_RESULT_PASS &&
                   test.state == FIXED_ROTOR_STATE_COMPLETE,
               "completion does not auto-restart");
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

static RotatingDqCurrentTestInput rotating_nominal_input(uint32_t seq,
                                                        int64_t encoder_count)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTestInput in;
    memset(&in, 0, sizeof(in));
    in.time_us = (uint64_t)seq * 50ull;
    in.adc_seq = seq;
    in.encoder_count = encoder_count;
    in.theta_e_rad =
        rotating_dq_current_test_theta_from_count(&cfg, encoder_count, 1.0f);
    in.iv_a = 0.0f;
    in.iw_a = 0.0f;
    in.vbus_v = 11.7f;
    in.adc_valid = true;
    in.encoder_valid = true;
    in.theta_valid = true;
    in.electrical_offset_valid = true;
    in.nfault_ok = true;
    in.drv_ok = true;
    in.m1_safe = true;
    in.pwm_ccr_ok = true;
    in.pwm_allowed = true;
    in.fault_active = false;
    in.raw_pc0 = 1000u;
    in.raw_pc1 = 1000u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.020142f;
    in.ccr1 = 2100u;
    in.ccr2 = 2100u;
    in.ccr3 = 2100u;
    in.ccr4 = 3799u;
    in.ccer = 0x00001555u;
    in.bdtr = 0x00008000u;
    in.tim1_cnt = 3000u;
    in.adc_rank_order = 0u;
    in.callback_cycles = 1000u;
    in.electrical_offset_rad = 1.0f;
    return in;
}

static void rotating_tick(RotatingDqCurrentTest *test,
                          uint32_t seq,
                          int64_t encoder_count,
                          RotatingDqCurrentTestOutput *out)
{
    RotatingDqCurrentTestInput in = rotating_nominal_input(seq, encoder_count);
    rotating_dq_current_test_fast_isr(test, &in, out);
}

static void rotating_tick_with_iq(RotatingDqCurrentTest *test,
                                  uint32_t seq,
                                  int64_t encoder_count,
                                  float iq_a,
                                  RotatingDqCurrentTestOutput *out)
{
    RotatingDqCurrentTestInput in = rotating_nominal_input(seq, encoder_count);
    const float half_sqrt3_iq = 0.86602540378f * iq_a;
    in.theta_e_rad = 0.0f;
    in.iv_a = half_sqrt3_iq;
    in.iw_a = -half_sqrt3_iq;
    rotating_dq_current_test_fast_isr(test, &in, out);
}

static void test_rotating_offset_and_angle_guards(void)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTest test;
    RotatingDqCurrentTestOutput out;
    RotatingDqCurrentTestInput in;
    RotatingDqOffsetAdmission admission;

    printf("\n== rotating dq offset and angle guards ==\n");

    admission.sample_count = 128u;
    admission.iv_mean_a = 13.0f * 0.020142f;
    admission.iw_mean_a = 0.0f;
    admission.iu_mean_a = -admission.iv_mean_a;
    admission.id_mean_a = 0.0f;
    admission.iq_mean_a = 0.0f;
    admission.iv_std_a = 0.0f;
    admission.iw_std_a = 0.0f;
    admission.iu_std_a = 0.0f;
    admission.id_std_a = 0.0f;
    admission.iq_std_a = 0.0f;
    admission.phase_current_peak_a = 0.07f;
    admission.adc_valid = true;
    admission.nfault_ok = true;
    admission.fault_code = 0u;
    check_true(!rotating_dq_current_test_offset_admission_ok(&cfg, &admission),
               "offset mismatch of 13 counts is rejected");

    admission.iv_mean_a = 0.005f;
    admission.iw_mean_a = -0.006f;
    admission.iu_mean_a = 0.001f;
    admission.id_mean_a = 0.004f;
    admission.iq_mean_a = -0.003f;
    admission.phase_current_peak_a = 0.161f;
    check_true(rotating_dq_current_test_offset_admission_ok(&cfg, &admission),
               "quiet zero-current offset admission tolerates isolated below-limit peak");

    admission.phase_current_peak_a = 0.222f;
    check_true(rotating_dq_current_test_offset_admission_ok(&cfg, &admission),
               "pre-power offset admission ignores isolated peak and relies on runtime fast protection");
    admission.phase_current_peak_a = 0.07f;

    cfg.enable_zero_diagnostic_only = false;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, false);
    rotating_dq_current_test_service_main(&test, &out);
    check_true((out.result == ROTATING_DQ_RESULT_FAIL) &&
                   ((out.fault_code &
                     ROTATING_DQ_FAULT_ELECTRICAL_OFFSET_INVALID) != 0u) &&
                   !out.power_stage_request,
               "no reliable electrical offset blocks rotation and MOE request");

    cfg.enable_zero_diagnostic_only = false;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 0);
    in.electrical_offset_valid = false;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code &
                ROTATING_DQ_FAULT_ELECTRICAL_OFFSET_INVALID) != 0u,
               "fast path rejects invalid electrical offset");

    check_true(rotating_dq_current_test_encoder_delta_u16(0u, 65535u) == 1,
               "encoder uint16 wrap positive delta");
    check_true(rotating_dq_current_test_encoder_delta_u16(65535u, 0u) == -1,
               "encoder uint16 wrap negative delta");

    const float theta0 = rotating_dq_current_test_theta_from_count(&cfg, 0, 0.25f);
    const float theta1 = rotating_dq_current_test_theta_from_count(&cfg, 4096 / 7, 0.25f);
    check_true(nearf(theta0, theta1, 0.02f),
               "electrical angle advances by pole-pair relationship");
}

static void test_rotating_fast_path_and_limits(void)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTest test;
    RotatingDqCurrentTestOutput out = {0};
    RotatingDqCurrentTestInput in;

    printf("\n== rotating dq fast path and limits ==\n");
    cfg.enable_zero_ticks = 4u;
    cfg.iq_hold_ticks = 3u;
    cfg.hold_zero_ticks = 2u;
    cfg.tracking_error_limit_ticks = 100000u;
    cfg.saturation_limit_ticks = 100000u;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_service_main(&test, &out);
    check_true(test.control_tick_seq == 0u &&
                   test.voltage_command_seq == 0u &&
                   !out.power_stage_request,
               "rotating main service does not run PI");

    rotating_dq_current_test_request_start(&test, true);
    rotating_tick(&test, 1u, 1000, &out);
    check_true(test.control_tick_seq == 1u &&
                   test.voltage_command_seq == 1u &&
                   nearf(out.id_ref_a, 0.0f, EPS),
               "one adc_seq produces one rotating control tick and id_ref zero");

    in = rotating_nominal_input(1u, 1000);
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_ADC_DUPLICATE) != 0u,
               "rotating duplicate snapshot faults");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    rotating_tick(&test, 1u, 1000, &out);
    rotating_tick(&test, 3u, 1000, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_ADC_SEQ_GAP) != 0u,
               "rotating adc seq gap faults");

    cfg.enable_zero_diagnostic_only = false;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq <= ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++seq) {
        in = rotating_nominal_input(seq, 1000);
        in.iv_a = 0.21f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true((out.fault_code & ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT) != 0u &&
                   test.zero_phase_over_limit_consecutive ==
                       ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS,
               "rotating ENABLE_ZERO phase current limit faults after consecutive samples");

    cfg.enable_zero_ticks = 200u;
    cfg.enable_zero_diagnostic_only = true;
    cfg.enable_zero_diagnostic_ticks = 200u;
    /* These cases exercise the 128-sample classification rules directly. */
    cfg.zero_startup_guard_ticks = 0u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.theta_e_rad = 0.0f;
    in.iv_a = 0.03f;
    in.iw_a = 0.03f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) == 0u &&
                   test.state == ROTATING_DQ_STATE_ENABLE_ZERO,
               "rotating ENABLE_ZERO waits for enough samples before offset judgment");
    for (uint32_t seq = 2u; seq <= 128u; ++seq) {
        in = rotating_nominal_input(seq, 1000);
        in.theta_e_rad = 0.0f;
        in.iv_a = 0.03f;
        in.iw_a = 0.03f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true((out.fault_code & ROTATING_DQ_FAULT_ZERO_CURRENT_INVALID) != 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) == 0u),
               "rotating diagnostic ENABLE_ZERO persistent non-common-mode current uses zero-current fault");

    {
        RotatingDqCurrentTest isolated_test;
        RotatingDqCurrentTestOutput isolated_out = {0};
        rotating_dq_current_test_init(&isolated_test, &cfg);
        rotating_dq_current_test_request_start(&isolated_test, true);
        for (uint32_t seq = 1u; seq <= 128u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            if (seq == 99u) {
                /* Reproduce the hardware -9/-1 count isolated excursion. */
                in.raw_pc0 = 991u;
                in.raw_pc1 = 999u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = -0.181278f;
                in.iw_a = -0.020142f;
            }
            rotating_dq_current_test_fast_isr(&isolated_test, &in,
                                               &isolated_out);
        }
        check_true(isolated_test.zero_diag_completed &&
                       isolated_test.zero_clean_phase_metric_max_a >
                           isolated_test.config.zero_phase_current_peak_limit_a &&
                       isolated_test.zero_phase_over_limit_consecutive_max == 1u &&
                       isolated_out.fault_code == ROTATING_DQ_FAULT_NONE,
                   "isolated reconstructed peak does not fail zero-current window summary");

        rotating_dq_current_test_init(&isolated_test, &cfg);
        rotating_dq_current_test_request_start(&isolated_test, true);
        memset(&isolated_out, 0, sizeof(isolated_out));
        for (uint32_t seq = 1u; seq <= 128u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            if (seq == 83u) {
                /* Reproduce the hardware +10/-3 count direct-channel peak. */
                in.raw_pc0 = 1010u;
                in.raw_pc1 = 997u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = 0.20142f;
                in.iw_a = -0.060426f;
            }
            rotating_dq_current_test_fast_isr(&isolated_test, &in,
                                               &isolated_out);
        }
        check_true(isolated_test.zero_diag_completed &&
                       isolated_test.zero_measured_phase_metric_max_a >
                           isolated_test.config.zero_phase_current_peak_limit_a &&
                       isolated_test.zero_phase_over_limit_consecutive_max == 1u &&
                       isolated_out.fault_code == ROTATING_DQ_FAULT_NONE,
                   "isolated direct ADC peak does not fail offset window summary");
    }

    {
        RotatingDqCurrentTestConfig cm_window_cfg = cfg;
        RotatingDqCurrentTest cm_window_test;
        RotatingDqCurrentTestOutput cm_window_out = {0};
        cm_window_cfg.enable_zero_diagnostic_only = true;
        cm_window_cfg.enable_zero_diagnostic_ticks = 200u;
        cm_window_cfg.enable_zero_ticks = 200u;
        rotating_dq_current_test_init(&cm_window_test, &cm_window_cfg);
        rotating_dq_current_test_request_start(&cm_window_test, true);
        for (uint32_t seq = 1u; seq <= 128u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            if (seq == 1u) {
                in.raw_pc0 = 1001u;
                in.raw_pc1 = 1001u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = 0.020142f;
                in.iw_a = 0.020142f;
            } else if ((seq % 7u) == 0u && seq <= 119u) {
                in.raw_pc0 = 1005u;
                in.raw_pc1 = 1005u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = 0.101f;
                in.iw_a = 0.101f;
            }
            rotating_dq_current_test_fast_isr(&cm_window_test, &in,
                                               &cm_window_out);
        }
        check_true(cm_window_test.result == ROTATING_DQ_RESULT_RUNNING &&
                       cm_window_test.zero_common_mode_shift_count == 18u &&
                       cm_window_test.zero_common_mode_harmless_count == 1u &&
                       cm_window_test.zero_common_mode_excluded_count == 17u &&
                       cm_window_test.zero_clean_sample_count == 111u &&
                       (cm_window_out.fault_code == ROTATING_DQ_FAULT_NONE),
                   "harmless common-mode shapes stay clean while causal reconstructed spikes are excluded");

        rotating_dq_current_test_init(&cm_window_test, &cm_window_cfg);
        rotating_dq_current_test_request_start(&cm_window_test, true);
        memset(&cm_window_out, 0, sizeof(cm_window_out));
        for (uint32_t seq = 1u; seq <= 128u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            if ((seq % 3u) == 0u && seq <= 99u) {
                in.raw_pc0 = 1005u;
                in.raw_pc1 = 1005u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = 0.101f;
                in.iw_a = 0.101f;
            }
            rotating_dq_current_test_fast_isr(&cm_window_test, &in,
                                               &cm_window_out);
        }
        check_true((cm_window_out.fault_code &
                    ROTATING_DQ_FAULT_CURRENT_SENSE_COMMON_MODE_EXCESS) != 0u &&
                       ((cm_window_out.fault_code &
                         ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) == 0u),
                   "excess common-mode exclusions use an independent diagnostic fault");

        cm_window_cfg.phase_current_limit_a = 0.30f;
        cm_window_cfg.dq_current_limit_a = 0.30f;
        rotating_dq_current_test_init(&cm_window_test, &cm_window_cfg);
        rotating_dq_current_test_request_start(&cm_window_test, true);
        memset(&cm_window_out, 0, sizeof(cm_window_out));
        for (uint32_t seq = 1u; seq <= 128u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            in.iv_a = 0.21f;
            in.iw_a = 0.0f;
            rotating_dq_current_test_fast_isr(&cm_window_test, &in,
                                               &cm_window_out);
        }
        check_true((cm_window_out.fault_code &
                    ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) != 0u,
                   "direct ADC phase offset still faults current-offset admission");
    }
    cfg.enable_zero_diagnostic_only = false;
    cfg.enable_zero_ticks = 4u;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.iv_a = 0.05f;
    in.iw_a = -0.03f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.state == ROTATING_DQ_STATE_ENABLE_ZERO &&
                   fabsf(out.vd_v) < 0.000001f &&
                   fabsf(out.vq_v) < 0.000001f &&
                   test.tracking_error_ticks == 0u,
               "normal ENABLE_ZERO holds zero vector and does not track noise");

    cfg.enable_zero_current_pi = true;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.iv_a = 0.02f;
    in.iw_a = -0.01f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.state == ROTATING_DQ_STATE_ENABLE_ZERO &&
                   out.power_stage_request && out.pwm_output_request &&
                   (fabsf(out.vd_v) > 0.000001f ||
                    fabsf(out.vq_v) > 0.000001f),
               "ENABLE_ZERO current-PI diagnostic closes the inner loop at zero reference");
    cfg.enable_zero_current_pi = false;

    test.state = ROTATING_DQ_STATE_RAMP_ZERO_1;
    test.result = ROTATING_DQ_RESULT_RUNNING;
    test.iq_ref_a = 0.018f;
    in = rotating_nominal_input(2u, 1000);
    in.iv_a = 0.08f;
    in.iw_a = -0.02f;
    memset(&out, 0, sizeof(out));
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.state == ROTATING_DQ_STATE_RAMP_ZERO_1 &&
                   fabsf(out.vd_v) < 0.000001f &&
                   fabsf(out.vq_v) < 0.000001f &&
                   test.tracking_error_ticks == 0u,
               "RAMP_ZERO holds zero vector and does not track noise");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.nfault_ok = false;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_NFAULT) != 0u &&
                   out.safe_shutdown_request,
               "rotating nFAULT fast shutdown");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    rotating_tick(&test, 1u, 1000, &out);
    in = rotating_nominal_input(2u, 1000);
    in.theta_e_rad += 1.0f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_ANGLE_JUMP) != 0u,
               "rotating angle jump faults");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    rotating_tick(&test, 1u, 0, &out);
    rotating_tick(&test, 2u, 20, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_OVERSPEED) != 0u,
               "rotating overspeed faults");

    cfg.speed_limit_rpm = 100000.0f;
    cfg.angle_jump_limit_rad = 10.0f;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    rotating_tick(&test, 1u, 0, &out);
    rotating_tick(&test, 2u, cfg.zero_encoder_limit_counts + 1, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_ENCODER) != 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) == 0u),
               "rotating ENABLE_ZERO encoder motion uses encoder fault");
    cfg.speed_limit_rpm = 100.0f;
    cfg.angle_jump_limit_rad = 0.25f;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    rotating_tick(&test, 1u, 0, &out);
    in = rotating_nominal_input(2u, 4097);
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true((out.fault_code & ROTATING_DQ_FAULT_ONE_REV) != 0u,
               "rotating one mechanical revolution limit faults");
}

static void test_rotating_enable_zero_diagnostic_mode(void)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTest test;
    RotatingDqCurrentTestOutput out = {0};
    RotatingDqCurrentTestInput in;

    printf("\n== rotating dq ENABLE_ZERO diagnostic mode ==\n");

    cfg.enable_zero_diagnostic_only = true;
    cfg.enable_zero_diagnostic_ticks = 4u;
    cfg.enable_zero_ticks = 100u;
    cfg.log_decimation = 1u;
    cfg.tracking_error_limit_ticks = 100000u;
    cfg.saturation_limit_ticks = 100000u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    for (uint32_t seq = 1u; seq <= 8u && !out.done; ++seq) {
        rotating_tick(&test, seq, 1000, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS &&
                   test.state == ROTATING_DQ_STATE_COMPLETE &&
                   test.zero_diag_completed &&
                   test.positive_stats.sample_count == 0u &&
                   test.negative_stats.sample_count == 0u,
               "diagnostic-only mode completes ENABLE_ZERO without entering +/-iq");
    check_true(strcmp(rotating_dq_zero_diag_classification(&test),
                      "ENABLE_ZERO_DATA_VALID") == 0,
               "diagnostic-only clean completion is classified as valid data");
    check_true(test.zero_diag_count == 0u,
               "short diagnostic-only window skips per-tick fast-loop samples");

    cfg.enable_zero_diagnostic_ticks = 12u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq <= 16u && !out.done; ++seq) {
        rotating_tick(&test, seq, 1000, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS &&
                   test.zero_diag_completed &&
                   test.zero_diag_count == 0u,
               "long diagnostic-only window stays lightweight without per-tick samples");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.raw_pc0 = 1010u;
    in.raw_pc1 = 990u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.20f;
    in.iw_a = -0.20f;
    in.theta_e_rad = 0.0f;
    rotating_dq_current_test_capture_zero_diag_sample(
        &test, &in, ROTATING_DQ_ZERO_STAGE_BASELINE_MOE_OFF, false, 123u);
    check_true(test.zero_diag_count == 1u &&
                   test.zero_diag[0].delta_pc0 == 10 &&
                   test.zero_diag[0].delta_pc1 == -10 &&
                   nearf(test.zero_diag[0].iv_a, 0.20f, 0.0001f) &&
                   nearf(test.zero_diag[0].iw_a, -0.20f, 0.0001f) &&
                   nearf(test.zero_diag[0].iu_a, 0.0f, 0.0001f) &&
                   nearf(test.zero_diag[0].i_alpha_a, 0.0f, 0.0001f),
               "zero diagnostic reconstructs PC0/PC1 as V/W and iu=-(iv+iw)");

    cfg.enable_zero_diagnostic_only = false;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq <= ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++seq) {
        in = rotating_nominal_input(seq, 1000);
        in.theta_e_rad = 0.0f;
        in.raw_pc0 = 1000u;
        in.raw_pc1 = 1000u;
        in.offset_pc0 = 1000u;
        in.offset_pc1 = 1000u;
        in.current_amp_per_count = 0.02f;
        in.iv_a = 0.22f;
        in.iw_a = 0.04f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true(test.zero_first_trip.valid &&
                   ((out.fault_code & ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT) != 0u) &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u) &&
                   ((test.zero_first_trip.source_mask &
                     ROTATING_DQ_ZERO_TRIP_PHASE_METRIC) != 0u) &&
                   ((test.zero_first_trip.source_mask &
                      ROTATING_DQ_ZERO_TRIP_DQ_METRIC) != 0u),
               "same ENABLE_ZERO sequence can latch both phase and dq current faults");
    check_true(strcmp(rotating_dq_zero_diag_classification(&test),
                      "LIVE_ZERO_OFFSET_MISMATCH") == 0,
               "neutral zero-voltage phase+dq trip is classified as live zero offset mismatch");

    cfg.enable_zero_diagnostic_only = true;
    cfg.phase_current_limit_a = 0.20f;
    cfg.dq_current_limit_a = 0.15f;
    cfg.enable_zero_diagnostic_ticks = 40u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.theta_e_rad = rotating_dq_current_test_theta_from_count(&cfg, 1000, 1.0f);
    in.raw_pc0 = 1011u;
    in.raw_pc1 = 1002u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.18f;
    in.iw_a = 0.04f;
    memset(&out, 0, sizeof(out));
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.result == ROTATING_DQ_RESULT_RUNNING &&
                   test.zero_first_trip.valid &&
                   test.zero_phase_startup_over_limit_count == 1u &&
                   test.zero_phase_startup_over_limit_last_tick == 1u &&
                   test.zero_dq_startup_over_limit_count == 1u &&
                   test.zero_dq_startup_over_limit_last_tick == 1u &&
                   test.zero_phase_fault_set_tick == 0u &&
                   test.zero_dq_fault_set_tick == 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT) == 0u) &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u),
               "diagnostic-only startup window observes phase+dq soft over-limit without immediate fault");

    for (uint32_t seq = 2u; seq <= cfg.enable_zero_diagnostic_ticks + 8u && !out.done; ++seq) {
        rotating_tick(&test, seq, 1000, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS && test.zero_diag_completed,
               "startup phase+dq soft over-limit can complete when later samples are clean");
    check_true(strcmp(rotating_dq_zero_diag_classification(&test),
                      "STARTUP_CURRENT_TRANSIENT_OBSERVED") == 0,
               "completed startup phase+dq soft over-limit is classified as observed transient");

    {
        RotatingDqCurrentTestConfig guard_cfg =
            rotating_dq_current_test_default_config();
        RotatingDqCurrentTest guard_test;
        RotatingDqCurrentTestOutput guard_out = {0};

        guard_cfg.enable_zero_diagnostic_only = true;
        guard_cfg.enable_zero_diagnostic_ticks = 144u;
        guard_cfg.enable_zero_ticks = 200u;
        guard_cfg.tracking_error_limit_ticks = 100000u;
        guard_cfg.saturation_limit_ticks = 100000u;
        rotating_dq_current_test_init(&guard_test, &guard_cfg);
        rotating_dq_current_test_request_start(&guard_test, true);
        for (uint32_t seq = 1u; seq <= 144u && !guard_out.done; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            if (seq == 1u) {
                in.raw_pc0 = 999u;
                in.raw_pc1 = 947u;
                in.offset_pc0 = 1000u;
                in.offset_pc1 = 1000u;
                in.current_amp_per_count = 0.020142f;
                in.iv_a = -0.020142f;
                in.iw_a = -1.067526f;
            }
            rotating_dq_current_test_fast_isr(&guard_test, &in, &guard_out);
        }
        check_true(guard_out.result == ROTATING_DQ_RESULT_PASS &&
                       guard_test.zero_clean_sample_count == 128u &&
                       guard_test.zero_startup_direct_outlier_count == 1u &&
                       guard_test.zero_startup_first_outlier_tick == 1u &&
                       guard_test.zero_startup_last_outlier_tick == 1u &&
                       guard_test.zero_startup_pc0_peak_delta_counts == -1 &&
                       guard_test.zero_startup_pc1_peak_delta_counts == -53 &&
                       guard_test.zero_measured_phase_metric_max_a < 0.01f &&
                       ((guard_out.fault_code &
                         ROTATING_DQ_FAULT_CURRENT_OFFSET_INVALID) == 0u),
                   "startup-only PC1 outlier is observed but excluded from the full steady zero-current window");
    }

    cfg.phase_current_limit_a = 10.0f;
    cfg.dq_current_limit_a = 0.15f;
    cfg.enable_zero_diagnostic_ticks = 40u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq <= ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS; ++seq) {
        rotating_tick(&test, seq, 1000, &out);
    }
    in = rotating_nominal_input(ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 1u, 1000);
    in.theta_e_rad = rotating_dq_current_test_theta_from_count(&cfg, 1000, 1.0f);
    in.raw_pc0 = 1005u;
    in.raw_pc1 = 1005u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.12f;
    in.iw_a = 0.12f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.result == ROTATING_DQ_RESULT_RUNNING &&
                   test.zero_common_mode_shift_count == 1u &&
                   test.zero_common_mode_max_counts == 10u &&
                   test.zero_common_mode_diff_max_counts == 0u &&
                   test.zero_dq_over_limit_consecutive == 0u &&
                   test.zero_dq_over_limit_consecutive_max == 0u &&
                   test.zero_dq_fault_set_tick == 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u),
               "causal common-mode post-startup dq over-limit holds the ENABLE_ZERO counter");

    in = rotating_nominal_input(ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 2u, 1000);
    in.theta_e_rad = rotating_dq_current_test_theta_from_count(&cfg, 1000, 1.0f);
    in.raw_pc0 = 1007u;
    in.raw_pc1 = 1003u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.14f;
    in.iw_a = 0.06f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.result == ROTATING_DQ_RESULT_RUNNING &&
                   test.zero_common_mode_shift_count == 2u &&
                   test.zero_common_mode_max_counts == 10u &&
                   test.zero_common_mode_diff_max_counts == 4u &&
                   test.zero_dq_fault_set_tick == 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u),
               "asymmetric small same-sign ENABLE_ZERO shift is recorded without faulting");

    {
        RotatingDqCurrentTestConfig cf_cfg =
            rotating_dq_current_test_default_config();
        RotatingDqCurrentTest cf_test;
        RotatingDqCurrentTestOutput cf_out = {0};

        cf_cfg.enable_zero_diagnostic_only = true;
        cf_cfg.enable_zero_current_pi = true;
        cf_cfg.zero_startup_guard_ticks = 0u;
        cf_cfg.enable_zero_diagnostic_ticks = 100u;
        cf_cfg.enable_zero_ticks = 100u;
        cf_cfg.tracking_error_limit_ticks = 100000u;
        cf_cfg.saturation_limit_ticks = 100000u;
        rotating_dq_current_test_init(&cf_test, &cf_cfg);
        rotating_dq_current_test_request_start(&cf_test, true);
        for (uint32_t seq = 1u; seq <= 4u; ++seq) {
            in = rotating_nominal_input(seq, 1000);
            in.theta_e_rad = 0.0f;
            in.raw_pc0 = 1005u;
            in.raw_pc1 = 1005u;
            in.offset_pc0 = 1000u;
            in.offset_pc1 = 1000u;
            in.current_amp_per_count = 0.020142f;
            in.iv_a = 0.101f;
            in.iw_a = 0.101f;
            rotating_dq_current_test_fast_isr(&cf_test, &in, &cf_out);
        }
        check_true(cf_test.result == ROTATING_DQ_RESULT_RUNNING &&
                       cf_test.zero_phase_over_limit_consecutive == 0u &&
                       cf_test.zero_dq_over_limit_consecutive == 0u &&
                       nearf(cf_test.controller.integrator_d_v, 0.0f, EPS) &&
                       nearf(cf_test.controller.integrator_q_v, 0.0f, EPS) &&
                       nearf(cf_out.vd_v, 0.0f, EPS) &&
                       nearf(cf_out.vq_v, 0.0f, EPS),
                   "zero-reference causal common-mode samples hold soft counters and do not drive PI");
        check_true(cf_out.common_mode_shape &&
                       cf_out.common_mode_harmful &&
                       cf_out.common_mode_caused_dq_crossing &&
                       fmaxf(fabsf(cf_out.id_measured_a),
                             fabsf(cf_out.iq_measured_a)) >
                           cf_cfg.dq_current_limit_a &&
                       fabsf(cf_out.id_control_a) < EPS &&
                       fabsf(cf_out.iq_control_a) < EPS &&
                       nearf(cf_out.integrator_q_delta_v, 0.0f, EPS) &&
                       nearf(cf_out.integrator_q_aw_clamp_delta_v, 0.0f, EPS),
                   "common-mode shadow output separates raw DQ, controller DQ, and integrator attribution");
    }

    {
        RotatingDqCurrentTestConfig cm_cfg = cfg;
        RotatingDqCurrentTest cm_test;
        RotatingDqCurrentTestOutput cm_out = {0};
        RotatingDqCurrentTestInput cm_in;
        cm_cfg.enable_zero_diagnostic_only = false;
        cm_cfg.phase_current_limit_a = 0.20f;
        cm_cfg.dq_current_limit_a = 0.15f;
        rotating_dq_current_test_init(&cm_test, &cm_cfg);
        rotating_dq_current_test_request_start(&cm_test, true);
        cm_test.state = ROTATING_DQ_STATE_RAMP_ZERO_2;
        cm_test.result = ROTATING_DQ_RESULT_RUNNING;
        cm_test.iq_ref_a = -0.018f;
        for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
            cm_in = rotating_nominal_input(100u + i, 1000);
            cm_in.theta_e_rad = 4.082f;
            cm_in.raw_pc0 = 991u;
            cm_in.raw_pc1 = 994u;
            cm_in.offset_pc0 = 1000u;
            cm_in.offset_pc1 = 1000u;
            cm_in.current_amp_per_count = 0.020142f;
            cm_in.iv_a = -0.181f;
            cm_in.iw_a = -0.121f;
            rotating_dq_current_test_fast_isr(&cm_test, &cm_in, &cm_out);
        }
        check_true(cm_test.result == ROTATING_DQ_RESULT_RUNNING &&
                       cm_test.zero_phase_over_limit_consecutive == 0u &&
                       cm_test.zero_dq_over_limit_consecutive == 0u &&
                       ((cm_out.fault_code & ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT) == 0u) &&
                       ((cm_out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u),
                   "near-zero ramp common-mode current reconstruction spike does not fault");
        check_true(cm_test.result == ROTATING_DQ_RESULT_RUNNING,
                   "zero-return common-mode suppression only affects soft faulting");

        rotating_dq_current_test_init(&cm_test, &cm_cfg);
        rotating_dq_current_test_request_start(&cm_test, true);
        cm_test.state = ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE;
        cm_test.result = ROTATING_DQ_RESULT_RUNNING;
        cm_test.iq_ref_a = -0.018f;
        memset(&cm_out, 0, sizeof(cm_out));
        for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
            cm_in = rotating_nominal_input(150u + i, 1000);
            cm_in.theta_e_rad = 4.082f;
            cm_in.raw_pc0 = 991u;
            cm_in.raw_pc1 = 994u;
            cm_in.offset_pc0 = 1000u;
            cm_in.offset_pc1 = 1000u;
            cm_in.current_amp_per_count = 0.020142f;
            cm_in.iv_a = -0.181f;
            cm_in.iw_a = -0.121f;
            rotating_dq_current_test_fast_isr(&cm_test, &cm_in, &cm_out);
        }
        check_true(((cm_out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u) &&
                       cm_test.zero_dq_over_limit_consecutive ==
                           ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS,
                   "nonzero iq ramp does not suppress common-mode-like over-limit");

        rotating_dq_current_test_init(&cm_test, &cm_cfg);
        rotating_dq_current_test_request_start(&cm_test, true);
        cm_test.state = ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE;
        cm_test.result = ROTATING_DQ_RESULT_RUNNING;
        cm_test.iq_ref_a = -0.020f;
        memset(&cm_out, 0, sizeof(cm_out));
        for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
            cm_in = rotating_nominal_input(200u + i, 1000);
            cm_in.theta_e_rad = 4.082f;
            cm_in.raw_pc0 = 991u;
            cm_in.raw_pc1 = 994u;
            cm_in.offset_pc0 = 1000u;
            cm_in.offset_pc1 = 1000u;
            cm_in.current_amp_per_count = 0.020142f;
            cm_in.iv_a = -0.181f;
            cm_in.iw_a = -0.121f;
            rotating_dq_current_test_fast_isr(&cm_test, &cm_in, &cm_out);
        }
        check_true(((cm_out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u) &&
                       cm_test.zero_dq_over_limit_consecutive ==
                           ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS,
                   "nonzero iq hold does not suppress common-mode-like over-limit");
    }

    {
        RotatingDqCurrentTestConfig frozen_cfg =
            rotating_dq_current_test_default_config();
        RotatingDqCurrentTest frozen_test;
        RotatingDqCurrentTestOutput frozen_out = {0};
        RotatingDqCurrentTestInput frozen_in;

        frozen_cfg.enable_zero_diagnostic_only = true;
        frozen_cfg.enable_zero_current_pi = true;
        frozen_cfg.freeze_zero_reference_integrator = true;
        frozen_cfg.zero_startup_guard_ticks = 0u;
        frozen_cfg.enable_zero_diagnostic_ticks = 100u;
        frozen_cfg.enable_zero_ticks = 100u;
        frozen_cfg.tracking_error_limit_ticks = 100000u;
        frozen_cfg.saturation_limit_ticks = 100000u;
        rotating_dq_current_test_init(&frozen_test, &frozen_cfg);
        rotating_dq_current_test_request_start(&frozen_test, true);
        frozen_test.controller.integrator_d_v = 0.40f;
        frozen_test.controller.integrator_q_v = -0.30f;

        for (uint32_t seq = 1u; seq <= 20u; ++seq) {
            frozen_in = rotating_nominal_input(seq, 1000);
            frozen_in.theta_e_rad = 0.0f;
            frozen_in.raw_pc0 = 1002u;
            frozen_in.raw_pc1 = 999u;
            frozen_in.offset_pc0 = 1000u;
            frozen_in.offset_pc1 = 1000u;
            frozen_in.current_amp_per_count = 0.020142f;
            frozen_in.iv_a = 0.040284f;
            frozen_in.iw_a = -0.020142f;
            rotating_dq_current_test_fast_isr(
                &frozen_test, &frozen_in, &frozen_out);
        }
        check_true(frozen_test.result == ROTATING_DQ_RESULT_RUNNING &&
                       nearf(frozen_test.controller.integrator_d_v, 0.0f, EPS) &&
                       nearf(frozen_test.controller.integrator_q_v, 0.0f, EPS) &&
                       nearf(frozen_out.vd_integrator_v, 0.0f, EPS) &&
                       nearf(frozen_out.vq_integrator_v, 0.0f, EPS) &&
                       nearf(frozen_out.vd_feedforward_v, 0.0f, EPS) &&
                       nearf(frozen_out.vq_feedforward_v, 0.0f, EPS),
                   "D-group zero-reference mode clears and locks both integrators");
        check_true(fabsf(frozen_out.vq_proportional_v) > EPS &&
                       nearf(frozen_out.vq_unclamped_v,
                             frozen_out.vq_proportional_v,
                             EPS) &&
                       nearf(frozen_out.vq_diagnostic_v,
                             frozen_out.vq_proportional_v,
                             EPS),
                   "D-group output retains proportional action with zero I and FF");

        frozen_cfg.freeze_zero_reference_integrator = false;
        rotating_dq_current_test_init(&frozen_test, &frozen_cfg);
        rotating_dq_current_test_request_start(&frozen_test, true);
        frozen_in.adc_seq = 1u;
        rotating_dq_current_test_fast_isr(
            &frozen_test, &frozen_in, &frozen_out);
        check_true(fabsf(frozen_test.controller.integrator_q_v) > EPS,
                   "normal zero-reference PI still integrates when D-group freeze is disabled");
    }

    {
        RotatingDqCurrentTestConfig p_only_cfg =
            rotating_dq_current_test_default_config();
        RotatingDqCurrentTest p_only_test;
        RotatingDqCurrentTestOutput p_only_out = {0};

        p_only_cfg.enable_zero_diagnostic_only = false;
        p_only_cfg.freeze_zero_reference_integrator = true;
        p_only_cfg.require_direction_match = false;
        p_only_cfg.iq_target_a = 0.020f;
        p_only_cfg.direction_capture_counts = 1000;
        p_only_cfg.tracking_error_limit_ticks = 100000u;
        p_only_cfg.saturation_limit_ticks = 100000u;
        rotating_dq_current_test_init(&p_only_test, &p_only_cfg);
        rotating_dq_current_test_request_start(&p_only_test, true);
        p_only_test.state = ROTATING_DQ_STATE_HOLD_IQ_POSITIVE;
        p_only_test.result = ROTATING_DQ_RESULT_RUNNING;
        p_only_test.iq_ref_a = 0.020f;
        p_only_test.controller.integrator_d_v = -0.25f;
        p_only_test.controller.integrator_q_v = 0.30f;

        for (uint32_t seq = 1u; seq <= 8u; ++seq) {
            RotatingDqCurrentTestInput p_only_in =
                rotating_nominal_input(seq, 1000);
            p_only_in.theta_e_rad = 0.0f;
            p_only_in.raw_pc0 = 1000u;
            p_only_in.raw_pc1 = 1000u;
            p_only_in.offset_pc0 = 1000u;
            p_only_in.offset_pc1 = 1000u;
            p_only_in.current_amp_per_count = 0.020142f;
            p_only_in.iv_a = 0.0f;
            p_only_in.iw_a = 0.0f;
            rotating_dq_current_test_fast_isr(
                &p_only_test, &p_only_in, &p_only_out);
        }

        check_true(p_only_test.result == ROTATING_DQ_RESULT_RUNNING &&
                       nearf(p_only_test.controller.integrator_d_v, 0.0f, EPS) &&
                       nearf(p_only_test.controller.integrator_q_v, 0.0f, EPS) &&
                       nearf(p_only_out.vd_integrator_v, 0.0f, EPS) &&
                       nearf(p_only_out.vq_integrator_v, 0.0f, EPS),
                   "P-only bipolar mode locks integrators at nonzero iq reference");
        check_true(p_only_out.vq_proportional_v > EPS &&
                       p_only_out.vq_v > EPS &&
                       nearf(p_only_out.vq_unclamped_v,
                             p_only_out.vq_proportional_v,
                             EPS),
                   "P-only bipolar mode retains proportional q-axis output");
    }

    rotating_tick(&test, ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 3u, 1000, &out);
    check_true(test.zero_dq_over_limit_consecutive == 0u,
               "clean ENABLE_ZERO sample clears dq over-limit consecutive count");
    for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
        const uint32_t seq =
            ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 4u + i;
        in = rotating_nominal_input(seq, 1000);
        in.theta_e_rad = rotating_dq_current_test_theta_from_count(&cfg, 1000, 1.0f);
        in.raw_pc0 = 1011u;
        in.raw_pc1 = 1005u;
        in.offset_pc0 = 1000u;
        in.offset_pc1 = 1000u;
        in.current_amp_per_count = 0.02f;
        in.iv_a = 0.12f;
        in.iw_a = 0.12f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true(((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u) &&
                   test.zero_dq_over_limit_consecutive ==
                       ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS,
               "four consecutive post-startup dq over-limit samples fault ENABLE_ZERO");

    cfg.phase_current_limit_a = 10.0f;
    cfg.dq_current_limit_a = 0.15f;
    cfg.enable_zero_diagnostic_ticks = 40u;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    in = rotating_nominal_input(1u, 1000);
    in.theta_e_rad = 0.0f;
    in.raw_pc0 = 1011u;
    in.raw_pc1 = 1005u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.10f;
    in.iw_a = 0.10f;
    memset(&out, 0, sizeof(out));
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(test.zero_first_trip.valid &&
                   test.result == ROTATING_DQ_RESULT_RUNNING &&
                   test.zero_dq_startup_over_limit_count == 1u &&
                   test.zero_dq_startup_over_limit_last_tick == 1u &&
                   test.zero_dq_fault_set_tick == 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u) &&
                   strcmp(rotating_dq_zero_diag_classification(&test),
                          "DQ_STARTUP_TRANSIENT_OBSERVED") == 0,
               "dq-only startup over-limit is observed without immediate fault");

    for (uint32_t seq = 2u; seq <= ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS; ++seq) {
        in = rotating_nominal_input(seq, 1000);
        in.theta_e_rad = 0.0f;
        in.raw_pc0 = 1011u;
        in.raw_pc1 = 1005u;
        in.offset_pc0 = 1000u;
        in.offset_pc1 = 1000u;
        in.current_amp_per_count = 0.02f;
        in.iv_a = 0.10f;
        in.iw_a = 0.10f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    in = rotating_nominal_input(ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 1u, 1000);
    in.theta_e_rad = 0.0f;
        in.raw_pc0 = 1011u;
        in.raw_pc1 = 1005u;
    in.offset_pc0 = 1000u;
    in.offset_pc1 = 1000u;
    in.current_amp_per_count = 0.02f;
    in.iv_a = 0.10f;
    in.iw_a = 0.10f;
    rotating_dq_current_test_fast_isr(&test, &in, &out);
    check_true(((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u) &&
                   test.zero_dq_fault_set_tick ==
                       (ROTATING_DQ_ENABLE_ZERO_STARTUP_OBSERVE_TICKS + 1u),
               "dq-only over-limit after startup observe window faults normally");
}

static void test_rotating_block_integrator_admission(void)
{
    const float ki = 2010.62f;
    const float dt = 0.00005f;
    RotatingDqBlockIntegratorAdmission admission;
    float id_delta = 0.0f;
    float iq_delta = 0.0f;
    float iq_delta_sum = 0.0f;
    uint32_t tick = 0u;

    rotating_dq_block_integrator_reset(&admission);
    for (uint32_t i = 0u; i < 20000u; ++i) {
        const float noise = ((i & 1u) == 0u) ? 0.040f : -0.040f;
        tick++;
        (void)rotating_dq_block_integrator_step(
            &admission, tick, 0.0f, noise, true, ki, dt,
            &id_delta, &iq_delta);
        iq_delta_sum += iq_delta;
    }
    check_true(admission.iq_axis.admit_count == 0u &&
                   !admission.iq_axis.admitted &&
                   nearf(iq_delta_sum, 0.0f, EPS),
               "E-group zero-mean quantization noise does not random-walk integrator");

    rotating_dq_block_integrator_reset(&admission);
    tick = 0u;
    for (uint32_t block = 0u; block < 6u; ++block) {
        const float block_error = ((block & 1u) == 0u) ? 0.060f : -0.060f;
        for (uint32_t sample = 0u;
             sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
             ++sample) {
            tick++;
            (void)rotating_dq_block_integrator_step(
                &admission, tick, 0.0f, block_error, true, ki, dt,
                &id_delta, &iq_delta);
        }
    }
    check_true(admission.iq_axis.admit_count == 0u &&
                   !admission.iq_axis.admitted,
               "E-group alternating above-threshold blocks fail same-sign admission");

    rotating_dq_block_integrator_reset(&admission);
    tick = 0u;
    iq_delta_sum = 0.0f;
    for (uint32_t block = 0u; block < 4u; ++block) {
        for (uint32_t sample = 0u;
             sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
             ++sample) {
            tick++;
            (void)rotating_dq_block_integrator_step(
                &admission, tick, 0.0f, 0.060f, true, ki, dt,
                &id_delta, &iq_delta);
            iq_delta_sum += iq_delta;
        }
        if (block < 3u) {
            check_true(nearf(iq_delta_sum, 0.0f, EPS),
                       "E-group confirmation blocks are not retroactively integrated");
        }
    }
    const float expected_block_delta =
        ki * dt * (float)ROTATING_DQ_BLOCK_INTEGRATOR_TICKS * 0.060f;
    check_true(admission.iq_axis.admitted &&
                   admission.iq_axis.admit_count == 1u &&
                   admission.iq_axis.active_block_count == 1u &&
                   nearf(iq_delta_sum, expected_block_delta, 1.0e-5f),
               "E-group persistent DC error admits then preserves Ki*Ts*sum(error)");

    for (uint32_t block = 0u; block < 2u; ++block) {
        for (uint32_t sample = 0u;
             sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
             ++sample) {
            tick++;
            (void)rotating_dq_block_integrator_step(
                &admission, tick, 0.0f, 0.020f, true, ki, dt,
                &id_delta, &iq_delta);
        }
    }
    check_true(!admission.iq_axis.admitted,
               "E-group two below-OFF blocks exit without threshold chatter");

    for (uint32_t block = 0u; block < 3u; ++block) {
        for (uint32_t sample = 0u;
             sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
             ++sample) {
            tick++;
            (void)rotating_dq_block_integrator_step(
                &admission, tick, 0.0f, 0.060f, true, ki, dt,
                &id_delta, &iq_delta);
        }
    }
    for (uint32_t sample = 0u;
         sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
         ++sample) {
        tick++;
        (void)rotating_dq_block_integrator_step(
            &admission, tick, 0.0f, -0.040f, true, ki, dt,
            &id_delta, &iq_delta);
    }
    check_true(!admission.iq_axis.admitted &&
                   admission.iq_axis.sign_reversal_count == 1u,
               "E-group clear reverse block exits and requires fresh admission");

    rotating_dq_block_integrator_reset(&admission);
    for (uint32_t sample = 0u;
         sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
         ++sample) {
        (void)rotating_dq_block_integrator_step(
            &admission,
            sample + 1u,
            0.0f,
            0.10f,
            sample < 27u,
            ki,
            dt,
            &id_delta,
            &iq_delta);
    }
    check_true(admission.invalid_block_count == 1u &&
                   admission.iq_axis.admit_count == 0u &&
                   nearf(iq_delta, 0.0f, EPS),
               "E-group harmful common-mode or ADC-invalid samples invalidate sparse block");

    {
        RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
        RotatingDqCurrentTest test;
        RotatingDqCurrentTestOutput out = {0};
        cfg.enable_zero_block_integrator = true;
        cfg.enable_zero_diagnostic_only = false;
        cfg.enable_zero_current_pi = false;
        cfg.iq_target_a = 0.060f;
        cfg.iq_ref_hard_limit_a = 0.080f;
        cfg.direction_capture_counts = 1000;
        cfg.iq_hold_ticks = 4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
        cfg.tracking_error_limit_ticks = 100000u;
        rotating_dq_current_test_init(&test, &cfg);
        rotating_dq_current_test_request_start(&test, true);
        test.state = ROTATING_DQ_STATE_HOLD_IQ_POSITIVE;
        test.iq_ref_a = cfg.iq_target_a;
        uint32_t seq = 0u;
        for (uint32_t block = 0u; block < 4u; ++block) {
            for (uint32_t sample = 0u;
                 sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
                 ++sample) {
                RotatingDqCurrentTestInput in =
                    rotating_nominal_input(++seq, 1000);
                in.iv_a = 0.0f;
                in.iw_a = 0.0f;
                rotating_dq_current_test_fast_isr(&test, &in, &out);
            }
            if (block < 3u) {
                check_true(nearf(test.controller.integrator_q_v, 0.0f, EPS),
                           "E2 confirmation blocks do not retroactively integrate");
            }
        }
        const float expected_positive_delta =
            test.controller.ki * cfg.dt_s *
            (float)ROTATING_DQ_BLOCK_INTEGRATOR_TICKS * cfg.iq_target_a;
        check_true(rotating_dq_block_integrator_diagnostic_state()
                           ->iq_axis.admit_count == 1u &&
                       nearf(test.controller.integrator_d_v, 0.0f, EPS) &&
                       nearf(test.controller.integrator_q_v,
                             expected_positive_delta,
                             1.0e-5f),
                   "E2 60mA positive DC error admits q integrator with preserved Ki");
        const RotatingDqBlockIntegratorHoldSnapshot *positive_hold =
            rotating_dq_block_integrator_positive_hold_snapshot();
        check_true(!positive_hold->valid,
                   "E2 HOLD snapshot copy is deferred out of fast ISR");
        rotating_dq_current_test_service_main(&test, &out);
        check_true(positive_hold->valid &&
                       positive_hold->hold_sample_count ==
                           cfg.iq_hold_ticks &&
                       positive_hold->tracking_sample_count > 0u &&
                       positive_hold->admission.completed_block_count == 4u &&
                       positive_hold->admission.iq_axis.admit_count == 1u &&
                       positive_hold->admission.iq_axis.active_block_count == 1u &&
                       nearf(positive_hold->integrator_q_end_v,
                             expected_positive_delta,
                             1.0e-5f),
                   "E2 positive HOLD snapshot captures block admission before reset");

        test.state = ROTATING_DQ_STATE_RAMP_ZERO_1;
        test.state_ticks = 0u;
        test.iq_ref_a = 0.0f;
        RotatingDqCurrentTestInput zero_in =
            rotating_nominal_input(++seq, 1000);
        rotating_dq_current_test_fast_isr(&test, &zero_in, &out);
        check_true(nearf(test.controller.integrator_q_v, 0.0f, EPS),
                   "E2 zero transition clears stored q integrator");
        check_true(positive_hold->valid &&
                       positive_hold->admission.completed_block_count == 4u &&
                       positive_hold->admission.iq_axis.admit_count == 1u,
                   "E2 positive HOLD snapshot survives zero-transition reset");

        test.state = ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE;
        test.state_ticks = 0u;
        test.iq_ref_a = -cfg.iq_target_a;
        for (uint32_t block = 0u; block < 4u; ++block) {
            for (uint32_t sample = 0u;
                 sample < ROTATING_DQ_BLOCK_INTEGRATOR_TICKS;
                 ++sample) {
                RotatingDqCurrentTestInput in =
                    rotating_nominal_input(++seq, 1000);
                in.iv_a = 0.0f;
                in.iw_a = 0.0f;
                rotating_dq_current_test_fast_isr(&test, &in, &out);
            }
        }
        check_true(rotating_dq_block_integrator_diagnostic_state()
                           ->iq_axis.admit_count == 1u &&
                       nearf(test.controller.integrator_d_v, 0.0f, EPS) &&
                       nearf(test.controller.integrator_q_v,
                             -expected_positive_delta,
                             1.0e-5f),
                   "E2 60mA reversed reference requires fresh admission and negative q integration");
        const RotatingDqBlockIntegratorHoldSnapshot *negative_hold =
            rotating_dq_block_integrator_negative_hold_snapshot();
        check_true(!negative_hold->valid,
                   "E2 negative HOLD snapshot copy is deferred out of fast ISR");
        rotating_dq_current_test_service_main(&test, &out);
        check_true(negative_hold->valid &&
                       negative_hold->hold_sample_count ==
                           cfg.iq_hold_ticks &&
                       negative_hold->tracking_sample_count > 0u &&
                       negative_hold->admission.completed_block_count == 4u &&
                       negative_hold->admission.iq_axis.admit_count == 1u &&
                       negative_hold->admission.iq_axis.active_block_count == 1u &&
                       nearf(negative_hold->integrator_q_end_v,
                             -expected_positive_delta,
                             1.0e-5f) &&
                       positive_hold->valid,
                   "E2 negative HOLD snapshot is independent of positive HOLD snapshot");
    }
}

static void test_rotating_direction_sequence(void)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTest test;
    RotatingDqCurrentTestOutput out = {0};
    int64_t enc = 0;

    printf("\n== rotating dq direction sequence ==\n");
    cfg.enable_zero_ticks = 4u;
    cfg.iq_hold_ticks = 3u;
    cfg.hold_zero_ticks = 2u;
    cfg.iq_ramp_rate_a_per_s = 10.0f;
    cfg.tracking_error_limit_ticks = 100000u;
    cfg.saturation_limit_ticks = 100000u;
    cfg.speed_limit_rpm = 100.0f;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        if (test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
            enc += 1;
        } else if (test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            enc -= 1;
        }
        rotating_tick(&test, seq, enc, &out);
        if (out.done) {
            break;
        }
    }
    check_true(fabsf(test.iq_ref_a) <= 0.030001f, "iq_ref hard limit maintained");
    check_true(out.result == ROTATING_DQ_RESULT_PASS, "+iq and -iq opposite directions pass");
    check_true(test.positive_stats.mechanical_direction ==
                   -test.negative_stats.mechanical_direction,
               "recorded positive/negative directions are opposite");
    check_true(rotating_dq_velocity_iq_sign_candidate(&test) == 1,
               "+iq producing positive motion maps positive speed to +iq");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    enc = 0;
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        if (test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
            enc -= 1;
        } else if (test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            enc += 1;
        }
        rotating_tick(&test, seq, enc, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS &&
                   rotating_dq_velocity_iq_sign_candidate(&test) == -1,
               "+iq producing negative motion maps positive speed to -iq");

    rotating_dq_current_test_init(&test, &cfg);
    test.config.direction_capture_counts = 3;
    test.config.speed_limit_rpm = 100000.0f;
    test.config.iq_ramp_rate_a_per_s = 0.05f;
    rotating_dq_current_test_request_start(&test, true);
    enc = 0;
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        if (test.state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE) {
            enc -= 1;
        }
        rotating_tick(&test, seq, enc, &out);
        if (test.state == ROTATING_DQ_STATE_RAMP_ZERO_1) {
            break;
        }
    }
    check_true(test.state == ROTATING_DQ_STATE_RAMP_ZERO_1 &&
                   test.positive_stats.mechanical_direction < 0 &&
                   ((out.fault_code & ROTATING_DQ_FAULT_OVERSPEED) == 0u),
               "+iq ramp exits on direction evidence before fixed hold");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    enc = 0;
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        if (test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE ||
            test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            enc += 1;
        }
        rotating_tick(&test, seq, enc, &out);
        if (out.done) {
            break;
        }
    }
    check_true((out.result == ROTATING_DQ_RESULT_FAIL) &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DIRECTION) != 0u),
               "same sustained direction for +/-iq fails");
    check_true(rotating_dq_velocity_iq_sign_candidate(&test) == 0,
               "invalid direction evidence yields no velocity iq sign candidate");

    rotating_dq_current_test_request_start(&test, true);
    check_true(out.result == ROTATING_DQ_RESULT_FAIL ||
                   test.result == ROTATING_DQ_RESULT_FAIL,
               "faulted rotating test does not auto-restart");

    rotating_dq_current_test_init(&test, &cfg);
    test.config.log_decimation = 1u;
    test.config.enable_zero_ticks = ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY + 20u;
    test.config.tracking_error_limit_ticks = 1000000u;
    test.config.saturation_limit_ticks = 1000000u;
    rotating_dq_current_test_request_start(&test, true);
    for (uint32_t seq = 1u; seq <= (ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY + 10u); ++seq) {
        rotating_tick(&test, seq, 0, &out);
    }
    check_true(test.log_count == ROTATING_DQ_CURRENT_TEST_LOG_CAPACITY &&
                   test.log_dropped > 0u,
               "rotating log buffer saturates without overflow");
}

static void test_rotating_low_current_tracking_sequence(void)
{
    RotatingDqCurrentTestConfig cfg = rotating_dq_current_test_default_config();
    RotatingDqCurrentTest test;
    RotatingDqCurrentTestOutput out = {0};
    int64_t enc = 0;

    printf("\n== rotating dq low current tracking sequence ==\n");
    cfg.enable_zero_ticks = 4u;
    cfg.iq_hold_ticks = 6u;
    cfg.hold_zero_ticks = 2u;
    cfg.iq_target_a = 0.05f;
    cfg.iq_ref_hard_limit_a = 0.06f;
    cfg.iq_ramp_rate_a_per_s = 1000.0f;
    cfg.require_direction_match = false;
    cfg.direction_capture_counts = 1000000;
    cfg.tracking_iq_ref_mean_min_a = 0.010f;
    cfg.tracking_iq_mean_min_a = 0.010f;
    cfg.tracking_id_mean_abs_limit_a = 0.10f;
    cfg.tracking_error_limit_ticks = 100000u;
    cfg.saturation_limit_ticks = 100000u;
    cfg.speed_limit_rpm = 100000.0f;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    test.state = ROTATING_DQ_STATE_RAMP_IQ_POSITIVE;
    test.result = ROTATING_DQ_RESULT_RUNNING;
    test.iq_ref_a = 0.04f;
    memset(&out, 0, sizeof(out));
    for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
        RotatingDqCurrentTestInput in = rotating_nominal_input(250u + i, 0);
        in.theta_e_rad = 4.565f;
        in.raw_pc0 = 993u;
        in.raw_pc1 = 994u;
        in.offset_pc0 = 1000u;
        in.offset_pc1 = 1000u;
        in.current_amp_per_count = 0.020142f;
        in.iv_a = -0.141f;
        in.iw_a = -0.121f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true(test.result == ROTATING_DQ_RESULT_RUNNING &&
                   test.zero_dq_over_limit_consecutive == 0u &&
                   ((out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) == 0u),
               "low-current tracking suppresses small same-sign two-shunt common-mode spikes");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    test.state = ROTATING_DQ_STATE_RAMP_IQ_POSITIVE;
    test.result = ROTATING_DQ_RESULT_RUNNING;
    test.iq_ref_a = 0.04f;
    memset(&out, 0, sizeof(out));
    for (uint32_t i = 0u; i < ROTATING_DQ_ENABLE_ZERO_SOFT_TRIP_TICKS; ++i) {
        RotatingDqCurrentTestInput in = rotating_nominal_input(300u + i, 0);
        in.theta_e_rad = 4.565f;
        in.raw_pc0 = 989u;
        in.raw_pc1 = 1000u;
        in.offset_pc0 = 1000u;
        in.offset_pc1 = 1000u;
        in.current_amp_per_count = 0.020142f;
        in.iv_a = -0.221f;
        in.iw_a = 0.0f;
        rotating_dq_current_test_fast_isr(&test, &in, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_FAIL &&
                   ((out.fault_code & ROTATING_DQ_FAULT_PHASE_CURRENT_LIMIT) != 0u ||
                    (out.fault_code & ROTATING_DQ_FAULT_DQ_CURRENT_LIMIT) != 0u),
               "low-current tracking still faults non-common-mode current limit");

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        float iq = 0.0f;
        if (test.state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE ||
            test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
            iq = 0.04f;
            enc += 1;
        } else if (test.state == ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE ||
                   test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            iq = -0.04f;
            enc += 1;
        }
        rotating_tick_with_iq(&test, seq, enc, iq, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS &&
                   test.positive_stats.iq_mean_a > 0.010f &&
                   test.negative_stats.iq_mean_a < -0.010f,
               "tracking mode passes on +/-iq sign even if encoder direction is same");

    {
        RotatingDqCurrentTestConfig hold_cfg =
            rotating_dq_current_test_default_config();
        RotatingDqCurrentTest hold_test;
        RotatingDqCurrentTestOutput hold_out = {0};
        hold_cfg.enable_zero_ticks = 2u;
        hold_cfg.enable_zero_block_integrator = true;
        hold_cfg.enable_zero_diagnostic_only = false;
        hold_cfg.iq_target_a = 0.020f;
        hold_cfg.iq_ref_hard_limit_a = 0.030f;
        hold_cfg.iq_ramp_rate_a_per_s = 2.0f;
        hold_cfg.iq_hold_ticks =
            (3u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS) +
            ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES;
        hold_cfg.hold_zero_ticks = 2u;
        hold_cfg.require_direction_match = false;
        hold_cfg.direction_capture_counts = 1000000;
        hold_cfg.tracking_iq_mean_min_a = 0.010f;
        hold_cfg.tracking_id_mean_abs_limit_a = 0.10f;
        hold_cfg.tracking_error_limit_ticks = 100000u;
        hold_cfg.saturation_limit_ticks = 100000u;
        hold_cfg.speed_limit_rpm = 100000.0f;
        rotating_dq_current_test_init(&hold_test, &hold_cfg);
        rotating_dq_current_test_request_start(&hold_test, true);
        for (uint32_t seq = 1u; seq < 2000u && !hold_out.done; ++seq) {
            float iq = 0.0f;
            if (hold_test.state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE) {
                iq = -0.040f;
            } else if (hold_test.state ==
                       ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
                iq = 0.011f;
            } else if (hold_test.state ==
                       ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE) {
                iq = 0.040f;
            } else if (hold_test.state ==
                       ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
                iq = -0.011f;
            }
            rotating_tick_with_iq(&hold_test, seq, 0, iq, &hold_out);
            rotating_dq_current_test_service_main(&hold_test, &hold_out);
        }
        const RotatingDqBlockIntegratorHoldSnapshot *pos_hold =
            rotating_dq_block_integrator_positive_hold_snapshot();
        const RotatingDqBlockIntegratorHoldSnapshot *neg_hold =
            rotating_dq_block_integrator_negative_hold_snapshot();
        check_true(hold_out.result == ROTATING_DQ_RESULT_PASS &&
                       hold_test.positive_stats.iq_mean_a < 0.0f &&
                       hold_test.negative_stats.iq_mean_a > 0.0f &&
                       pos_hold->valid && neg_hold->valid &&
                       pos_hold->tracking_sample_count >=
                           ROTATING_DQ_BIPOLAR_HOLD_TRACKING_MIN_SAMPLES &&
                       neg_hold->tracking_sample_count >=
                           ROTATING_DQ_BIPOLAR_HOLD_TRACKING_MIN_SAMPLES,
                   "E2 tracking verdict uses HOLD-only means, not P-only ramps");
    }

    cfg.single_direction_positive_only = true;
    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    enc = 0;
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        float iq = 0.0f;
        if (test.state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE ||
            test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
            iq = 0.04f;
            enc += 1;
        } else if (test.state == ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE ||
                   test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            iq = -0.04f;
            enc -= 1;
        }
        rotating_tick_with_iq(&test, seq, enc, iq, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_PASS &&
                   test.positive_stats.iq_mean_a > 0.010f &&
                   test.negative_stats.sample_count == 0u,
               "single-direction tracking passes without entering -iq stage");
    cfg.single_direction_positive_only = false;

    rotating_dq_current_test_init(&test, &cfg);
    rotating_dq_current_test_request_start(&test, true);
    memset(&out, 0, sizeof(out));
    enc = 0;
    for (uint32_t seq = 1u; seq < 200u && !out.done; ++seq) {
        float iq = 0.0f;
        if (test.state == ROTATING_DQ_STATE_RAMP_IQ_POSITIVE ||
            test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE) {
            iq = 0.04f;
        } else if (test.state == ROTATING_DQ_STATE_RAMP_IQ_NEGATIVE ||
                   test.state == ROTATING_DQ_STATE_HOLD_IQ_NEGATIVE) {
            iq = 0.04f;
        }
        rotating_tick_with_iq(&test, seq, enc, iq, &out);
    }
    check_true(out.result == ROTATING_DQ_RESULT_FAIL &&
                   ((out.fault_code & ROTATING_DQ_FAULT_TRACKING) != 0u),
               "tracking mode fails when -iq stage has wrong measured sign");

    {
        RotatingDqCurrentTest forced_test;
        RotatingDqCurrentTestConfig forced_cfg =
            rotating_dq_current_test_default_config();
        rotating_dq_current_test_init(&forced_test, &forced_cfg);
        forced_test.result = ROTATING_DQ_RESULT_PASS;
        forced_test.state = ROTATING_DQ_STATE_COMPLETE;
        forced_test.iq_ref_a = 0.03f;
        rotating_dq_current_test_force_fault(&forced_test,
                                             ROTATING_DQ_FAULT_TRACKING);
        check_true(forced_test.result == ROTATING_DQ_RESULT_FAIL &&
                       forced_test.state == ROTATING_DQ_STATE_FAIL &&
                       forced_test.iq_ref_a == 0.0f &&
                       ((forced_test.fault_code & ROTATING_DQ_FAULT_TRACKING) != 0u),
                   "post-completion admission fault changes PASS to FAIL");
    }

    {
        RotatingDqCurrentTest ext_test;
        RotatingDqCurrentTestOutput ext_out = {0};
        RotatingDqCurrentTestConfig ext_cfg =
            rotating_dq_current_test_default_config();
        ext_cfg.enable_zero_ticks = 1u;
        ext_cfg.iq_hold_ticks = 100u;
        ext_cfg.hold_zero_ticks = 2u;
        ext_cfg.single_direction_positive_only = true;
        ext_cfg.require_direction_match = false;
        ext_cfg.tracking_error_limit_ticks = 100000u;
        ext_cfg.saturation_limit_ticks = 100000u;
        rotating_dq_current_test_init(&ext_test, &ext_cfg);
        rotating_dq_current_test_request_start(&ext_test, true);
        rotating_tick(&ext_test, 1u, 0, &ext_out);

        RotatingDqCurrentTestInput ext_in = rotating_nominal_input(2u, 0);
        ext_in.external_iq_ref_valid = true;
        ext_in.external_iq_ref_a = 0.012f;
        rotating_dq_current_test_fast_isr(&ext_test, &ext_in, &ext_out);
        check_true(ext_test.state == ROTATING_DQ_STATE_HOLD_IQ_POSITIVE &&
                       nearf(ext_test.iq_ref_a, 0.012f, 0.000001f),
                   "external speed-loop iq command enters current-loop hold");

        ext_in = rotating_nominal_input(3u, 0);
        ext_in.external_iq_ref_valid = true;
        ext_in.external_iq_ref_a = -0.010f;
        rotating_dq_current_test_fast_isr(&ext_test, &ext_in, &ext_out);
        check_true(nearf(ext_test.iq_ref_a, -0.010f, 0.000001f),
                   "external speed-loop iq command may brake within hard limit");

        ext_in = rotating_nominal_input(4u, 0);
        ext_in.external_iq_ref_valid = true;
        ext_in.external_iq_ref_a = 0.031f;
        rotating_dq_current_test_fast_isr(&ext_test, &ext_in, &ext_out);
        check_true(ext_test.result == ROTATING_DQ_RESULT_FAIL &&
                       ((ext_test.fault_code & ROTATING_DQ_FAULT_IQ_REF_LIMIT) != 0u),
                    "external speed-loop iq command remains hard limited");
    }

    {
        RotatingDqCurrentTest ext_e2_test;
        RotatingDqCurrentTestOutput ext_e2_out = {0};
        RotatingDqCurrentTestConfig ext_e2_cfg =
            rotating_dq_current_test_default_config();
        ext_e2_cfg.enable_zero_ticks = 1u;
        ext_e2_cfg.iq_hold_ticks = 100000u;
        ext_e2_cfg.hold_zero_ticks = 100000u;
        ext_e2_cfg.require_direction_match = false;
        ext_e2_cfg.direction_capture_counts = 1000000;
        ext_e2_cfg.tracking_error_limit_ticks = 100000u;
        ext_e2_cfg.saturation_limit_ticks = 100000u;
        ext_e2_cfg.speed_limit_rpm = 100000.0f;
        ext_e2_cfg.enable_external_iq_block_integrator = true;
        ext_e2_cfg.tracking_iq_ref_mean_min_a = 0.005f;
        ext_e2_cfg.iq_ref_hard_limit_a = 0.080f;
        rotating_dq_current_test_init(&ext_e2_test, &ext_e2_cfg);
        rotating_dq_current_test_request_start(&ext_e2_test, true);
        rotating_tick(&ext_e2_test, 1u, 0, &ext_e2_out);

        uint32_t seq = 2u;
        for (uint32_t i = 0u;
             i < (4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS);
             ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = true;
            in.external_iq_ref_a = 0.010f;
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        const RotatingDqBlockIntegratorAdmission *external_admission =
            rotating_dq_block_integrator_diagnostic_state();
        check_true(!external_admission->iq_axis.admitted &&
                       external_admission->iq_axis.active_block_count == 0u &&
                       ext_e2_test.controller.integrator_q_v == 0.0f,
                   "sub-0.75-count external iq target cannot accumulate torque");

        RotatingDqCurrentTestInput reset_zero = rotating_nominal_input(seq++, 0);
        reset_zero.external_iq_ref_valid = true;
        reset_zero.external_integrator_enable = true;
        reset_zero.external_iq_ref_a = 0.0f;
        rotating_dq_current_test_fast_isr(&ext_e2_test,
                                          &reset_zero,
                                          &ext_e2_out);
        float low_res_p_v[2] = {0.0f, 0.0f};
        for (uint32_t i = 0u; i < 2u; ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            const float injected_iq_a = (i == 0u) ? 0.10f : -0.10f;
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = false;
            in.external_iq_ref_a = 0.020f;
            const float i_alpha_a = -sinf(in.theta_e_rad) * injected_iq_a;
            const float i_beta_a = cosf(in.theta_e_rad) * injected_iq_a;
            in.iv_a = 0.5f *
                (-i_alpha_a + 1.73205080757f * i_beta_a);
            in.iw_a = 0.5f *
                (-i_alpha_a - 1.73205080757f * i_beta_a);
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
            low_res_p_v[i] = ext_e2_out.vq_proportional_v;
            check_true(fabsf(ext_e2_out.iq_measured_a) > 0.09f &&
                           nearf(ext_e2_out.iq_control_a, 0.0f, EPS),
                       "low-resolution P averaging leaves raw iq visible to protection");
        }
        check_true(nearf(low_res_p_v[0], low_res_p_v[1], EPS) &&
                       nearf(low_res_p_v[0],
                             ext_e2_test.controller.kp * 0.020f,
                             EPS),
                   "one-count external iq P term ignores alternating raw ADC noise");
        for (uint32_t i = 0u;
             i < (4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS);
             ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = false;
            in.external_iq_ref_a = 0.020f;
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(!external_admission->iq_axis.admitted &&
                       ext_e2_test.controller.integrator_q_v == 0.0f,
                   "unready speed estimator gates one-count current integration");

        reset_zero = rotating_nominal_input(seq++, 0);
        reset_zero.external_iq_ref_valid = true;
        reset_zero.external_integrator_enable = true;
        reset_zero.external_iq_ref_a = 0.0f;
        rotating_dq_current_test_fast_isr(&ext_e2_test,
                                          &reset_zero,
                                          &ext_e2_out);
        for (uint32_t i = 0u;
             i < (4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS);
             ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = true;
            in.external_iq_ref_a = 0.020f;
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(external_admission->iq_axis.admitted &&
                       ext_e2_test.controller.integrator_q_v > 0.0f &&
                       ext_e2_test.controller.integrator_q_v <=
                           ROTATING_DQ_EXTERNAL_LOW_RES_MAX_DELTA_V + EPS,
                   "one-count external iq uses slew-limited block integration");
        check_true(nearf(ext_e2_out.vq_feedforward_v,
                         ext_e2_cfg.phase_resistance_ohm * 0.020f,
                         0.000001f) &&
                       ext_e2_out.vq_unclamped_v >
                           ext_e2_out.vq_proportional_v,
                   "external E2 iq adds R*iq feedforward before voltage limiting");

        const float low_res_ramp_a[] = {
            0.018f, 0.016f, 0.014f, 0.012f,
            0.010f, 0.008f, 0.006f, 0.005f
        };
        for (uint32_t i = 0u;
             i < (sizeof(low_res_ramp_a) / sizeof(low_res_ramp_a[0]));
             ++i) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq++, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = true;
            in.external_iq_ref_a = low_res_ramp_a[i];
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(external_admission->iq_axis.admitted &&
                       ext_e2_test.controller.integrator_q_v > 0.0f,
                   "low-resolution hysteresis survives quantized speed-reference dips");

        reset_zero = rotating_nominal_input(seq++, 0);
        reset_zero.external_iq_ref_valid = true;
        reset_zero.external_integrator_enable = true;
        reset_zero.external_iq_ref_a = 0.0f;
        rotating_dq_current_test_fast_isr(&ext_e2_test,
                                          &reset_zero,
                                          &ext_e2_out);
        for (uint32_t i = 0u;
             i < (4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS);
             ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = true;
            in.external_iq_ref_a = 0.060f;
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(external_admission->iq_axis.admitted &&
                       external_admission->iq_axis.active_block_count == 1u &&
                       ext_e2_test.controller.integrator_q_v > 0.0f &&
                       ext_e2_test.controller.integrator_d_v == 0.0f,
                   "external E2 reference admits after three blocks and preserves Ki");
        RotatingDqCurrentTestInput changed = rotating_nominal_input(seq++, 0);
        changed.external_iq_ref_valid = true;
        changed.external_integrator_enable = true;
        changed.external_iq_ref_a = 0.058f;
        rotating_dq_current_test_fast_isr(&ext_e2_test, &changed, &ext_e2_out);
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(external_admission->iq_axis.admitted &&
                       external_admission->completed_block_count > 0u &&
                       ext_e2_test.controller.integrator_q_v > 0.0f,
                   "external E2 same-sign slew preserves admission and q integrator");

        RotatingDqCurrentTestInput stepped = rotating_nominal_input(seq++, 0);
        stepped.external_iq_ref_valid = true;
        stepped.external_integrator_enable = true;
        stepped.external_iq_ref_a = 0.020f;
        rotating_dq_current_test_fast_isr(&ext_e2_test, &stepped, &ext_e2_out);
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(!external_admission->iq_axis.admitted &&
                       external_admission->completed_block_count == 0u &&
                       ext_e2_test.controller.integrator_q_v == 0.0f,
                   "external E2 large reference step clears admission and q integrator");

        RotatingDqCurrentTestInput zero = rotating_nominal_input(seq++, 0);
        zero.external_iq_ref_valid = true;
        zero.external_integrator_enable = true;
        zero.external_iq_ref_a = 0.0f;
        rotating_dq_current_test_fast_isr(&ext_e2_test, &zero, &ext_e2_out);
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(!external_admission->iq_axis.admitted &&
                       external_admission->block_tick_count == 0u &&
                       ext_e2_test.controller.integrator_d_v == 0.0f &&
                       ext_e2_test.controller.integrator_q_v == 0.0f,
                   "external E2 zero crossing clears both integrators and admission");

        for (uint32_t i = 0u;
             i < (4u * ROTATING_DQ_BLOCK_INTEGRATOR_TICKS);
             ++i, ++seq) {
            RotatingDqCurrentTestInput in = rotating_nominal_input(seq, 0);
            in.external_iq_ref_valid = true;
            in.external_integrator_enable = true;
            in.external_iq_ref_a = -0.060f;
            rotating_dq_current_test_fast_isr(&ext_e2_test, &in, &ext_e2_out);
        }
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(external_admission->iq_axis.admitted &&
                       external_admission->iq_axis.admitted_sign < 0 &&
                       ext_e2_test.controller.integrator_q_v < 0.0f &&
                       ext_e2_test.controller.integrator_d_v == 0.0f,
                   "external E2 negative reference requires fresh negative admission");
        check_true(nearf(ext_e2_out.vq_feedforward_v,
                         -ext_e2_cfg.phase_resistance_ohm * 0.060f,
                         0.000001f),
                   "external E2 resistance feedforward follows iq sign");

        RotatingDqCurrentTestInput missing = rotating_nominal_input(seq++, 0);
        rotating_dq_current_test_fast_isr(&ext_e2_test, &missing, &ext_e2_out);
        external_admission = rotating_dq_block_integrator_diagnostic_state();
        check_true(!external_admission->iq_axis.admitted &&
                       ext_e2_test.iq_ref_a == 0.0f &&
                       ext_e2_test.controller.integrator_q_v == 0.0f,
                    "missing external E2 command returns to safe zero without integration");
        check_true(ext_e2_test.positive_stats.sample_count == 0u,
                   "external E2 omits redundant direction statistics");
        const RotatingDqBlockIntegratorHoldSnapshot *bounded_hold =
            rotating_dq_block_integrator_positive_hold_snapshot();
        check_true(bounded_hold != NULL &&
                       bounded_hold->tracking_sample_count <=
                           ROTATING_DQ_BIPOLAR_HOLD_TRACKING_WINDOW_SAMPLES &&
                       bounded_hold->hold_sample_count < 512u,
                   "bounded runtime stops hold diagnostics after the evidence window");
        check_true(ext_e2_out.power_stage_request &&
                       ext_e2_out.pwm_output_request &&
                       !ext_e2_out.safe_shutdown_request,
                   "bounded diagnostics preserve the active control output contract");

        RotatingDqCurrentTest supervisor_owned = ext_e2_test;
        supervisor_owned.result = ROTATING_DQ_RESULT_RUNNING;
        supervisor_owned.state = ROTATING_DQ_STATE_HOLD_ZERO_1;
        supervisor_owned.state_ticks = 1u;
        supervisor_owned.config.hold_zero_ticks = 1u;
        supervisor_owned.config.single_direction_positive_only = true;
        supervisor_owned.config.direction_capture_counts = 1000000;
        supervisor_owned.positive_stats.sample_count = 0u;
        RotatingDqCurrentTestInput complete_in =
            rotating_nominal_input(seq, 0);
        complete_in.external_iq_ref_valid = true;
        complete_in.external_integrator_enable = true;
        complete_in.external_iq_ref_a = 0.0f;
        rotating_dq_current_test_fast_isr(&supervisor_owned,
                                          &complete_in,
                                          &ext_e2_out);
        check_true(supervisor_owned.result == ROTATING_DQ_RESULT_PASS &&
                       supervisor_owned.fault_code == ROTATING_DQ_FAULT_NONE,
                   "external speed supervisor owns final tracking verdict");
    }
}

static float test_count_for_theta(float theta_cmd,
                                  float offset,
                                  const ElectricalOffsetCalibrationConfig *cfg)
{
    float theta_m = (theta_cmd - offset) /
                    ((float)cfg->encoder_direction * (float)cfg->pole_pairs);
    return theta_m * (float)cfg->encoder_cpr / (2.0f * 3.14159265358979323846f);
}

static void test_electrical_offset_calibration_math(void)
{
    ElectricalOffsetCalibrationConfig cfg =
        electrical_offset_calibration_default_config();
    ElectricalOffsetCalibrationPoint points[ELECTRICAL_OFFSET_CAL_POINT_COUNT];
    ElectricalOffsetCalibrationResult res;
    const float offset = 6.20f;
    const float theta0 = 0.0f;
    const float theta_pos = 2.0f * 3.14159265358979323846f / 3.0f;
    const float theta_neg = -2.0f * 3.14159265358979323846f / 3.0f;

    printf("\n== electrical offset calibration math ==\n");

    points[0].theta_cmd_rad = theta0;
    points[1].theta_cmd_rad = theta_pos;
    points[2].theta_cmd_rad = theta_neg;
    points[0].encoder_count_mean = test_count_for_theta(theta0, offset, &cfg);
    points[1].encoder_count_mean = test_count_for_theta(theta_pos, offset, &cfg);
    points[2].encoder_count_mean = test_count_for_theta(theta_neg, offset, &cfg);
    points[0].valid = true;
    points[1].valid = true;
    points[2].valid = true;

    check_true(electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res),
               "three ideal alignment points recover valid offset");
    check_true(electrical_offset_angle_distance_rad(res.electrical_offset_rad,
                                                    offset) < 0.001f,
               "recovered electrical offset matches expected");
    check_true(res.max_offset_spread_deg <= 0.01f,
               "ideal offset spread is near zero");
    check_true(fabsf(res.expected_delta_count - 195.05f) <= 0.1f,
               "120 electrical degrees maps to about 195 encoder counts");
    check_true(fabsf(res.delta_count_0_to_pos120 - res.expected_delta_count) <= 0.1f &&
                   fabsf(res.delta_count_0_to_neg120 + res.expected_delta_count) <= 0.1f,
                "positive and negative 120 degree deltas have opposite signs");

    ElectricalOffsetCalibrationPoint hardware_like[ELECTRICAL_OFFSET_CAL_POINT_COUNT];
    hardware_like[0].theta_cmd_rad = theta0;
    hardware_like[1].theta_cmd_rad = theta_pos;
    hardware_like[2].theta_cmd_rad = theta_neg;
    hardware_like[0].encoder_count_mean = 210.0f;
    hardware_like[1].encoder_count_mean = 397.0f;
    hardware_like[2].encoder_count_mean = 594.0f;
    hardware_like[0].valid = true;
    hardware_like[1].valid = true;
    hardware_like[2].valid = true;
    check_true(electrical_offset_calibration_evaluate(&cfg,
                                                      hardware_like,
                                                      0.35f,
                                                      &res),
               "-120 command may settle at the +240 electrical equivalent");
    check_true(res.encoder_direction_ok && res.pole_pairs_ok,
               "equivalent -120 point still validates direction and pole pairs");
    check_true(fabsf(res.delta_count_0_to_neg120 - 384.0f) <= 0.1f,
               "hardware-like negative point keeps raw positive equivalent delta");
    check_true(fabsf(res.pole_pairs_est_average - 7.0f) <= 0.25f,
               "equivalent negative point estimates seven pole pairs");

    ElectricalOffsetCalibrationPoint first_point_stuck[ELECTRICAL_OFFSET_CAL_POINT_COUNT];
    first_point_stuck[0].theta_cmd_rad = theta0;
    first_point_stuck[1].theta_cmd_rad = theta_pos;
    first_point_stuck[2].theta_cmd_rad = theta_neg;
    first_point_stuck[0].encoder_count_mean = -25.0f;
    first_point_stuck[1].encoder_count_mean = 130.0f;
    first_point_stuck[2].encoder_count_mean = 327.0f;
    first_point_stuck[0].valid = true;
    first_point_stuck[1].valid = true;
    first_point_stuck[2].valid = true;
    check_true(!electrical_offset_calibration_evaluate(&cfg,
                                                        first_point_stuck,
                                                        0.35f,
                                                        &res),
               "hardware first-point stiction remains rejected");
    first_point_stuck[0].encoder_count_mean = -65.0f;
    check_true(electrical_offset_calibration_evaluate(&cfg,
                                                       first_point_stuck,
                                                       0.35f,
                                                       &res),
               "same-direction preconditioned point0 restores three-point geometry");

    points[0].encoder_count_mean += 4096.0f;
    check_true(electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res),
               "encoder count wrap still evaluates");
    check_true(electrical_offset_angle_distance_rad(res.electrical_offset_rad,
                                                    offset) < 0.001f,
               "wrapped encoder count preserves circular offset");
    points[0].encoder_count_mean -= 4096.0f;

    const float near_zero = 0.04f;
    points[0].encoder_count_mean = test_count_for_theta(theta0, near_zero, &cfg);
    points[1].encoder_count_mean = test_count_for_theta(theta_pos, near_zero, &cfg);
    points[2].encoder_count_mean = test_count_for_theta(theta_neg, near_zero, &cfg);
    points[2].encoder_count_mean += 4096.0f;
    check_true(electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res),
               "circular mean handles offsets around 0/2pi boundary");
    check_true(electrical_offset_angle_distance_rad(res.electrical_offset_rad,
                                                    near_zero) < 0.001f,
               "circular mean does not behave like ordinary arithmetic mean");

    points[2].encoder_count_mean =
        test_count_for_theta(theta_neg, near_zero, &cfg) + 80.0f;
    check_true(!electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res) &&
                   ((res.fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_SPREAD) != 0u),
               "offset spread over 8 electrical degrees fails");

    points[2].encoder_count_mean = test_count_for_theta(theta_neg, offset, &cfg);
    points[0].encoder_count_mean = test_count_for_theta(theta0, offset, &cfg);
    points[1].encoder_count_mean = test_count_for_theta(theta_pos, offset, &cfg);
    cfg.encoder_direction = -1;
    check_true(!electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res) &&
                   ((res.fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_DIRECTION) != 0u),
               "encoder direction mismatch fails");

    cfg = electrical_offset_calibration_default_config();
    points[1].encoder_count_mean = points[0].encoder_count_mean + 140.0f;
    points[2].encoder_count_mean = points[0].encoder_count_mean - 140.0f;
    check_true(!electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res) &&
                   ((res.fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_POLE_PAIRS) != 0u),
               "wrong pole pair count fails");

    points[1].encoder_count_mean = test_count_for_theta(theta_pos, offset, &cfg);
    points[2].encoder_count_mean = test_count_for_theta(theta_neg, offset, &cfg);
    check_true(!electrical_offset_calibration_evaluate(&cfg, points, 0.41f, &res) &&
                   ((res.fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_VOLTAGE_LIMIT) != 0u),
               "alignment voltage over 0.40V fails");

    points[1].valid = false;
    check_true(!electrical_offset_calibration_evaluate(&cfg, points, 0.35f, &res) &&
                   ((res.fail_flags & ELECTRICAL_OFFSET_CAL_FAIL_POINT_INVALID) != 0u),
               "invalid alignment point fails");
}

static ElectricalOffsetPreAlignmentGateInput good_pre_alignment_gate(void)
{
    ElectricalOffsetPreAlignmentGateInput in;
    memset(&in, 0, sizeof(in));
    in.admission_preflight_pass = true;
    in.admission_handoff_pass = true;
    in.admission_active = false;
    in.admission_disabled_ack = true;
    in.snapshots_after_admission_disable = 2u;
    in.admission_hook_calls_after_disable = 0u;
    in.overlap_count = 0u;
    in.offset_pc0_valid = true;
    in.offset_pc1_valid = true;
    in.dc_cal_bits_clear = true;
    in.gate_enabled = true;
    in.nfault_asserted = false;
    in.producer_gap_count = 0u;
    in.producer_duplicate_count = 0u;
    in.true_unpaired_count = 0u;
    in.torn_count = 0u;
    in.generation_mismatch_count = 0u;
    in.alignment_dispatch_enabled = true;
    in.alignment_active = true;
    in.command_flag_zero = true;
    in.v_alpha_zero = true;
    in.v_beta_zero = true;
    in.voltage_magnitude_zero = true;
    in.modulation_command_zero = true;
    in.last_applied_command_zero = true;
    in.voltage_command_pending_clear = true;
    in.voltage_command_seq_stable = true;
    in.pwm_shadow_safe = true;
    in.pwm_active_safe = true;
    in.ccr1_safe = true;
    in.ccr2_safe = true;
    in.ccr3_safe = true;
    in.ccr_safe_for_moe_off = true;
    in.ccr_safe_for_moe_enable = false;
    in.ccr_alignment_start_ready = false;
    in.moe_off = true;
    in.drv_ready = true;
    return in;
}

static void test_electrical_offset_state_flow(void)
{
    ElectricalOffsetPreAlignmentGateInput in = good_pre_alignment_gate();
    ElectricalOffsetPreAlignmentGateResult r;
    CurrentSensorAdmissionResult good = {0};
    CurrentSensorAdmissionPreflightVerdict v;

    printf("\n== electrical offset admission handoff state flow ==\n");

    check_true(electrical_offset_handoff_next_state(true, false) ==
                   ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP,
               "handoff pass requests ALIGN_POINT_0_RAMP");
    check_true(electrical_offset_handoff_next_state(false, false) ==
                   ELECTRICAL_OFFSET_STATE_FAIL,
               "handoff fail requests FAIL");
    check_true(strcmp(electrical_offset_state_name(ELECTRICAL_OFFSET_STATE_PREFLIGHT_HANDOFF),
                      "PREFLIGHT_HANDOFF") == 0,
               "PREFLIGHT_HANDOFF state string is stable");
    check_true(strcmp(electrical_offset_failure_name(ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY),
                      "electrical_offset_drv_not_ready") == 0,
               "failure enum string is specific");

    good.admission_pass = true;
    good.mean_pass = true;
    good.std_pass = true;
    good.consecutive_pass = true;
    good.hard_outlier_pass = true;
    v = current_sensor_admission_evaluate_preflight(&good,
                                                    1555u,
                                                    2851u,
                                                    168000000u,
                                                    true,
                                                    true,
                                                    true,
                                                    true);
    check_true(!v.target_5us_met && v.deadline_pass && v.preflight_pass,
               "target_5us_met=0 does not set hard preflight fault");

    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(r.pass &&
                   r.admission_pass &&
                   r.handoff_pass &&
                   r.offsets_pass &&
                   r.dc_cal_clear_pass &&
                   r.adc_pass &&
                   r.drv_pass &&
                   r.dispatch_pass &&
                   r.software_zero_command_pass &&
                   r.applied_zero_command_pass &&
                   r.zero_command_pass &&
                   r.moe_still_off_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_NONE,
               "all prealignment gate items pass into ALIGN_POINT_0_RAMP");

    in = good_pre_alignment_gate();
    in.snapshots_after_admission_disable = 1u;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.handoff_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ADMISSION_HANDOFF_FAILED,
               "disable ack waits for two coherent snapshots");

    in = good_pre_alignment_gate();
    in.admission_active = true;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ADMISSION_REJECTED,
               "admission active blocks alignment dispatcher");

    in = good_pre_alignment_gate();
    in.overlap_count = 1u;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_HOOK_OVERLAP,
               "admission/alignment overlap is classified");

    in = good_pre_alignment_gate();
    in.offset_pc1_valid = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.offsets_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_OFFSETS_INVALID,
               "offset PC0/PC1 must survive handoff");

    in = good_pre_alignment_gate();
    in.dc_cal_bits_clear = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.dc_cal_clear_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_DC_CAL_NOT_CLEAR,
               "DC_CAL uncleared has a specific failure");

    in = good_pre_alignment_gate();
    in.gate_enabled = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.drv_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY,
               "EN_GATE low is drv_not_ready, not vague FAIL");

    in = good_pre_alignment_gate();
    in.drv_ready = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY,
               "DRV readback failure is drv_not_ready");

    in = good_pre_alignment_gate();
    in.producer_gap_count = 1u;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.adc_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID,
               "producer gap blocks prealignment gate");

    in = good_pre_alignment_gate();
    in.producer_duplicate_count = 1u;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID,
               "producer duplicate blocks prealignment gate");

    in = good_pre_alignment_gate();
    in.true_unpaired_count = 1u;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ADC_PIPELINE_INVALID,
               "true unpaired blocks prealignment gate");

    in = good_pre_alignment_gate();
    in.alignment_dispatch_enabled = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.dispatch_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DISPATCH_NOT_READY,
               "alignment dispatcher must be enabled before ramp");

    in = good_pre_alignment_gate();
    in.alignment_active = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_DISPATCH_NOT_READY,
               "alignment active flag gates dispatcher");

    in = good_pre_alignment_gate();
    in.v_alpha_zero = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.software_zero_command_pass &&
                   !r.zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO,
               "v_alpha command must be zero before ALIGN_POINT_0_RAMP");

    in = good_pre_alignment_gate();
    in.v_beta_zero = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.software_zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO,
               "v_beta command must be zero before ALIGN_POINT_0_RAMP");

    in = good_pre_alignment_gate();
    in.voltage_magnitude_zero = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.software_zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_ZERO,
               "voltage magnitude must be zero before ALIGN_POINT_0_RAMP");

    in = good_pre_alignment_gate();
    in.last_applied_command_zero = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.applied_zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_APPLIED,
               "last applied command must be zero before ALIGN_POINT_0_RAMP");

    in = good_pre_alignment_gate();
    in.voltage_command_pending_clear = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.applied_zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_PENDING_VOLTAGE_UPDATE,
               "pending voltage update blocks pre-gate check");

    in = good_pre_alignment_gate();
    in.voltage_command_seq_stable = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.applied_zero_command_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ZERO_COMMAND_NOT_APPLIED,
               "software zero with unapplied sequence fails");

    in = good_pre_alignment_gate();
    in.ccr1_safe = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_CCR_NOT_SAFE,
               "CCR safe value is required before alignment");

    in = good_pre_alignment_gate();
    in.moe_off = false;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   !r.moe_still_off_pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_CCR_NOT_SAFE,
               "ALIGN_POINT_0_RAMP gate check keeps MOE off");

    in = good_pre_alignment_gate();
    in.nfault_asserted = true;
    r = electrical_offset_pre_alignment_gate_evaluate(&in);
    check_true(!r.pass &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_DRV_NOT_READY,
               "nFAULT asserted is drv_not_ready");

    check_true(electrical_offset_failure_name(ELECTRICAL_OFFSET_FAILURE_NONE)[0] == 'n',
               "first-failure latch can preserve none until first error");
    check_true(electrical_offset_failure_name(ELECTRICAL_OFFSET_FAILURE_STALE_FAULT_LATCHED)[0] == 'e',
               "stale fault has explicit enum for logs");
    check_true(electrical_offset_handoff_next_state(true, true) ==
                   ELECTRICAL_OFFSET_STATE_ALIGN_POINT_0_RAMP,
               "handoff helper never fall-throughs to FAIL in same iteration");
}

static ElectricalOffsetAlignmentProtectionInput good_alignment_protection_input(void)
{
    ElectricalOffsetAlignmentProtectionInput in;
    memset(&in, 0, sizeof(in));
    in.raw_pc0 = 2000u;
    in.raw_pc1 = 2000u;
    in.delta_pc0_counts = 0;
    in.delta_pc1_counts = 0;
    in.iu_a = 0.0f;
    in.iv_a = 0.0f;
    in.iw_a = 0.0f;
    in.id_a = 0.0f;
    in.iq_a = 0.0f;
    in.phase_abs_a = 0.0f;
    in.current_amp_per_count = 0.020142f;
    in.soft_limit_a = 0.30f;
    in.phase_emergency_limit_a = 0.50f;
    in.raw_hard_limit_counts = 24;
    in.soft_consecutive_required = 4u;
    in.raw_min_safe_count = 100u;
    in.raw_max_safe_count = 3995u;
    in.offset_valid = true;
    in.current_finite = true;
    in.nfault_ok = true;
    return in;
}

static void test_electrical_offset_alignment_protection(void)
{
    ElectricalOffsetAlignmentProtectionState st;
    ElectricalOffsetAlignmentProtectionInput in;
    ElectricalOffsetAlignmentProtectionResult r;
    float vu = 0.0f, vv = 0.0f, vw = 0.0f, vll = 0.0f;

    printf("\n== electrical offset alignment protection ==\n");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.iv_a = 0.25f;
    in.phase_abs_a = 0.25f;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(!r.trip &&
                   r.soft_consecutive_count == 0u &&
                   (r.source_mask & ELECTRICAL_OFFSET_ALIGNMENT_SOURCE_PHASE_SOFT) == 0u,
               "0.25A phase sample stays below revised alignment soft limit");

    electrical_offset_alignment_protection_reset(&st);
    for (uint32_t i = 0u; i < 4u; ++i) {
        in = good_alignment_protection_input();
        in.iw_a = -0.31f;
        in.phase_abs_a = 0.31f;
        r = electrical_offset_alignment_protection_update(&st, &in);
    }
    check_true(r.trip &&
                   r.soft_trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT &&
                   r.soft_consecutive_count == 4u,
               "four consecutive soft samples trip alignment soft overcurrent");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.delta_pc0_counts = 25;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(r.trip &&
                   r.immediate_trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_RAW_PC0_OVERCURRENT,
               "raw PC0 hard count limit trips immediately");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.phase_abs_a = 0.51f;
    in.iu_a = 0.51f;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(r.trip &&
                   r.immediate_trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_HARD_OVERCURRENT,
               "phase emergency current trips immediately");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.raw_pc1 = 3995u;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(r.trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_ADC_SATURATION,
               "ADC saturation trips immediately");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.nfault_ok = false;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(r.trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_DRV_RUNTIME_FAULT,
               "nFAULT is an immediate DRV runtime fault");

    electrical_offset_alignment_protection_reset(&st);
    in = good_alignment_protection_input();
    in.offset_valid = false;
    r = electrical_offset_alignment_protection_update(&st, &in);
    check_true(r.trip &&
                   r.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_OFFSET_SHIFT,
               "invalid or shifted offset blocks MOE/alignment");

    electrical_offset_alpha_beta_to_phase(0.35f, 0.0f, &vu, &vv, &vw, &vll);
    check_true(nearf(vu, 0.35f, 0.0001f), "0deg alpha command U phase is +Valpha");
    check_true(nearf(vv, -0.175f, 0.0001f), "0deg alpha command V phase is -Valpha/2");
    check_true(nearf(vw, -0.175f, 0.0001f), "0deg alpha command W phase is -Valpha/2");
    check_true(nearf(vll, 0.525f, 0.0001f), "0.35V alpha command line-line peak is 1.5x");
    check_true(nearf(electrical_offset_expected_phase_current_phase_resistance(0.35f, 3.20f),
                     0.109375f,
                     0.0001f),
               "3.20ohm phase definition expected current");
    check_true(nearf(electrical_offset_expected_phase_current_line_line_resistance(0.35f, 3.20f),
                     0.21875f,
                     0.0001f),
               "3.20ohm line-line definition expected current");

    check_true(strcmp(electrical_offset_failure_name(
                          ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT),
                      "electrical_offset_alignment_phase_soft_overcurrent") == 0,
               "specific alignment soft overcurrent failure string");
}

static void test_electrical_offset_timing_and_formatting(void)
{
    printf("\n== electrical offset timing and formatting ==\n");

    check_true(electrical_offset_dwt_elapsed(0xFFFFFFF0u, 0x00000020u) == 0x30u,
               "DWT uint32 wrap elapsed is correct");

    ElectricalOffsetTimingInput in;
    ElectricalOffsetTimingVerdict v;
    memset(&in, 0, sizeof(in));
    in.cpu_hz = 168000000u;
    in.scope_valid = true;
    in.alignment_fast_hook_core_cycles = 1000u;
    in.adc_callback_total_cycles = 4000u;
    in.alignment_state_transition_once_cycles = 6000u;
    in.main_service_cycles = 45000u;
    v = electrical_offset_timing_evaluate(&in);
    check_true(v.pass &&
                   !v.alignment_core_overrun &&
                   !v.adc_callback_overrun &&
                   v.old_main_scope_would_overrun &&
                   v.entry_init_overrun,
               "core<20us and callback<50us pass even if entry/main exceed 20us");

    in.alignment_fast_hook_core_cycles = 3360u;
    v = electrical_offset_timing_evaluate(&in);
    check_true(!v.pass &&
                   v.alignment_core_overrun &&
                   v.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CORE_OVERRUN,
               "true alignment fast hook core >=20us fails");

    in.alignment_fast_hook_core_cycles = 1000u;
    in.adc_callback_total_cycles = 8400u;
    v = electrical_offset_timing_evaluate(&in);
    check_true(!v.pass &&
                   v.adc_callback_overrun &&
                   v.failure == ELECTRICAL_OFFSET_FAILURE_ADC_CALLBACK_OVERRUN,
               "true ADC callback >=50us fails");

    in.adc_callback_total_cycles = 4000u;
    in.scope_valid = false;
    v = electrical_offset_timing_evaluate(&in);
    check_true(!v.pass &&
                   v.timing_scope_invalid &&
                   v.failure == ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_TIMING_SCOPE_INVALID,
               "invalid timing scope is classified separately");

    char buf[16];
    (void)electrical_offset_format_signed_milli(buf, sizeof(buf), 350, true);
    check_true(strcmp(buf, "+0.350") == 0, "positive voltage format keeps plus sign");
    (void)electrical_offset_format_signed_milli(buf, sizeof(buf), -175, true);
    check_true(strcmp(buf, "-0.175") == 0, "negative sub-unit voltage keeps minus sign");

    check_true(strcmp(electrical_offset_failure_name(
                          ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_CORE_OVERRUN),
                      "electrical_offset_alignment_core_overrun") == 0,
               "alignment core overrun failure string");
    check_true(strcmp(electrical_offset_failure_name(
                          ELECTRICAL_OFFSET_FAILURE_ADC_CALLBACK_OVERRUN),
                      "electrical_offset_adc_callback_overrun") == 0,
               "ADC callback overrun failure string");
}

static void test_electrical_offset_pwm_zero_diagnostic_helpers(void)
{
    printf("\n== electrical offset PWM zero diagnostic helpers ==\n");

    ElectricalOffsetPwmZeroStartInput start;
    memset(&start, 0, sizeof(start));
    start.commanded_v_alpha_zero = true;
    start.commanded_v_beta_zero = true;
    start.applied_v_alpha_zero = true;
    start.applied_v_beta_zero = true;
    start.duty_u_eq_v = true;
    start.duty_v_eq_w = true;
    start.ccr1_eq_ccr2 = true;
    start.ccr2_eq_ccr3 = true;
    start.preload_ack = true;
    start.tim_updates_after_preload = true;
    start.pending_voltage_update_clear = true;
    start.dc_cal_bits_clear = true;
    start.nfault_ok = true;
    start.gate_enabled = true;
    start.moe_off_before_enable = true;

    ElectricalOffsetPwmZeroStartResult sr =
        electrical_offset_pwm_zero_start_evaluate(&start);
    check_true(sr.safe_to_enable_moe,
               "zero command with equal duty/CCR and preload ack may enable MOE");
    start.ccr2_eq_ccr3 = false;
    sr = electrical_offset_pwm_zero_start_evaluate(&start);
    check_true(!sr.safe_to_enable_moe && !sr.ccrs_equal,
               "zero diagnostic blocks MOE when CCRs are unequal");
    start.ccr2_eq_ccr3 = true;
    start.preload_ack = false;
    sr = electrical_offset_pwm_zero_start_evaluate(&start);
    check_true(!sr.safe_to_enable_moe && !sr.preload_complete,
               "zero diagnostic blocks MOE before preload ack");

    check_true(electrical_offset_reconstructed_iu_counts(-13, 3) == 10,
               "PC0/PC1 deltas reconstruct IU by -(IV+IW)");
    check_true(nearf(electrical_offset_common_mode_shift_counts(-13.0f, 3.0f),
                     -5.0f,
                     EPS),
               "common-mode shift is mean of PC0/PC1 deltas");
    check_true(nearf(electrical_offset_differential_shift_counts(-13.0f, 3.0f),
                     -8.0f,
                     EPS),
               "differential shift is half delta difference");
    check_true(electrical_offset_min_phase_edge_distance_counts(3799u,
                                                                2100u,
                                                                2099u,
                                                                2099u) == 1699u,
               "PWM edge distance uses nearest CCR edge");

    ElectricalOffsetAlignmentProtectionState prot_state;
    ElectricalOffsetAlignmentProtectionResult pr;
    ElectricalOffsetAlignmentProtectionInput prot_in =
        good_alignment_protection_input();
    electrical_offset_alignment_protection_reset(&prot_state);
    for (uint32_t i = 0u; i < 4u; ++i) {
        prot_in.iu_a = 0.31f;
        prot_in.id_a = 0.31f;
        prot_in.phase_abs_a = 0.31f;
        pr = electrical_offset_alignment_protection_update(&prot_state,
                                                           &prot_in);
    }
    check_true(pr.trip &&
                   pr.failure ==
                       ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT,
               "normal alignment still trips on four revised soft overcurrent samples");

    ElectricalOffsetPwmZeroClassifyInput ci;
    memset(&ci, 0, sizeof(ci));
    ci.ccrs_equal = true;
    ci.line_to_line_zero = true;
    ci.stable_pc0_pc1_shift = true;
    ci.speed_near_zero = true;
    ci.nfault_ok = true;
    ci.raw_within_limits = true;
    ci.reconstruction_consistent = true;
    check_true(electrical_offset_pwm_zero_classify(&ci) ==
                   ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ACTIVE_OFFSET_SHIFT,
               "stable zero-voltage MOE-on raw shift classifies as PWM active offset shift");

    ci.near_switch_edge = true;
    check_true(electrical_offset_pwm_zero_classify(&ci) ==
                   ELECTRICAL_OFFSET_PWM_ZERO_CLASS_ADC_SWITCHING_EDGE_CONTAMINATION,
               "near-edge samples classify as ADC switching-edge contamination");

    ci.near_switch_edge = false;
    ci.ccrs_equal = false;
    check_true(electrical_offset_pwm_zero_classify(&ci) ==
                   ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ZERO_STATE_INVALID,
               "unequal CCRs classify as invalid zero PWM state");

    memset(&ci, 0, sizeof(ci));
    ci.ccrs_equal = true;
    ci.line_to_line_zero = true;
    ci.raw_within_limits = true;
    ci.reconstruction_consistent = false;
    check_true(electrical_offset_pwm_zero_classify(&ci) ==
                   ELECTRICAL_OFFSET_PWM_ZERO_CLASS_CURRENT_RECONSTRUCTION_INVALID,
               "inconsistent raw-to-IU math classifies as reconstruction invalid");

    memset(&ci, 0, sizeof(ci));
    ci.ccrs_equal = true;
    ci.line_to_line_zero = true;
    ci.current_ramp_like = true;
    check_true(electrical_offset_pwm_zero_classify(&ci) ==
                   ELECTRICAL_OFFSET_PWM_ZERO_CLASS_POSSIBLE_REAL_UNINTENDED_CURRENT,
               "ramp-like current classifies as possible real unintended current");

    check_true(strcmp(electrical_offset_pwm_zero_classification_name(
                          ELECTRICAL_OFFSET_PWM_ZERO_CLASS_PWM_ACTIVE_OFFSET_SHIFT),
                      "PWM_ACTIVE_OFFSET_SHIFT") == 0,
               "PWM zero classification name is stable for logs");
}

static void test_electrical_offset_trigger_sweep_helpers(void)
{
    printf("\n== electrical offset PWM zero trigger sweep helpers ==\n");

    ElectricalOffsetTriggerSweepTimingInput tin;
    memset(&tin, 0, sizeof(tin));
    tin.pclk2_hz = 84000000u;
    tin.apb2_prescaler = 2u;
    tin.tim_prescaler = 0u;
    tin.arr = 4199u;
    tin.phase_ccr = 2099u;
    tin.deadtime_counts = 50u;
    ElectricalOffsetTriggerSweepTiming timing =
        electrical_offset_trigger_sweep_timing(&tin);
    check_true(timing.tim_input_clock_hz == 168000000u,
               "TIM1 clock doubles when APB2 prescaler is not 1");
    check_true(timing.tim_counter_clock_hz == 168000000u,
               "TIM1 counter clock accounts for prescaler");
    check_true(nearf(timing.timer_tick_ns, 5.952381f, 0.01f),
               "TIM1 count/ns conversion is correct");
    check_true(nearf(timing.pwm_frequency_hz, 20000.0f, 0.5f),
               "center-aligned PWM frequency calculation is correct");
    check_true(timing.low_side_window_start_count == 2149u &&
                   timing.low_side_window_end_count == 4149u,
               "low-side valid window excludes switching edge and deadtime");
    check_true(timing.low_side_window_width_counts == 2000u,
               "low-side window width is computed");

    float guards[] = {0.20f, 0.40f, 0.75f, 1.00f, 1.50f,
                      2.00f, 3.00f, 4.00f, 6.00f};
    ElectricalOffsetTriggerSweepCandidate cand[16];
    memset(cand, 0, sizeof(cand));
    uint32_t n = electrical_offset_trigger_sweep_generate_candidates(
        &timing, guards, (uint32_t)(sizeof(guards) / sizeof(guards[0])),
        2099u, cand, 16u);
    check_true(n >= 10u, "candidate generation includes baseline, guards, and quiet center");
    check_true(cand[0].is_baseline && !cand[0].candidate_valid,
               "current trigger is included as invalid baseline when outside low-side window");
    check_true(cand[0].programmed_trigger_count == 2099u,
               "baseline preserves the existing CCR4 value for diagnosis");
    check_true(cand[1].programmed_trigger_count == 2183u,
               "first guard candidate is based on actual half-CCR low-side window");
    check_true(cand[1].programmed_trigger_count > timing.low_side_window_start_count,
               "guard candidate is placed inside the low-side window");
    check_true(cand[1].expected_low_side_state,
               "candidate generation only accepts low-side sample states");
    bool has_center = false;
    for (uint32_t i = 0u; i < n; ++i) {
        if (cand[i].is_quiet_center) {
            has_center = true;
            check_true(cand[i].candidate_valid, "quiet-center candidate is valid");
        }
    }
    check_true(has_center, "quiet-center candidate is generated");

    ElectricalOffsetTriggerSweepCandidate fixed[4];
    memset(fixed, 0, sizeof(fixed));
    uint32_t fixed_n = electrical_offset_trigger_sweep_generate_fixed_candidates(
        &timing, 3149u, 3u, fixed, 4u);
    check_true(fixed_n == 3u, "fixed CCR4 verification generates three repeats");
    for (uint32_t i = 0u; i < fixed_n; ++i) {
        check_true(fixed[i].programmed_trigger_count == 3149u,
                   "fixed CCR4 verification preserves the recommended trigger count");
        check_true(fixed[i].candidate_valid && fixed[i].expected_low_side_state,
                   "fixed CCR4 verification candidate is inside the half-CCR low-side window");
        check_true(!fixed[i].is_baseline && !fixed[i].is_quiet_center,
                   "fixed CCR4 verification candidates are explicit repeats, not search points");
    }

    ElectricalOffsetTriggerSweepCandidateResult res[16];
    memset(res, 0, sizeof(res));
    for (uint32_t i = 0u; i < n; ++i) {
        res[i].candidate_valid = cand[i].candidate_valid;
        res[i].delta_pc0_mean_counts = 0.5f;
        res[i].delta_pc1_mean_counts = -0.5f;
        res[i].pc0_std_counts = 2.0f;
        res[i].pc1_std_counts = 2.0f;
        res[i].iu_mean_a = 0.01f;
        res[i].iv_mean_a = 0.01f;
        res[i].iw_mean_a = 0.01f;
        res[i].phase_abs_peak_a = 0.12f;
        res[i].maximum_soft_consecutive_count = 0u;
        res[i].encoder_delta_counts = 0;
        res[i].reconstructed_zero_current_error_a = 0.01f + (float)i * 0.001f;
        res[i].distance_to_edge_us = cand[i].distance_to_nearest_switch_edge_us;
        res[i].center_distance_counts = 0.0f;
    }
    check_true(!electrical_offset_trigger_sweep_candidate_result_valid(&cand[0],
                                                                       &res[0]),
               "invalid baseline cannot become a valid result");
    check_true(electrical_offset_trigger_sweep_candidate_result_valid(&cand[1],
                                                                      &res[1]),
               "valid candidate passes strict zero-current criteria");
    res[1].maximum_soft_consecutive_count = 4u;
    check_true(!electrical_offset_trigger_sweep_candidate_result_valid(&cand[1],
                                                                       &res[1]),
               "soft overcurrent is recorded and rejects the candidate");
    res[1].maximum_soft_consecutive_count = 0u;
    res[1].delta_pc0_mean_counts = 2.5f;
    check_true(!electrical_offset_trigger_sweep_candidate_result_valid(&cand[1],
                                                                       &res[1]),
               "mean PC0 shift over 2 counts rejects candidate");
    res[1].delta_pc0_mean_counts = 0.5f;

    int32_t best = electrical_offset_trigger_sweep_recommend_candidate(cand, res, n);
    check_true(best > 0, "recommendation picks a valid non-baseline candidate");
    res[best].raw_hard_trip = true;
    check_true(!electrical_offset_trigger_sweep_candidate_result_valid(&cand[best],
                                                                       &res[best]),
               "hard protection immediately invalidates candidate");

    check_true(electrical_offset_trigger_sweep_classify(true, true, true, false, false) ==
                   ELECTRICAL_OFFSET_TRIGGER_SWEEP_TOO_CLOSE_TO_SWITCH_EDGE,
               "improvement with guard classifies trigger too close to edge");
    check_true(electrical_offset_trigger_sweep_classify(false, true, true, false, false) ==
                   ELECTRICAL_OFFSET_TRIGGER_SWEEP_COMMON_MODE_SETTLING_LONG,
               "long guard improvement classifies common-mode settling");
    check_true(electrical_offset_trigger_sweep_classify(false, false, true, true, false) ==
                   ELECTRICAL_OFFSET_TRIGGER_SWEEP_ACTIVE_OFFSET_INDEPENDENT,
               "all valid points shifted classifies active offset independent of trigger");
    check_true(electrical_offset_trigger_sweep_classify(false, false, false, false, false) ==
                   ELECTRICAL_OFFSET_TRIGGER_SWEEP_NO_VALID_LOW_SIDE_WINDOW,
               "no valid point is classified explicitly");
    check_true(electrical_offset_trigger_sweep_classify(false, false, true, false, true) ==
                   ELECTRICAL_OFFSET_TRIGGER_SWEEP_EVENT_CONFIG_INVALID,
               "two triggers per PWM or wrong event config is classified");
    check_true(strcmp(electrical_offset_trigger_sweep_classification_name(
                          ELECTRICAL_OFFSET_TRIGGER_SWEEP_TOO_CLOSE_TO_SWITCH_EDGE),
                      "TRIGGER_TOO_CLOSE_TO_SWITCH_EDGE") == 0,
               "trigger sweep classification name is stable");

    const uint32_t callback_cnt = 3145u;
    const uint32_t expected_trigger = cand[1].programmed_trigger_count;
    check_true(callback_cnt != expected_trigger,
               "callback TIM1_CNT is not treated as the real trigger count");

    ElectricalOffsetAlignmentProtectionState prot_state;
    ElectricalOffsetAlignmentProtectionInput prot_in =
        good_alignment_protection_input();
    ElectricalOffsetAlignmentProtectionResult pr;
    electrical_offset_alignment_protection_reset(&prot_state);
    for (uint32_t i = 0u; i < 4u; ++i) {
        prot_in.iu_a = 0.31f;
        prot_in.id_a = 0.31f;
        prot_in.phase_abs_a = 0.31f;
        pr = electrical_offset_alignment_protection_update(&prot_state,
                                                           &prot_in);
    }
    check_true(pr.trip &&
                   pr.failure ==
                       ELECTRICAL_OFFSET_FAILURE_ALIGNMENT_PHASE_SOFT_OVERCURRENT,
               "normal alignment trips on four revised soft overcurrent samples");
}

static void test_current_sensor_noise_diagnostic_math(void)
{
    int16_t pc0[128] = {0};
    int16_t pc1[128] = {0};
    uint16_t raw0[128] = {0};
    uint16_t raw1[128] = {0};
    uint32_t seq[128] = {0};
    uint16_t tim[128] = {0};
    uint32_t cb[128] = {0};
    CurrentSensorNoiseAnalysis ana;
    CurrentSensorNoiseNfaultSemantics nf;
    CurrentSensorNoiseSeqTracker tracker;
    CurrentSensorNoiseAdcIntegrity integrity;
    CurrentSensorNoiseAdmissionResult admission;
    CurrentSensorNoiseTransientResult transient;
    CurrentSensorNoiseLowpassEstimate lp;

    printf("\n== current sensor noise diagnostic ==\n");

    for (uint32_t i = 0u; i < 128u; ++i) {
        const int16_t pattern = (int16_t)((int32_t)(i % 5u) - 2);
        pc0[i] = pattern;
        pc1[i] = (int16_t)-pattern;
        raw0[i] = (uint16_t)(2000 + pc0[i]);
        raw1[i] = (uint16_t)(2000 + pc1[i]);
        seq[i] = i + 1u;
        tim[i] = (uint16_t)(100u + i);
        cb[i] = 1000u + i;
    }
    pc0[17] = 10;
    pc1[17] = 0;
    raw0[17] = 2010u;
    raw1[17] = 2000u;
    current_sensor_noise_analyze(pc0, pc1, raw0, raw1, seq, tim, cb, 0u,
                                 128u, 0.020142f, &ana);
    check_true(ana.pc0.abs_over_4_count >= 1u,
               "std=2-ish count noise can exceed 4-count peak threshold");
    check_true(ana.phase.samples_above_0p20A == 1u &&
                   ana.phase.longest_consecutive_above_0p20A == 1u,
               "single 10-count glitch is not sustained current");
    check_true(ana.worst_count == CURRENT_SENSOR_NOISE_WORST_CAPACITY &&
                   ana.worst[0].sample_index == 17u,
               "worst sample sorting keeps largest phase spike first");
    check_true(ana.worst[0].prev_delta_pc0 == pc0[16] &&
                   ana.worst[0].next_delta_pc0 == pc0[18],
               "worst sample includes previous and next deltas");

    pc0[40] = 7;
    pc0[41] = 7;
    pc0[42] = 7;
    pc1[40] = 0;
    pc1[41] = 0;
    pc1[42] = 0;
    current_sensor_noise_analyze(pc0, pc1, raw0, raw1, seq, tim, cb, 0u,
                                 128u, 0.020142f, &ana);
    check_true(ana.phase.longest_consecutive_above_0p12A >= 3u,
               "consecutive over-threshold samples are identified");

    admission = current_sensor_noise_evaluate_zero_current_admission(&ana);
    check_true(admission.base_noise_ok,
               "std=2.24-count class noise passes candidate base noise gate");

    {
        int16_t noisy0[128] = {0};
        int16_t noisy1[128] = {0};
        for (uint32_t i = 0u; i < 128u; ++i) {
            noisy0[i] = (int16_t)((i & 1u) ? 4 : -4);
            noisy1[i] = 0;
        }
        current_sensor_noise_analyze(noisy0, noisy1, NULL, NULL, NULL, NULL,
                                     NULL, 0u, 128u, 0.020142f, &ana);
        admission = current_sensor_noise_evaluate_zero_current_admission(&ana);
        check_true(!admission.base_noise_ok,
                   "std over 3 counts fails candidate admission");

        for (uint32_t i = 0u; i < 128u; ++i) {
            noisy0[i] = (int16_t)((i & 1u) ? 3 : -3);
        }
        current_sensor_noise_analyze(noisy0, noisy1, NULL, NULL, NULL, NULL,
                                     NULL, 0u, 128u, 0.020142f, &ana);
        admission = current_sensor_noise_evaluate_zero_current_admission(&ana);
        check_true(!admission.base_noise_ok,
                   "MAD over 2 counts fails candidate admission");

        memset(noisy0, 0, sizeof(noisy0));
        noisy0[127] = 14;
        current_sensor_noise_analyze(noisy0, noisy1, NULL, NULL, NULL, NULL,
                                     NULL, 0u, 128u, 0.020142f, &ana);
        admission = current_sensor_noise_evaluate_zero_current_admission(&ana);
        check_true(!admission.high_tail_ok,
                   "raw abs p99.9 over 12 counts fails candidate admission");

        memset(&ana, 0, sizeof(ana));
        ana.pc0.sample_count = 4096u;
        ana.pc1.sample_count = 4096u;
        ana.pc0.standard_deviation = 2.3f;
        ana.pc1.standard_deviation = 2.1f;
        ana.pc0.mad = 1.0f;
        ana.pc1.mad = 1.0f;
        ana.pc0.min = -9;
        ana.pc0.max = 14;
        ana.pc1.min = -9;
        ana.pc1.max = 11;
        ana.pc0.abs_percentile_99_9 = 10.0f;
        ana.pc1.abs_percentile_99_9 = 9.0f;
        admission = current_sensor_noise_evaluate_zero_current_admission(&ana);
        check_true(admission.high_tail_ok &&
                       admission.high_tail_pc0_pass &&
                       admission.high_tail_pc1_pass,
                   "high_tail uses raw abs p99.9, not isolated min/max");
    }

    {
        float phase_abs[32] = {0.0f};
        for (uint32_t i = 0u; i < 23u; ++i) {
            phase_abs[i] = (i & 1u) ? 0.201f : 0.0f;
        }
        transient = current_sensor_noise_classify_phase_abs(phase_abs, 32u,
                                                            0.20f, 4u, 0.35f);
        check_true(!transient.sustained_soft_fault &&
                       !transient.instantaneous_hard_fault,
                   "23 non-consecutive 0.20A spikes are not sustained current");
        phase_abs[4] = 0.21f;
        phase_abs[5] = 0.22f;
        phase_abs[6] = 0.23f;
        phase_abs[7] = 0.24f;
        transient = current_sensor_noise_classify_phase_abs(phase_abs, 32u,
                                                            0.20f, 4u, 0.35f);
        check_true(transient.sustained_soft_fault,
                   "four consecutive 0.20A samples are sustained abnormal current");
        phase_abs[20] = 0.36f;
        transient = current_sensor_noise_classify_phase_abs(phase_abs, 32u,
                                                            0.20f, 4u, 0.35f);
        check_true(transient.instantaneous_hard_fault,
                   "single 0.35A sample is an instantaneous hard abnormal candidate");
    }

    current_sensor_noise_seq_tracker_init(&tracker);
    for (uint32_t i = 0u; i < 6912u; ++i) {
        current_sensor_noise_seq_tracker_observe(&tracker, 100000u + i);
    }
    current_sensor_noise_seq_tracker_to_integrity(&tracker, 6912u, 6144u,
                                                  768u, &integrity);
    check_true(integrity.missed_adc_seq == 0u &&
                   integrity.duplicate_adc_seq == 0u &&
                   nearf(integrity.adc_sync_rate, 1.0f, 0.001f),
               "discard and analysis windows form one continuous ADC sequence");
    check_true(integrity.diagnostic_analysis_sample_count == 6144u &&
                   integrity.diagnostic_discard_sample_count == 768u,
               "diagnostic window separates discard and analysis sample counts");
    {
        const uint32_t producer_span = integrity.diagnostic_adc_seq_span;
        const uint32_t consumer_samples_read = 37u;
        const uint32_t consumer_skipped =
            (producer_span > consumer_samples_read)
                ? (producer_span - consumer_samples_read)
                : 0u;
        const bool adc_publish_sequence_valid =
            (integrity.missed_adc_seq == 0u) &&
            (integrity.duplicate_adc_seq == 0u);
        check_true(adc_publish_sequence_valid && consumer_skipped > 0u,
                   "consumer skipped snapshots do not create a producer ADC gap");
    }

    current_sensor_noise_seq_tracker_init(&tracker);
    current_sensor_noise_seq_tracker_observe(&tracker, 0xFFFFFFFEu);
    current_sensor_noise_seq_tracker_observe(&tracker, 0xFFFFFFFFu);
    current_sensor_noise_seq_tracker_observe(&tracker, 0u);
    current_sensor_noise_seq_tracker_observe(&tracker, 1u);
    current_sensor_noise_seq_tracker_to_integrity(&tracker, 4u, 4u, 0u,
                                                  &integrity);
    check_true(integrity.missed_adc_seq == 0u &&
                   integrity.duplicate_adc_seq == 0u,
               "uint32 ADC seq wrap does not create a false gap");

    current_sensor_noise_seq_tracker_init(&tracker);
    current_sensor_noise_seq_tracker_observe(&tracker, 1u);
    current_sensor_noise_seq_tracker_observe(&tracker, 2u);
    current_sensor_noise_seq_tracker_observe(&tracker, 4u);
    current_sensor_noise_seq_tracker_observe(&tracker, 4u);
    current_sensor_noise_seq_tracker_to_integrity(&tracker, 4u, 4u, 0u,
                                                  &integrity);
    check_true(integrity.missed_adc_seq == 1u &&
                   integrity.duplicate_adc_seq == 1u,
               "real missing and duplicate ADC snapshots are detected");

    {
        CurrentSensorNoiseOnlineAccumulator online;
        CurrentSensorNoiseAnalysis online_ana;
        int16_t small_pc0[32] = {0};
        int16_t small_pc1[32] = {0};
        uint16_t small_raw0[32] = {0};
        uint16_t small_raw1[32] = {0};
        uint32_t small_seq[32] = {0};
        uint16_t small_tim[32] = {0};
        uint32_t small_cb[32] = {0};
        int16_t prev0 = 0;
        int16_t prev1 = 0;

        current_sensor_noise_online_reset(&online);
        for (uint32_t i = 0u; i < 32u; ++i) {
            int16_t d0 = (int16_t)((int32_t)(i % 7u) - 3);
            int16_t d1 = (int16_t)((int32_t)(i % 5u) - 2);
            if (i == 11u) {
                d0 = 14;
            }
            small_pc0[i] = d0;
            small_pc1[i] = d1;
            small_raw0[i] = (uint16_t)(2100 + d0);
            small_raw1[i] = (uint16_t)(2110 + d1);
            small_seq[i] = 5000u + i;
            small_tim[i] = (uint16_t)(300u + i);
            small_cb[i] = 7000u + i;
            current_sensor_noise_online_push(&online, i, small_seq[i],
                                             small_raw0[i], small_raw1[i],
                                             d0, d1, prev0, prev1,
                                             small_tim[i], 0u, small_cb[i],
                                             0.020142f);
            prev0 = d0;
            prev1 = d1;
        }
        current_sensor_noise_online_finalize(&online, &online_ana);
        current_sensor_noise_analyze(small_pc0, small_pc1,
                                     small_raw0, small_raw1,
                                     small_seq, small_tim, small_cb,
                                     0u, 32u, 0.020142f, &ana);
        check_true(online_ana.pc0.sample_count == ana.pc0.sample_count &&
                       nearf(online_ana.pc0.mean, ana.pc0.mean, 0.001f) &&
                       nearf(online_ana.pc0.standard_deviation,
                             ana.pc0.standard_deviation, 0.001f),
                   "online count statistics match offline analysis");
        check_true(online_ana.phase.longest_consecutive_above_0p20A ==
                       ana.phase.longest_consecutive_above_0p20A,
                   "online consecutive phase-current statistics match offline analysis");
        check_true(online_ana.worst_count <= CURRENT_SENSOR_NOISE_WORST_CAPACITY &&
                       online_ana.worst[0].sample_index == 11u &&
                       online_ana.worst[0].next_delta_pc0 == small_pc0[12],
                   "online worst-16 buffer stays bounded and captures next sample");

        current_sensor_noise_online_reset(&online);
        prev0 = 0;
        prev1 = 0;
        for (uint32_t i = 0u; i < 40u; ++i) {
            const int16_t d0 = (i & 1u) ? 45 : -45;
            const int16_t d1 = 0;
            current_sensor_noise_online_push(&online, i, 9000u + i,
                                             (uint16_t)(2200 + d0), 2200u,
                                             d0, d1, prev0, prev1,
                                             (uint16_t)i, 0u, 10000u + i,
                                             0.020142f);
            prev0 = d0;
            prev1 = d1;
        }
        current_sensor_noise_online_finalize(&online, &online_ana);
        check_true(online_ana.pc0.min == -45 &&
                       online_ana.pc0.max == 45 &&
                       online_ana.pc0.percentile_99_9 <=
                           (float)CURRENT_SENSOR_NOISE_DELTA_HIST_MAX &&
                       online_ana.worst_count == CURRENT_SENSOR_NOISE_WORST_CAPACITY,
                   "online histogram clamps out-of-range deltas and worst buffer stays capped");
    }

    lp = current_sensor_noise_estimate_one_pole_lowpass(20000.0f, 1000.0f,
                                                        0.045f, 0.10f);
    check_true(lp.rms_reduction_ratio > 0.0f &&
                   lp.rms_reduction_ratio < 1.0f &&
                   lp.snr_0p10a_after_filter > 2.22f,
               "offline one-pole low-pass estimate improves 0.10A SNR");

    {
        int16_t values[5] = {1, 3, 5, 7, 9};
        check_true(nearf(current_sensor_noise_percentile_i16(values, 5u, 50.0f),
                         5.0f, EPS),
                   "percentile median is correct");
        check_true(nearf(current_sensor_noise_percentile_i16(values, 5u, 90.0f),
                         8.2f, 0.001f),
                   "interpolated percentile is correct");
        check_true(nearf(current_sensor_noise_median_i16(values, 5u), 5.0f, EPS),
                   "median helper is correct");
        check_true(nearf(current_sensor_noise_mad_i16(values, 5u), 2.0f, EPS),
                   "MAD helper is correct");
    }

    check_true(current_sensor_noise_should_skip_power_for_low_gain(10.0f) &&
                   current_sensor_noise_should_skip_power_for_low_gain(20.0f) &&
                   !current_sensor_noise_should_skip_power_for_low_gain(40.0f),
               "low-gain power tests are skipped for resolution");

    nf = current_sensor_noise_nfault_from_raw(false);
    check_true(!nf.raw_pin_high && nf.asserted && !nf.ok,
               "nFAULT raw low means asserted");
    nf = current_sensor_noise_nfault_from_raw(true);
    check_true(nf.raw_pin_high && !nf.asserted && nf.ok,
               "nFAULT raw high means ok");
    check_true(nf.ok,
               "nFAULT while EN_GATE is enabled is the runtime pass/fail input");
}

static CurrentSensorAdmissionFastInput admission_input(uint32_t seq,
                                                       uint16_t raw0,
                                                       uint16_t raw1)
{
    CurrentSensorAdmissionFastInput in;
    memset(&in, 0, sizeof(in));
    in.adc_seq = seq;
    in.raw_pc0 = raw0;
    in.raw_pc1 = raw1;
    in.snapshot_valid = true;
    in.nfault_raw_high = true;
    return in;
}

static void admission_feed(CurrentSensorAdmission *adm,
                           uint32_t *seq,
                           uint16_t raw0,
                           uint16_t raw1)
{
    CurrentSensorAdmissionFastInput in = admission_input((*seq)++, raw0, raw1);
    current_sensor_admission_fast_isr(adm, &in);
}

static CurrentSensorAdmissionConfig admission_small_config(void)
{
    CurrentSensorAdmissionConfig cfg = current_sensor_admission_default_config();
    cfg.dc_cal_discard_samples = 2u;
    cfg.dc_cal_collect_samples = 4u;
    cfg.post_discard_samples = 3u;
    cfg.post_collect_samples = 16u;
    return cfg;
}

static bool admission_run_small_pattern_with_std_limit(
    const int16_t *pc0,
    const int16_t *pc1,
    uint32_t count,
    float std_limit_counts,
    bool recenter_live_zero_offset,
    uint16_t above_10_limit,
    CurrentSensorAdmission *adm)
{
    CurrentSensorAdmissionConfig cfg = admission_small_config();
    cfg.std_limit_counts = std_limit_counts;
    cfg.recenter_live_zero_offset = recenter_live_zero_offset;
    cfg.above_10_limit = above_10_limit;
    uint32_t seq = 1u;
    current_sensor_admission_init(adm, &cfg);
    if (!current_sensor_admission_request_start(adm)) {
        return false;
    }
    for (uint32_t i = 0u; i < 5u; ++i) {
        (void)current_sensor_admission_service_main(adm);
    }
    if (!current_sensor_admission_ack_dc_cal_enabled(adm)) {
        return false;
    }
    admission_feed(adm, &seq, 1960u, 1970u);
    admission_feed(adm, &seq, 1961u, 1971u);
    for (uint32_t i = 0u; i < cfg.dc_cal_collect_samples; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    if (adm->state != CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK) {
        return false;
    }
    if (!current_sensor_admission_service_main(adm).request_disable_dc_cal) {
        return false;
    }
    if (!current_sensor_admission_ack_dc_cal_disabled(adm)) {
        return false;
    }
    for (uint32_t i = 0u; i < cfg.post_discard_samples; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    for (uint32_t i = 0u; i < count; ++i) {
        admission_feed(adm, &seq,
                       (uint16_t)(1963 + pc0[i]),
                       (uint16_t)(1974 + pc1[i]));
    }
    current_sensor_admission_finalize(adm);
    return true;
}

static bool admission_run_small_pattern(const int16_t *pc0,
                                        const int16_t *pc1,
                                        uint32_t count,
                                        CurrentSensorAdmission *adm)
{
    return admission_run_small_pattern_with_std_limit(
        pc0,
        pc1,
        count,
        current_sensor_admission_default_config().std_limit_counts,
        false,
        current_sensor_admission_default_config().above_10_limit,
        adm);
}

static bool admission_run_small_pattern_with_post_ack_wait(uint32_t wait_count,
                                                           CurrentSensorAdmission *adm)
{
    CurrentSensorAdmissionConfig cfg = admission_small_config();
    uint32_t seq = 1u;
    current_sensor_admission_init(adm, &cfg);
    if (!current_sensor_admission_request_start(adm)) {
        return false;
    }
    if (!current_sensor_admission_ack_dc_cal_enabled(adm)) {
        return false;
    }
    for (uint32_t i = 0u; i < cfg.dc_cal_discard_samples; ++i) {
        admission_feed(adm, &seq, 1960u, 1970u);
    }
    for (uint32_t i = 0u; i < cfg.dc_cal_collect_samples; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    if (adm->state != CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK) {
        return false;
    }
    const uint32_t dc_last_seq = adm->result.dc_cal_collect_last_seq;
    for (uint32_t i = 0u; i < wait_count; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    if (!current_sensor_admission_ack_dc_cal_disabled(adm)) {
        return false;
    }
    for (uint32_t i = 0u; i < cfg.post_discard_samples; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    for (uint32_t i = 0u; i < cfg.post_collect_samples; ++i) {
        admission_feed(adm, &seq, 1963u, 1974u);
    }
    current_sensor_admission_finalize(adm);
    CurrentSensorAdmissionResult res = current_sensor_admission_get_result(adm);
    const uint32_t scheduled = (uint32_t)cfg.dc_cal_discard_samples +
                               (uint32_t)cfg.dc_cal_collect_samples +
                               (uint32_t)cfg.post_discard_samples +
                               (uint32_t)cfg.post_collect_samples;
    return adm->state == CURRENT_SENSOR_ADMISSION_COMPLETE &&
           res.producer_gap_count == 0u &&
           res.producer_duplicate_count == 0u &&
           res.wait_post_cal_ack_snapshot_count == wait_count &&
           res.transition_wait_snapshot_count == wait_count &&
           res.observed_snapshot_count == scheduled + wait_count &&
           res.scheduled_snapshot_count == scheduled &&
           res.analysis_snapshot_count ==
               ((uint32_t)cfg.dc_cal_collect_samples +
                (uint32_t)cfg.post_collect_samples) &&
           res.discard_snapshot_count ==
               ((uint32_t)cfg.dc_cal_discard_samples +
                (uint32_t)cfg.post_discard_samples) &&
           res.post_cal_discard_first_seq == dc_last_seq + wait_count + 1u &&
           res.admission_pass;
}

static void test_current_sensor_admission_module(void)
{
    CurrentSensorAdmission adm;
    CurrentSensorAdmissionResult res;
    int16_t pc0[16] = {0};
    int16_t pc1[16] = {0};

    printf("\n== current sensor lightweight admission ==\n");

    for (uint32_t i = 0u; i < 16u; ++i) {
        pc0[i] = (int16_t)((int32_t)(i % 5u) - 2);
        pc1[i] = (int16_t)-pc0[i];
    }
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "admission small pattern runs");
    res = current_sensor_admission_get_result(&adm);
    check_true(adm.state == CURRENT_SENSOR_ADMISSION_COMPLETE &&
                   res.dc_cal_samples == 4u &&
                   res.post_samples == 16u &&
                   res.dc_cal_discard_count == 2u &&
                   res.post_cal_discard_count == 3u &&
                   res.scheduled_snapshot_count == 25u &&
                   res.observed_snapshot_count == 25u &&
                   res.producer_seq_span == res.observed_snapshot_count &&
                   res.admission_pass,
               "discard windows excluded and collect windows pass");
    check_true(res.dc_cal_offset_pc0 == 1963u &&
                   res.dc_cal_offset_pc1 == 1974u,
               "DC_CAL samples produce correct offsets");
    check_true(!current_sensor_admission_request_start(&adm),
               "completion locks out automatic restart");

    {
        CurrentSensorAdmissionConfig cfg = current_sensor_admission_default_config();
        uint32_t seq = 100u;
        current_sensor_admission_init(&adm, &cfg);
        check_true(current_sensor_admission_request_start(&adm),
                   "default admission can be requested once");
        check_true(current_sensor_admission_ack_dc_cal_enabled(&adm),
                   "main acknowledges DC_CAL enable");
        for (uint32_t i = 0u; i < 128u + 256u; ++i) {
            for (uint32_t service = 0u; service < 3u; ++service) {
                (void)current_sensor_admission_service_main(&adm);
            }
            admission_feed(&adm, &seq, 1963u, 1974u);
        }
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_WAIT_POST_CAL_ACK &&
                       adm.dc.count == 256u &&
                       adm.result.dc_cal_discard_count == 128u &&
                       adm.result.observed_snapshot_count == 384u,
                   "main frequency does not affect DC_CAL sample count");
    }

    check_true(admission_run_small_pattern_with_post_ack_wait(1u, &adm),
               "WAIT_POST_CAL_ACK one snapshot updates seq without gap");
    check_true(admission_run_small_pattern_with_post_ack_wait(10u, &adm),
               "WAIT_POST_CAL_ACK ten snapshots update seq without formal samples");
    check_true(admission_run_small_pattern_with_post_ack_wait(100u, &adm),
               "WAIT_POST_CAL_ACK hundred snapshots do not create producer gap");

    {
        CurrentSensorAdmissionConfig cfg = current_sensor_admission_default_config();
        uint32_t seq = 500u;
        current_sensor_admission_init(&adm, &cfg);
        check_true(current_sensor_admission_request_start(&adm),
                   "default admission full run requested");
        check_true(current_sensor_admission_ack_dc_cal_enabled(&adm),
                   "default admission DC_CAL enable acknowledged");
        for (uint32_t i = 0u; i < cfg.dc_cal_discard_samples + cfg.dc_cal_collect_samples; ++i) {
            admission_feed(&adm, &seq, 1963u, 1974u);
        }
        check_true(current_sensor_admission_ack_dc_cal_disabled(&adm),
                   "default admission DC_CAL disable acknowledged");
        for (uint32_t i = 0u; i < cfg.post_discard_samples + cfg.post_collect_samples; ++i) {
            admission_feed(&adm, &seq, 1963u, 1974u);
        }
        current_sensor_admission_finalize(&adm);
        res = current_sensor_admission_get_result(&adm);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_COMPLETE &&
                       res.scheduled_snapshot_count == 1152u &&
                       res.observed_snapshot_count == 1152u &&
                       res.dc_cal_discard_count == 128u &&
                       res.dc_cal_samples == 256u &&
                       res.post_cal_discard_count == 256u &&
                       res.post_samples == 512u &&
                       res.admission_pass,
                   "default admission consumes exactly 1152 snapshots");
        CurrentSensorAdmissionFastInput late = admission_input(seq++, 100u, 100u);
        late.nfault_raw_high = false;
        current_sensor_admission_fast_isr(&adm, &late);
        current_sensor_admission_finalize(&adm);
        res = current_sensor_admission_get_result(&adm);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_COMPLETE &&
                       res.nfault_runtime_asserted_count == 0u &&
                       res.admission_pass,
                   "gate-off nFAULT after completion is ignored by runtime admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        for (uint32_t i = 0u; i < cfg.dc_cal_discard_samples + cfg.dc_cal_collect_samples; ++i) {
            admission_feed(&adm, &seq, 1963u, 1974u);
        }
        for (uint32_t i = 0u; i < 2001u; ++i) {
            admission_feed(&adm, &seq, 1963u, 1974u);
        }
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_POST_CAL_ACK_TIMEOUT) &&
                       adm.result.producer_gap_count == 0u,
                   "WAIT_POST_CAL_ACK timeout fails without producer gap");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        admission_feed(&adm, &seq, 1963u, 1974u);
        CurrentSensorAdmissionFastInput in = admission_input(seq + 1u, 1963u, 1974u);
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_ADC_SEQ_GAP),
                   "producer gap immediately fails admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        admission_feed(&adm, &seq, 1963u, 1974u);
        CurrentSensorAdmissionFastInput in = admission_input(seq - 1u, 1963u, 1974u);
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_ADC_DUPLICATE),
                   "producer duplicate immediately fails admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        CurrentSensorAdmissionFastInput in = admission_input(seq++, 1963u, 1974u);
        in.adc_true_unpaired = true;
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       adm.result.runtime_true_unpaired_count == 1u,
                   "true unpaired immediately fails admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        CurrentSensorAdmissionFastInput in = admission_input(seq++, 1963u, 1974u);
        in.adc_torn = true;
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_TORN),
                   "torn snapshot immediately fails admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        CurrentSensorAdmissionFastInput in = admission_input(seq++, 1963u, 1974u);
        in.adc_generation_mismatch = true;
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_GENERATION_MISMATCH),
                   "generation mismatch immediately fails admission");
    }

    {
        CurrentSensorAdmissionConfig cfg = admission_small_config();
        uint32_t seq = 1u;
        current_sensor_admission_init(&adm, &cfg);
        (void)current_sensor_admission_request_start(&adm);
        (void)current_sensor_admission_ack_dc_cal_enabled(&adm);
        CurrentSensorAdmissionFastInput in = admission_input(seq++, 1963u, 1974u);
        in.nfault_raw_high = false;
        current_sensor_admission_fast_isr(&adm, &in);
        check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                       adm.result.nfault_runtime_asserted_count == 1u,
                   "nFAULT runtime low immediately fails admission");
    }

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    pc0[3] = 10;
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "single phase 10-count sample can complete");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(res.admission_pass &&
                   res.longest_above_10_counts == 0u &&
                   res.phase_metric_max_counts == 10u,
               "single phase 10 counts is allowed");

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    pc0[2] = 11;
    pc0[3] = 11;
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "two phase >10-count samples complete collection");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass &&
                   !res.consecutive_pass,
               "two consecutive phase >10 counts fails finalize");

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    for (uint32_t i = 0u; i < 3u; ++i) {
        pc0[i] = 6;
        pc1[i] = 5;
    }
    check_true(admission_run_small_pattern_with_std_limit(
                   pc0, pc1, 16u, 3.5f, true, 3u, &adm),
               "calibration three-sample tail collection completes");
    res = current_sensor_admission_get_result(&adm);
    check_true(res.admission_pass && res.consecutive_pass &&
                   res.longest_above_10_counts == 3u,
               "calibration profile permits at most three >10-count samples");

    pc0[3] = 6;
    pc1[3] = 5;
    check_true(admission_run_small_pattern_with_std_limit(
                   pc0, pc1, 16u, 3.5f, true, 3u, &adm),
               "calibration four-sample tail collection completes");
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass && !res.consecutive_pass &&
                   res.longest_above_10_counts == 4u,
               "calibration profile rejects four consecutive >10-count samples");

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    for (uint32_t i = 0u; i < 5u; ++i) { pc0[i] = 9; }
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "five phase >8-count samples complete collection");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass &&
                   !res.consecutive_pass,
               "five consecutive phase >8 counts fails finalize");

    memset(pc1, 0, sizeof(pc1));
    for (uint32_t i = 0u; i < 9u; ++i) {
        pc0[i] = 7;
    }
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "nine-style phase >6-count test runs with compact window");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass &&
                   !res.consecutive_pass,
               "phase >6 count run over configured limit fails");

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    pc0[0] = 21;
    (void)admission_run_small_pattern(pc0, pc1, 16u, &adm);
    check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                   (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_RAW_OUTLIER),
               "raw delta >20 counts immediately fails");

    memset(pc0, 0, sizeof(pc0));
    memset(pc1, 0, sizeof(pc1));
    pc0[0] = 9;
    pc1[0] = 9;
    (void)admission_run_small_pattern(pc0, pc1, 16u, &adm);
    check_true(adm.state == CURRENT_SENSOR_ADMISSION_FAIL &&
                   (adm.result.fail_flags & CURRENT_SENSOR_ADMISSION_FAIL_PHASE_OUTLIER),
               "reconstructed phase metric >17 counts immediately fails");

    for (uint32_t i = 0u; i < 16u; ++i) {
        pc0[i] = (i & 1u) ? 4 : -4;
        pc1[i] = 0;
    }
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "std over limit collection completes");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass && !res.std_pass,
               "std >3 counts fails admission");

    for (uint32_t i = 0u; i < 16u; ++i) {
        const int16_t magnitude = (i < 4u) ? 4 : 3;
        pc0[i] = (i & 1u) ? magnitude : (int16_t)-magnitude;
        pc1[i] = 0;
    }
    check_true(admission_run_small_pattern_with_std_limit(
                   pc0, pc1, 16u, 3.5f, false, 1u, &adm),
               "calibration noise profile collection completes");
    res = current_sensor_admission_get_result(&adm);
    check_true(res.admission_pass && res.std_pass &&
                   res.pc0_std_counts > 3.0f &&
                   res.pc0_std_counts < 3.5f,
               "calibration profile accepts bounded 3-to-3.5 count noise");
    check_true(admission_run_small_pattern_with_std_limit(
                   pc0, pc1, 16u, 3.0f, false, 1u, &adm),
               "default noise profile collection completes");
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass && !res.std_pass,
               "default profile still rejects noise above 3 counts");

    for (uint32_t i = 0u; i < 16u; ++i) {
        pc0[i] = 2;
        pc1[i] = 0;
    }
    check_true(admission_run_small_pattern_with_std_limit(
                   pc0, pc1, 16u, 3.5f, true, 3u, &adm),
               "live-zero recenter collection completes");
    res = current_sensor_admission_get_result(&adm);
    check_true(res.admission_pass && res.mean_pass &&
                   res.dc_cal_offset_pc0 == 1963u &&
                   res.live_zero_offset_pc0 == 1965u &&
                   res.live_zero_shift_pc0_counts == 2,
               "live-zero recenter removes bounded post-DC_CAL offset shift");
    check_true(admission_run_small_pattern(pc0, pc1, 16u, &adm),
               "mean over limit collection completes");
    current_sensor_admission_finalize(&adm);
    res = current_sensor_admission_get_result(&adm);
    check_true(!res.admission_pass && !res.mean_pass,
               "mean over limit fails admission");

    check_true(current_sensor_admission_state_name(CURRENT_SENSOR_ADMISSION_COMPLETE)[0] == 'C',
               "admission state names are available for logs");

    {
        CurrentSensorAdmissionResult good = {0};
        good.admission_pass = true;
        good.mean_pass = true;
        good.std_pass = true;
        good.consecutive_pass = true;
        good.hard_outlier_pass = true;
        CurrentSensorAdmissionPreflightVerdict v =
            current_sensor_admission_evaluate_preflight(
                &good,
                1555u,
                2851u,
                168000000u,
                true,
                true,
                true,
                true);
        check_true(!v.target_5us_met &&
                       v.deadline_pass &&
                       v.functional_pass &&
                       v.preflight_pass,
                   "9.256us admission target miss does not block 16.97us callback preflight");
        v = current_sensor_admission_evaluate_preflight(
            &good,
            200u,
            8400u,
            168000000u,
            true,
            true,
            true,
            true);
        check_true(!v.deadline_pass && !v.preflight_pass,
                   "50us callback deadline miss blocks preflight");
        v = current_sensor_admission_evaluate_preflight(
            &good,
            200u,
            2000u,
            168000000u,
            false,
            true,
            true,
            true);
        check_true(!v.functional_pass && !v.preflight_pass,
                   "functional sample failure blocks handoff");
        v = current_sensor_admission_evaluate_preflight(
            &good,
            200u,
            2000u,
            168000000u,
            true,
            false,
            true,
            true);
        check_true(!v.deadline_pass && !v.preflight_pass,
                   "producer or ADC pipeline failure blocks preflight");
    }
}

int main(void)
{
    test_velocity_overspeed_evidence();
    test_velocity_low_speed_coupling_and_run_guard();
    test_math_and_pi_signs();
    test_limit_antiwindup_and_reset();
    test_fixed_rotor_protections();
    test_fixed_rotor_fast_isr_contract();
    test_fixed_rotor_full_sequence_without_plant();
    test_discrete_rl_current_loop();
    test_rotating_offset_and_angle_guards();
    test_rotating_fast_path_and_limits();
    test_rotating_enable_zero_diagnostic_mode();
    test_rotating_block_integrator_admission();
    test_rotating_direction_sequence();
    test_rotating_low_current_tracking_sequence();
    test_electrical_offset_calibration_math();
    test_electrical_offset_state_flow();
    test_electrical_offset_alignment_protection();
    test_electrical_offset_timing_and_formatting();
    test_electrical_offset_pwm_zero_diagnostic_helpers();
    test_electrical_offset_trigger_sweep_helpers();
    test_current_sensor_noise_diagnostic_math();
    test_current_sensor_admission_module();

    if (g_failures != 0) {
        printf("\ncurrent_controller_test failed: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("\ncurrent_controller_test passed.\n");
    return 0;
}
