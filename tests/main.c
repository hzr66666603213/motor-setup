#include "control/current_controller.h"
#include "control/velocity_controller.h"
#include "foc/foc_math.h"
#include "foc/svpwm.h"
#include "sim/foc_sim.h"

#include <math.h>
#include <stdio.h>

/*
 * tests/main.c
 *
 * PC 端 FOC 仿真单元测试入口。
 *
 * 目标：
 * - 不依赖 STM32 HAL，不包含任何 stm32f4xx_hal.h。
 * - 使用普通 gcc 编译运行。
 * - 直接测试 Clarke/Park/SVPWM/current_controller。
 * - 通过 foc_sim_step_wrapper() 测试完整 FOC 仿真链路。
 * - 打印 id/iq、vd/vq、duty_u/v/w，方便和 Simulink 波形对照。
 * - 覆盖 vbus=0、角度越界、电流阶跃等边界情况。
 */

#define TEST_EPSILON_F 1.0e-4f

static int g_failures = 0;

static int is_duty_valid(float duty)
{
    return isfinite(duty) && (duty >= 0.0f) && (duty <= 1.0f);
}

static int is_near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

static void check_true(int condition, const char *name)
{
    if (!condition) {
        printf("[FAIL] %s\n", name);
        g_failures++;
    } else {
        printf("[ OK ] %s\n", name);
    }
}

static void print_step(const char *name,
                       int status,
                       float id,
                       float iq,
                       float vd,
                       float vq,
                       float duty_u,
                       float duty_v,
                       float duty_w)
{
    printf("%s: status=%d id=% .6f A iq=% .6f A vd=% .6f V vq=% .6f V duty=(%.6f, %.6f, %.6f)\n",
           name,
           status,
           id,
           iq,
           vd,
           vq,
           duty_u,
           duty_v,
           duty_w);
}

static void test_math_blocks(void)
{
    float alpha = 0.0f;
    float beta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float v_alpha = 0.0f;
    float v_beta = 0.0f;
    SvpwmDuty duty;
    CurrentController cc;
    float vd = 0.0f;
    float vq = 0.0f;

    printf("\n== direct math/controller blocks ==\n");

    foc_clarke(1.0f, -0.5f, -0.5f, &alpha, &beta);
    check_true(is_near(alpha, 1.0f, TEST_EPSILON_F), "Clarke alpha follows ia");
    check_true(is_near(beta, 0.0f, TEST_EPSILON_F), "Clarke balanced beta is zero");

    foc_park(alpha, beta, 0.0f, &id, &iq);
    check_true(is_near(id, 1.0f, TEST_EPSILON_F), "Park id at zero angle");
    check_true(is_near(iq, 0.0f, TEST_EPSILON_F), "Park iq at zero angle");

    foc_inv_park(1.0f, 0.0f, 0.0f, &v_alpha, &v_beta);
    check_true(is_near(v_alpha, 1.0f, TEST_EPSILON_F), "inverse Park alpha at zero angle");
    check_true(is_near(v_beta, 0.0f, TEST_EPSILON_F), "inverse Park beta at zero angle");

    duty = svpwm_generate(2.0f, 1.0f, 24.0f);
    check_true(is_duty_valid(duty.duty_a) &&
                   is_duty_valid(duty.duty_b) &&
                   is_duty_valid(duty.duty_c),
               "SVPWM duty range");
    printf("svpwm: duty=(%.6f, %.6f, %.6f)\n", duty.duty_a, duty.duty_b, duty.duty_c);

    current_controller_init(&cc, 0.05f, 100.0f, 3.0f);
    current_controller_update(&cc,
                              0.0f,
                              0.5f,
                              0.1f,
                              -0.2f,
                              12.0f,
                              0.00005f,
                              &vd,
                              &vq);
    check_true(isfinite(vd) && isfinite(vq), "current_controller finite output");
    check_true((sqrtf(vd * vd + vq * vq) <= 3.0001f), "current_controller voltage limit");
    printf("current_controller: vd=% .6f V vq=% .6f V\n", vd, vq);
}

static int run_foc_sim_case(const char *name,
                            float ia,
                            float ib,
                            float ic,
                            float mechanical_angle,
                            float mechanical_velocity,
                            float vbus,
                            float id_target,
                            float iq_target,
                            float dt,
                            double pole_pairs,
                            float encoder_offset,
                            int expected_status)
{
    double id = 0.0;
    double iq = 0.0;
    double vd = 0.0;
    double vq = 0.0;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double duty_u = 0.0;
    double duty_v = 0.0;
    double duty_w = 0.0;

    const int status = foc_sim_step_wrapper(ia,
                                            ib,
                                            ic,
                                            mechanical_angle,
                                            mechanical_velocity,
                                            vbus,
                                            id_target,
                                            iq_target,
                                            dt,
                                            pole_pairs,
                                            encoder_offset,
                                            &id,
                                            &iq,
                                            &vd,
                                            &vq,
                                            &v_alpha,
                                            &v_beta,
                                            &duty_u,
                                            &duty_v,
                                            &duty_w);

    print_step(name,
               status,
               (float)id,
               (float)iq,
               (float)vd,
               (float)vq,
               (float)duty_u,
               (float)duty_v,
               (float)duty_w);

    check_true(status == expected_status, "foc_sim_step_wrapper expected status");
    check_true(isfinite(id) &&
                   isfinite(iq) &&
                   isfinite(vd) &&
                   isfinite(vq) &&
                   isfinite(v_alpha) &&
                   isfinite(v_beta),
               "foc_sim_step_wrapper finite output");
    check_true(is_duty_valid(duty_u) &&
                   is_duty_valid(duty_v) &&
                   is_duty_valid(duty_w),
               "foc_sim_step_wrapper duty range");

    return status;
}

static void test_foc_sim_nominal_and_edges(void)
{
    printf("\n== foc_sim_step_wrapper nominal and edge cases ==\n");

    (void)run_foc_sim_case("nominal",
                           0.2f,
                           -0.1f,
                           -0.1f,
                           0.3f,
                           1.0f,
                           12.0f,
                           0.0f,
                           0.5f,
                           0.00005f,
                           7.0,
                           0.0f,
                           0);

    /*
     * 角度越界测试：
     * mechanical_angle 可以远大于 2*pi 或为负值，内部会归一化电角度。
     */
    (void)run_foc_sim_case("angle_overflow_positive",
                           0.2f,
                           -0.1f,
                           -0.1f,
                           1000.0f,
                           2.0f,
                           12.0f,
                           0.0f,
                           0.4f,
                           0.00005f,
                           7.0,
                           0.2f,
                           0);

    (void)run_foc_sim_case("angle_overflow_negative",
                           0.2f,
                           -0.1f,
                           -0.1f,
                           -1000.0f,
                           -2.0f,
                           12.0f,
                           0.0f,
                           -0.4f,
                           0.00005f,
                           7.0,
                           -0.2f,
                           0);

    /*
     * vbus=0 时不允许继续做真实调制。
     * 仿真入口返回 -2，并输出 50% duty，便于模型保持安全状态。
     */
    (void)run_foc_sim_case("vbus_zero",
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.00005f,
                           7.0,
                           0.0f,
                           -2);

    (void)run_foc_sim_case("invalid_pole_pairs",
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           12.0f,
                           0.0f,
                           0.0f,
                           0.00005f,
                           0.0,
                           0.0f,
                           -2);
}

static void test_current_step_response(void)
{
    double id = 0.0;
    double iq = 0.0;
    double vd = 0.0;
    double vq = 0.0;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double duty_u = 0.0;
    double duty_v = 0.0;
    double duty_w = 0.0;

    printf("\n== current step response ==\n");

    foc_sim_reset();

    for (int i = 0; i < 40; ++i) {
        const float iq_target = (i < 20) ? 0.0f : 1.0f;
        const int status = foc_sim_step_wrapper(0.05,
                                                -0.02,
                                                -0.03,
                                                0.01 * (double)i,
                                                1.0,
                                                12.0,
                                                0.0,
                                                iq_target,
                                                0.00005,
                                                7.0,
                                                0.0,
                                                &id,
                                                &iq,
                                                &vd,
                                                &vq,
                                                &v_alpha,
                                                &v_beta,
                                                &duty_u,
                                                &duty_v,
                                                &duty_w);

        if ((i == 0) || (i == 19) || (i == 20) || (i == 39)) {
            print_step("current_step",
                       status,
                       (float)id,
                       (float)iq,
                       (float)vd,
                       (float)vq,
                       (float)duty_u,
                       (float)duty_v,
                       (float)duty_w);
        }

        check_true(status == 0, "current step status");
        check_true(is_duty_valid(duty_u) &&
                       is_duty_valid(duty_v) &&
                       is_duty_valid(duty_w),
                   "current step duty range");
    }
}

static void test_invalid_pole_pairs_resets_current_integrator(void)
{
    double id = 0.0;
    double iq = 0.0;
    double vd = 0.0;
    double vq = 0.0;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double duty_u = 0.0;
    double duty_v = 0.0;
    double duty_w = 0.0;

    printf("\n== invalid pole_pairs resets current integrator ==\n");

    foc_sim_init();

    /*
     * 先给一个持续 iq 目标，让电流 PI 积分器累积非零状态。
     */
    for (int i = 0; i < 20; ++i) {
        (void)foc_sim_step_wrapper(0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   12.0,
                                   0.0,
                                   1.0,
                                   0.00005,
                                   7.0,
                                   0.0,
                                   &id,
                                   &iq,
                                   &vd,
                                   &vq,
                                   &v_alpha,
                                   &v_beta,
                                   &duty_u,
                                   &duty_v,
                                   &duty_w);
    }
    check_true(fabs(vq) > TEST_EPSILON_F, "current integrator accumulated before invalid input");

    /*
     * pole_pairs 无效时 wrapper 会提前返回；这里必须清积分器。
     */
    const int invalid_status = foc_sim_step_wrapper(0.0,
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    12.0,
                                                    0.0,
                                                    0.0,
                                                    0.00005,
                                                    0.0,
                                                    0.0,
                                                    &id,
                                                    &iq,
                                                    &vd,
                                                    &vq,
                                                    &v_alpha,
                                                    &v_beta,
                                                    &duty_u,
                                                    &duty_v,
                                                    &duty_w);
    check_true(invalid_status == -2, "invalid pole_pairs status");
    check_true((duty_u == 0.5) && (duty_v == 0.5) && (duty_w == 0.5),
               "invalid pole_pairs safe duty");

    /*
     * 如果积分器已经被 reset，下一次零目标/零电流应该输出近似零电压。
     */
    const int valid_status = foc_sim_step_wrapper(0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  12.0,
                                                  0.0,
                                                  0.0,
                                                  0.00005,
                                                  7.0,
                                                  0.0,
                                                  &id,
                                                  &iq,
                                                  &vd,
                                                  &vq,
                                                  &v_alpha,
                                                  &v_beta,
                                                  &duty_u,
                                                  &duty_v,
                                                  &duty_w);
    printf("after_invalid_pole_pairs: status=%d vd=% .6f V vq=% .6f V duty=(%.6f, %.6f, %.6f)\n",
           valid_status,
           vd,
           vq,
           duty_u,
           duty_v,
           duty_w);
    check_true(valid_status == 0, "valid step after invalid pole_pairs status");
    check_true(fabs(vd) <= TEST_EPSILON_F && fabs(vq) <= TEST_EPSILON_F,
               "current integrator reset after invalid pole_pairs");
}

static void test_velocity_wrapper(void)
{
    double iq_target = 0.0;

    printf("\n== velocity controller wrapper ==\n");

    const int status = foc_sim_velocity_step_wrapper(5.0,
                                                     1.0,
                                                     0.001,
                                                     0.02,
                                                     0.2,
                                                     1.0,
                                                     20.0,
                                                     1.0,
                                                     &iq_target);
    printf("velocity_wrapper: status=%d iq_target=% .6f A\n", status, iq_target);
    check_true(status == 0, "velocity wrapper status");
    check_true(isfinite(iq_target), "velocity wrapper finite output");
    check_true((iq_target >= -1.0) && (iq_target <= 1.0), "velocity wrapper current limit");
}

static void test_velocity_count_window(void)
{
    VelocityCountWindow window;
    const int32_t deltas[] = {1, 2, 1, 2, 1};
    float rpm = 0.0f;

    printf("\n== velocity count sliding window ==\n");
    velocity_count_window_reset(&window);
    for (uint32_t i = 0u; i < 5u; ++i) {
        rpm = velocity_count_window_update_rpm(&window,
                                               deltas[i],
                                               0.01f,
                                               4096u);
    }
    check_true(is_near(rpm, 2.05078125f, 1.0e-5f),
               "five 10ms deltas form one 50ms speed window");

    rpm = velocity_count_window_update_rpm(&window, 0, 0.01f, 4096u);
    check_true(is_near(rpm, 1.7578125f, 1.0e-5f),
               "sixth sample evicts the oldest delta");

    velocity_count_window_reset(&window);
    for (uint32_t i = 0u; i < 5u; ++i) {
        rpm = velocity_count_window_update_rpm(&window, -1, 0.01f, 4096u);
    }
    check_true(is_near(rpm, -1.46484375f, 1.0e-5f),
               "sliding window preserves reverse direction");

    check_true(velocity_count_window_update_rpm(0, 1, 0.01f, 4096u) == 0.0f &&
                   velocity_count_window_update_rpm(&window, 1, 0.0f, 4096u) == 0.0f &&
                   velocity_count_window_update_rpm(&window, 1, 0.01f, 0u) == 0.0f,
               "invalid estimator inputs return zero safely");

    check_true(velocity_count_window_set_samples(&window, 2u),
               "velocity estimator accepts a two-sample window");
    const int32_t short_deltas[] = {1, 2};
    for (uint32_t i = 0u; i < 2u; ++i) {
        rpm = velocity_count_window_update_rpm(&window,
                                               short_deltas[i],
                                               0.01f,
                                               4096u);
    }
    check_true(is_near(rpm, 2.197265625f, 1.0e-5f),
               "two 10ms deltas form one 20ms speed window");
    rpm = velocity_count_window_update_rpm(&window, 3, 0.01f, 4096u);
    check_true(is_near(rpm, 3.662109375f, 1.0e-5f),
               "two-sample window evicts its oldest delta");
    check_true(!velocity_count_window_set_samples(&window, 0u) &&
                   !velocity_count_window_set_samples(
                       &window, VELOCITY_COUNT_WINDOW_SAMPLES + 1u),
               "velocity estimator rejects invalid window lengths");

    check_true(velocity_count_window_set_samples(&window, 10u),
               "low-speed estimator accepts a ten-sample window");
    for (uint32_t i = 0u; i < 10u; ++i) {
        rpm = velocity_count_window_update_rpm(&window,
                                               1,
                                               0.01f,
                                               4096u);
    }
    check_true(is_near(rpm, 1.46484375f, 1.0e-5f),
               "ten 10ms samples form one 100ms low-speed window");

    VelocityEdgePeriodEstimator edge;
    velocity_edge_period_init(&edge, 1000, 400u);
    for (uint32_t tick = 0u; tick < 146u; ++tick) {
        velocity_edge_period_sample(&edge, (tick == 145u) ? 1001 : 1000);
    }
    check_true(is_near(velocity_edge_period_rpm(&edge, 20000.0f, 4096u),
                       2.0064f,
                       0.01f),
               "edge-period estimator resolves about 2rpm from one encoder interval");
    for (uint32_t tick = 0u; tick < 401u; ++tick) {
        velocity_edge_period_sample(&edge, 1001);
    }
    check_true(velocity_edge_period_rpm(&edge, 20000.0f, 4096u) == 0.0f,
               "edge-period estimator reports zero after the stale timeout");
    velocity_edge_period_init(&edge, 2000, 400u);
    for (uint32_t tick = 0u; tick < 146u; ++tick) {
        velocity_edge_period_sample(&edge, (tick == 145u) ? 1999 : 2000);
    }
    check_true(velocity_edge_period_rpm(&edge, 20000.0f, 4096u) < 0.0f,
               "edge-period estimator preserves reverse direction");

    float filtered_rpm = velocity_first_order_filter_step(0.0f, 8.0f, 0.25f);
    check_true(is_near(filtered_rpm, 2.0f, 1.0e-6f),
               "first-order speed filter applies the configured alpha");
    filtered_rpm = velocity_first_order_filter_step(filtered_rpm,
                                                    -2.0f,
                                                    0.25f);
    check_true(is_near(filtered_rpm, 1.0f, 1.0e-6f),
               "one reverse edge does not immediately reverse filtered speed");
    check_true(is_near(velocity_first_order_filter_step(1.0f, 7.0f, 2.0f),
                       7.0f,
                       1.0e-6f) &&
                   is_near(velocity_first_order_filter_step(1.0f,
                                                            7.0f,
                                                            0.0f),
                           1.0f,
                           1.0e-6f),
               "first-order speed filter clamps alpha at both boundaries");
}

static void test_velocity_controller_anti_windup(void)
{
    VelocityController controller;
    float output = 0.0f;

    printf("\n== velocity controller anti-windup ==\n");
    velocity_controller_init(&controller, 0.02f, 0.20f, 0.030f, 25.0f);
    for (uint32_t i = 0u; i < 200u; ++i) {
        output = velocity_controller_update(&controller,
                                            2.0f * 0.10471975512f,
                                            0.0f,
                                            0.01f);
    }
    check_true(output <= 0.030001f,
               "velocity PI output remains current limited");
    check_true(controller.integrator_a < 0.027f,
               "conditional integration prevents output-limit windup");
    check_true(controller.anti_windup_hold_count > 0u,
               "velocity PI records anti-windup holds");

    const float integrator_before_unwind = controller.integrator_a;
    (void)velocity_controller_update(&controller, 0.0f, 1.0f, 0.01f);
    check_true(controller.integrator_a < integrator_before_unwind,
               "opposite error immediately unwinds velocity integrator");

    velocity_controller_reset(&controller);
    output = velocity_controller_update(&controller, 10.0f, 0.0f, 0.01f);
    check_true(is_near(output, 0.030f, 1.0e-6f) &&
                   is_near(controller.integrator_a, 0.0f, 1.0e-6f),
               "proportional saturation does not wind the integrator");
    check_true(controller.saturation_count == 1u,
               "velocity PI records output saturation");

    velocity_controller_init(&controller, 0.0f, 0.0f, 0.030f, 25.0f);
    velocity_controller_set_error_reversal_decay(&controller, 0.25f);
    controller.integrator_a = 0.020f;
    controller.last_error_rad_s = 1.0f;
    output = velocity_controller_update(&controller, 0.0f, 1.0f, 0.01f);
    check_true(is_near(controller.integrator_a, 0.005f, 1.0e-6f) &&
                   is_near(output, 0.005f, 1.0e-6f) &&
                   controller.error_reversal_decay_count == 1u,
               "error reversal decays stale velocity integrator state");
}

static void test_low_speed_velocity_pi_candidate(void)
{
    const float rpm_to_rad_s = 0.10471975512f;
    const float update_dt_s = 0.01f;
    VelocityController controller;
    float target_rpm = 0.0f;
    float output_a = 0.0f;
    float output_peak_a = 0.0f;

    printf("\n== low-speed velocity PI candidate ==\n");
    velocity_controller_init(&controller,
                             0.040f,
                             0.004f,
                             0.030f,
                             25.0f * rpm_to_rad_s);
    controller.integrator_limit_a = 0.001f;
    velocity_controller_set_error_reversal_decay(&controller, 0.25f);
    velocity_controller_set_output_slew_limit(&controller, 0.20f);

    for (uint32_t update = 0u; update < 300u; ++update) {
        if (target_rpm < 2.0f) {
            target_rpm += 0.01f;
            if (target_rpm > 2.0f) {
                target_rpm = 2.0f;
            }
        }
        output_a = velocity_controller_update(&controller,
                                               target_rpm * rpm_to_rad_s,
                                               0.0f,
                                               update_dt_s);
        if (fabsf(output_a) > output_peak_a) {
            output_peak_a = fabsf(output_a);
        }
    }
    check_true(output_a >= 0.008f && output_a <= 0.010f,
               "2rpm conservative command stays in the measurable 8-10mA range");
    check_true(output_peak_a <= 0.030001f &&
                   controller.saturation_count == 0u,
               "2rpm ramp remains inside authorized 30mA output limit");
    VelocityLowSpeedAssist assist;
    velocity_low_speed_assist_reset(&assist);
    output_a = velocity_low_speed_assist_update(&assist,
                                                output_a,
                                                2.0f,
                                                0.0f,
                                                0.030f,
                                                0.5f,
                                                2.5f,
                                                0.030f,
                                                true);
    check_true(assist.active && is_near(output_a, 0.030f, 1.0e-6f),
               "stalled 2rpm command reaches the authorized 30mA stiction ceiling");
    output_a = velocity_low_speed_assist_update(&assist,
                                                output_a,
                                                2.0f,
                                                2.6f,
                                                0.030f,
                                                0.5f,
                                                2.5f,
                                                0.030f,
                                                true);
    check_true(!assist.active && assist.deactivation_count == 1u,
               "assist is removed above the upper speed hysteresis threshold");
    output_a = velocity_low_speed_assist_update(&assist,
                                                -0.010f,
                                                2.0f,
                                                0.0f,
                                                0.030f,
                                                0.5f,
                                                2.5f,
                                                0.030f,
                                                true);
    check_true(!assist.active && is_near(output_a, -0.010f, 1.0e-6f),
               "reverse braking request retains full authority without assist");
    output_a = velocity_low_speed_assist_update(&assist,
                                                0.029f,
                                                2.0f,
                                                0.0f,
                                                0.030f,
                                                0.5f,
                                                2.5f,
                                                0.030f,
                                                true);
    check_true(is_near(output_a, 0.030f, 1.0e-6f),
               "low-speed assist cannot exceed the authorized 30mA continuous limit");

    const float integrator_before_braking = controller.integrator_a;
    const float output_before_braking = output_a;
    float braking_output_a = velocity_controller_update(
        &controller, 2.0f * rpm_to_rad_s, 4.0f * rpm_to_rad_s, update_dt_s);
    check_true(braking_output_a < output_before_braking,
               "braking request first slews output toward zero");
    check_true(is_near(velocity_controller_apply_hold_direction_guard(
                           &controller,
                           -0.010f,
                           2.0f,
                           false),
                       0.0f,
                       1.0e-6f),
               "sub-threshold hold speed cannot command reverse torque");
    check_true(is_near(velocity_controller_apply_hold_direction_guard(
                           &controller,
                           -0.010f,
                           2.0f,
                           true),
                       -0.010f,
                       1.0e-6f),
               "qualified overspeed or fall phase retains reverse braking");
    check_true(is_near(velocity_controller_apply_coulomb_feedforward(
                           &controller,
                           0.004f,
                           2.0f,
                           0.008f,
                           true),
                       0.012f,
                       1.0e-6f),
               "positive hold friction feedforward adds bounded torque");
    check_true(is_near(velocity_controller_apply_coulomb_feedforward(
                           &controller,
                           0.028f,
                           2.0f,
                           0.008f,
                           true),
                       0.030f,
                       1.0e-6f),
               "hold friction feedforward respects the 30mA current limit");
    check_true(is_near(velocity_controller_apply_coulomb_feedforward(
                           &controller,
                           -0.004f,
                           -2.0f,
                           0.008f,
                           false),
                       -0.004f,
                       1.0e-6f),
               "disabled hold friction feedforward preserves fall command");
    velocity_low_speed_assist_reset(&assist);
    output_a = velocity_controller_apply_coulomb_feedforward(
        &controller, 0.008f, 2.0f, 0.008f, true);
    output_a = velocity_low_speed_assist_update(&assist,
                                                output_a,
                                                2.0f,
                                                0.0f,
                                                0.008f,
                                                0.75f,
                                                2.50f,
                                                0.030f,
                                                true);
    check_true(assist.active && is_near(output_a, 0.024f, 1.0e-6f),
               "manually selected 8mA hold assist remains below the authorized 30mA limit");
    for (uint32_t update = 0u; update < 4u && braking_output_a >= 0.0f;
         ++update) {
        braking_output_a = velocity_controller_update(
            &controller, 2.0f * rpm_to_rad_s, 4.0f * rpm_to_rad_s, update_dt_s);
    }
    check_true(braking_output_a < 0.0f,
               "sustained measured overspeed commands reverse braking iq");
    check_true(controller.integrator_a < integrator_before_braking,
               "reverse braking unwinds stale positive integrator");

    velocity_controller_reset(&controller);
    for (uint32_t update = 0u; update < 200u; ++update) {
        output_a = velocity_controller_update(&controller,
                                               25.0f * rpm_to_rad_s,
                                               -25.0f * rpm_to_rad_s,
                                               update_dt_s);
    }
    check_true(is_near(output_a, 0.030f, 1.0e-6f),
               "large speed error stays capped at 30mA");
    check_true(controller.integrator_a <= 0.001001f &&
                    controller.anti_windup_hold_count > 0u,
                "30mA saturation preserves velocity anti-windup");

    velocity_controller_reset(&controller);
    VelocityCountWindow candidate_window;
    check_true(velocity_count_window_set_samples(&candidate_window, 5u),
               "candidate trace uses the quantization-aware five-sample window");
    const int32_t hardware_trace[] = {1, -2, 0, 8, 4, -17, -21};
    float previous_output_a = 0.0f;
    float trace_peak_a = 0.0f;
    bool trace_slew_ok = true;
    bool trace_direct_reversal_seen = false;
    target_rpm = 0.0f;
    for (uint32_t i = 0u;
         i < (sizeof(hardware_trace) / sizeof(hardware_trace[0])); ++i) {
        target_rpm += 0.01f;
        const float measured_rpm = velocity_count_window_update_rpm(
            &candidate_window,
            hardware_trace[i],
            update_dt_s,
            4096u);
        output_a = velocity_controller_update(&controller,
                                               target_rpm * rpm_to_rad_s,
                                               measured_rpm * rpm_to_rad_s,
                                               update_dt_s);
        const bool same_nonzero_sign =
            (output_a * previous_output_a) > 0.0f;
        if (same_nonzero_sign &&
            fabsf(output_a) > fabsf(previous_output_a) + 0.002001f) {
            trace_slew_ok = false;
        }
        if ((output_a * previous_output_a) < 0.0f) {
            trace_direct_reversal_seen = true;
        }
        if (fabsf(output_a) > trace_peak_a) {
            trace_peak_a = fabsf(output_a);
        }
        previous_output_a = output_a;
    }
    check_true(trace_slew_ok,
               "hardware delta trace limits increasing torque magnitude");
    check_true(!trace_direct_reversal_seen,
               "hardware delta trace crosses torque direction through zero");
    check_true(trace_peak_a < 0.030f && controller.slew_limit_count > 0u,
               "hardware delta trace avoids immediate full-scale iq toggling");

    VelocityCountWindow speed_window;
    VelocityOverspeedAnalysis overspeed;
    check_true(velocity_count_window_set_samples(&speed_window, 5u),
               "25rpm guard uses configured five-sample window");
    for (uint32_t sample = 0u; sample < 5u; ++sample) {
        (void)velocity_count_window_update_rpm(&speed_window,
                                               18,
                                               update_dt_s,
                                               4096u);
    }
    velocity_count_window_analyze_overspeed(&speed_window,
                                            update_dt_s,
                                            4096u,
                                            25.0f,
                                            &overspeed);
    check_true(overspeed.windowed_rpm > 25.0f &&
                   overspeed.evidence == VELOCITY_OVERSPEED_EVIDENCE_WINDOWED,
               "sustained speed above 25rpm triggers windowed protection");
}

static void test_closed_loop_first_order_iq(void)
{
    const double dt = 5.0e-5;
    const double stop_time_s = 0.05;
    const int steps = (int)(stop_time_s / dt);
    const double r_ohm = 0.5;
    const double l_h = 1.0e-3;
    const double iq_ref_a = 0.5;
    const double voltage_limit_v = 3.0;
    double iq_a = 0.0;
    double id = 0.0;
    double iq_measured = 0.0;
    double vd = 0.0;
    double vq = 0.0;
    double v_alpha = 0.0;
    double v_beta = 0.0;
    double duty_u = 0.0;
    double duty_v = 0.0;
    double duty_w = 0.0;
    int status_ok = 1;
    int duty_ok = 1;
    int voltage_ok = 1;

    printf("\n== closed-loop first-order iq plant ==\n");

    /*
     * 重新初始化电流 PI，避免前面边界测试留下积分状态。
     * 该测试使用 theta_e=0，因此 Park 后 id=i_alpha、iq=i_beta。
     */
    foc_sim_init();

    for (int i = 0; i < steps; ++i) {
        const double ia = 0.0;
        const double ib = iq_a * 0.8660254037844386;
        const double ic = -ib;
        const int status = foc_sim_step_wrapper(ia,
                                                ib,
                                                ic,
                                                0.0,
                                                0.0,
                                                12.0,
                                                0.0,
                                                iq_ref_a,
                                                dt,
                                                7.0,
                                                0.0,
                                                &id,
                                                &iq_measured,
                                                &vd,
                                                &vq,
                                                &v_alpha,
                                                &v_beta,
                                                &duty_u,
                                                &duty_v,
                                                &duty_w);
        const double voltage_mag = sqrt(vd * vd + vq * vq);

        if (status != 0) {
            status_ok = 0;
        }
        if (!(is_duty_valid(duty_u) &&
              is_duty_valid(duty_v) &&
              is_duty_valid(duty_w))) {
            duty_ok = 0;
        }
        if (voltage_mag > (voltage_limit_v + 1.0e-6)) {
            voltage_ok = 0;
        }

        iq_a += ((vq - r_ohm * iq_a) / l_h) * dt;
    }

    printf("closed_loop_iq: iq_final=% .6f A iq_ref=% .6f A vd=% .6f V vq=% .6f V duty=(%.6f, %.6f, %.6f)\n",
           iq_a,
           iq_ref_a,
           vd,
           vq,
           duty_u,
           duty_v,
           duty_w);

    check_true(status_ok, "closed-loop plant status");
    check_true(duty_ok, "closed-loop plant duty range");
    check_true(voltage_ok, "closed-loop plant voltage limit");
    check_true(fabs(iq_a - iq_ref_a) <= 0.05, "closed-loop plant final iq near reference");
}

int main(void)
{
    test_math_blocks();
    test_foc_sim_nominal_and_edges();
    test_current_step_response();
    test_invalid_pole_pairs_resets_current_integrator();
    test_velocity_wrapper();
    test_velocity_count_window();
    test_velocity_controller_anti_windup();
    test_low_speed_velocity_pi_candidate();
    test_closed_loop_first_order_iq();

    if (g_failures != 0) {
        printf("\nFOC PC unit test failed: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("\nFOC PC unit test passed.\n");
    return 0;
}
