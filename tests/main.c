#include "control/current_controller.h"
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
    test_velocity_wrapper();
    test_closed_loop_first_order_iq();

    if (g_failures != 0) {
        printf("\nFOC PC unit test failed: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("\nFOC PC unit test passed.\n");
    return 0;
}
