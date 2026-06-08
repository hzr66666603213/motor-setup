#include "sim/foc_sim.h"

#include <math.h>
#include <stdio.h>

/*
 * foc_sim_test.c
 *
 * PC 侧 FOC 仿真入口 smoke test。
 * 该测试不链接任何 HAL 文件，只验证 Simulink 推荐入口 foc_sim_step_wrapper()：
 * - 不需要外部初始化；
 * - pole_pairs 使用 double；
 * - 输出为有限值；
 * - duty 始终位于 0.0..1.0；
 * - 无效 vbus 时输出安全 50% duty。
 */

static int in_range(double x, double lo, double hi)
{
    return (x >= lo) && (x <= hi);
}

int main(void)
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

    const int status = foc_sim_step_wrapper(0.2,
                                            -0.1,
                                            -0.1,
                                            0.3,
                                            1.0,
                                            12.0,
                                            0.0,
                                            0.5,
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

    if (status != 0) {
        return 1;
    }

    if (!isfinite(id) ||
        !isfinite(iq) ||
        !isfinite(vd) ||
        !isfinite(vq) ||
        !isfinite(v_alpha) ||
        !isfinite(v_beta)) {
        return 2;
    }

    if (!in_range(duty_u, 0.0, 1.0) ||
        !in_range(duty_v, 0.0, 1.0) ||
        !in_range(duty_w, 0.0, 1.0)) {
        return 3;
    }

    const int invalid_status = foc_sim_step_wrapper(0.0,
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    0.0,
                                                    0.0,
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

    if (invalid_status != -2) {
        return 4;
    }
    if ((duty_u != 0.5) || (duty_v != 0.5) || (duty_w != 0.5)) {
        return 5;
    }

    printf("foc_sim_test passed: id=%f iq=%f vd=%f vq=%f duty=(%f,%f,%f)\n",
           id,
           iq,
           vd,
           vq,
           duty_u,
           duty_v,
           duty_w);
    return 0;
}
