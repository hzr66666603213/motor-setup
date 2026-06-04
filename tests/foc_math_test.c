#include "foc/foc_math.h"

#include <math.h>
#include <stdio.h>

/*
 * foc_math_test.c
 *
 * FOC 数学模块的最小 smoke test。
 * 目的不是验证所有控制精度，而是用固定输入检查：
 * - Clarke/Park/反 Park 输出为有限值。
 * - SVPWM duty 始终落在 0..1。
 *
 * 该文件是 PC 侧测试，可以使用 printf；固件 ISR 中禁止 printf。
 */

static int in_range(float x, float lo, float hi)
{
    /* 测试辅助函数：检查数值是否位于闭区间 [lo, hi]。 */
    return (x >= lo) && (x <= hi);
}

int main(void)
{
    /* 固定输入，便于在不同编译器/平台上快速发现明显数学错误。 */
    float alpha = 0.0f;
    float beta = 0.0f;
    float d = 0.0f;
    float q = 0.0f;
    float va = 0.0f;
    float vb = 0.0f;
    float du = 0.0f;
    float dv = 0.0f;
    float dw = 0.0f;

    foc_clarke(1.0f, -0.5f, -0.5f, &alpha, &beta);
    foc_park(alpha, beta, 0.0f, &d, &q);
    foc_inv_park(1.0f, 0.0f, 0.0f, &va, &vb);
    foc_svpwm(2.0f, 1.0f, 24.0f, &du, &dv, &dw);

    /* 变换结果不应出现 NaN 或 Inf。 */
    if (!isfinite(alpha) || !isfinite(beta) || !isfinite(d) || !isfinite(q)) {
        return 1;
    }
    /* SVPWM 输出必须在可写入 PWM 后端的 duty 范围内。 */
    if (!in_range(du, 0.0f, 1.0f) || !in_range(dv, 0.0f, 1.0f) || !in_range(dw, 0.0f, 1.0f)) {
        return 2;
    }

    printf("foc_math_test passed: alpha=%f beta=%f duty=(%f,%f,%f)\n", alpha, beta, du, dv, dw);
    return 0;
}
