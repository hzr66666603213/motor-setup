#include "foc/svpwm.h"

#include "foc/foc_math.h"

/*
 * svpwm.c
 *
 * SVPWM 独立封装。
 * 输入 alpha/beta 静止坐标系电压，输出三相 duty。
 * 当 vbus 过低时输出 50% duty，实际关断由保护模块处理。
 */

SvpwmDuty svpwm_generate(float v_alpha_v, float v_beta_v, float vbus_v)
{
    SvpwmDuty duty;
    foc_svpwm(v_alpha_v, v_beta_v, vbus_v, &duty.duty_a, &duty.duty_b, &duty.duty_c);
    return duty;
}
