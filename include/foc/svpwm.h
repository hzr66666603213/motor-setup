#ifndef SVPWM_H
#define SVPWM_H

/*
 * svpwm.h
 *
 * 独立 SVPWM 模块。
 * foc_math.c 中也保留了 foc_svpwm()，本模块作为更清晰的工程化入口。
 * 运行上下文：20 kHz PWM ISR 可调用。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float duty_a; /* A/U 相 duty，0..1 */
    float duty_b; /* B/V 相 duty，0..1 */
    float duty_c; /* C/W 相 duty，0..1 */
} SvpwmDuty;

SvpwmDuty svpwm_generate(float v_alpha_v, float v_beta_v, float vbus_v);

#ifdef __cplusplus
}
#endif

#endif /* SVPWM_H */
