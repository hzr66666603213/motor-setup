#include "sim/foc_sim.h"

#include "foc/foc_math.h"
#include "foc/svpwm.h"

#include <math.h>
#include <stddef.h>

/*
 * foc_sim.c
 *
 * 面向 Simulink/PC 的纯算法 FOC 仿真入口。
 *
 * 本文件故意不包含任何 STM32 HAL 头文件，也不调用任何 hal_* 接口。
 * 它只复用固件中的数学和控制器模块，用来在 Simulink 中验证：
 * - Clarke/Park 坐标变换；
 * - d/q 电流 PI；
 * - 反 Park；
 * - SVPWM duty 生成。
 *
 * 这使得同一套 FOC 算法可以先在 PC/Simulink 中调通，再移植到 20 kHz PWM ISR。
 */

#define FOC_SIM_DEFAULT_CURRENT_KP       0.05f
#define FOC_SIM_DEFAULT_CURRENT_KI       100.0f
#define FOC_SIM_DEFAULT_MAX_VOLTAGE_V    3.0f
#define FOC_SIM_MIN_VALID_VBUS_V         1.0f
#define FOC_SIM_MAX_POLE_PAIRS_DOUBLE    255.0

static FocSimContext s_foc_sim_ctx;
static FocSimVelocityContext s_foc_sim_velocity_ctx;

static void foc_sim_fill_safe_output(FocSimOutput *output)
{
    if (output == NULL) {
        return;
    }

    output->id_a = 0.0f;
    output->iq_a = 0.0f;
    output->vd_v = 0.0f;
    output->vq_v = 0.0f;
    output->v_alpha_v = 0.0f;
    output->v_beta_v = 0.0f;
    output->duty_u = 0.5f;
    output->duty_v = 0.5f;
    output->duty_w = 0.5f;
}

static int foc_sim_input_is_valid(float ia_a,
                                  float ib_a,
                                  float ic_a,
                                  float mechanical_angle_rad,
                                  float mechanical_velocity_rad_s,
                                  float vbus_v,
                                  float id_target_a,
                                  float iq_target_a,
                                  float dt_s,
                                  uint8_t pole_pairs,
                                  float encoder_offset_rad)
{
    /*
     * PC/Simulink 仿真中更容易因为模型初始化顺序出现 NaN/Inf。
     * 这里在仿真入口做轻量检查，真实 ISR 侧仍建议把有效性检查放在传感器/保护模块。
     */
    if (!isfinite(ia_a) ||
        !isfinite(ib_a) ||
        !isfinite(ic_a) ||
        !isfinite(mechanical_angle_rad) ||
        !isfinite(mechanical_velocity_rad_s) ||
        !isfinite(vbus_v) ||
        !isfinite(id_target_a) ||
        !isfinite(iq_target_a) ||
        !isfinite(dt_s) ||
        !isfinite(encoder_offset_rad)) {
        return 0;
    }

    if ((dt_s <= 0.0f) || (vbus_v <= FOC_SIM_MIN_VALID_VBUS_V) || (pole_pairs == 0u)) {
        return 0;
    }

    return 1;
}

static void foc_sim_write_double_output(const FocSimOutput *output,
                                        double *id_a,
                                        double *iq_a,
                                        double *vd_v,
                                        double *vq_v,
                                        double *v_alpha_v,
                                        double *v_beta_v,
                                        double *duty_u,
                                        double *duty_v,
                                        double *duty_w)
{
    if (output == NULL) {
        return;
    }

    if (id_a != NULL) {
        *id_a = (double)output->id_a;
    }
    if (iq_a != NULL) {
        *iq_a = (double)output->iq_a;
    }
    if (vd_v != NULL) {
        *vd_v = (double)output->vd_v;
    }
    if (vq_v != NULL) {
        *vq_v = (double)output->vq_v;
    }
    if (v_alpha_v != NULL) {
        *v_alpha_v = (double)output->v_alpha_v;
    }
    if (v_beta_v != NULL) {
        *v_beta_v = (double)output->v_beta_v;
    }
    if (duty_u != NULL) {
        *duty_u = (double)output->duty_u;
    }
    if (duty_v != NULL) {
        *duty_v = (double)output->duty_v;
    }
    if (duty_w != NULL) {
        *duty_w = (double)output->duty_w;
    }
}

static int foc_sim_pole_pairs_from_double(double pole_pairs_double, uint8_t *pole_pairs)
{
    double rounded = 0.0;

    if (pole_pairs == NULL) {
        return 0;
    }
    if (!isfinite(pole_pairs_double) ||
        (pole_pairs_double < 1.0) ||
        (pole_pairs_double > FOC_SIM_MAX_POLE_PAIRS_DOUBLE)) {
        return 0;
    }

    /*
     * Simulink 默认数值类型是 double。
     * 这里接受 7.0，也接受 6.9/7.1 这类由模型运算产生的小数，统一四舍五入到整数极对数。
     */
    rounded = floor(pole_pairs_double + 0.5);
    if ((rounded < 1.0) || (rounded > FOC_SIM_MAX_POLE_PAIRS_DOUBLE)) {
        return 0;
    }

    *pole_pairs = (uint8_t)rounded;
    return 1;
}

void foc_sim_context_init(FocSimContext *ctx,
                          float current_kp,
                          float current_ki,
                          float max_voltage_v)
{
    if (ctx == NULL) {
        return;
    }

    if (!isfinite(current_kp)) {
        current_kp = FOC_SIM_DEFAULT_CURRENT_KP;
    }
    if (!isfinite(current_ki)) {
        current_ki = FOC_SIM_DEFAULT_CURRENT_KI;
    }
    if ((!isfinite(max_voltage_v)) || (max_voltage_v <= 0.0f)) {
        max_voltage_v = FOC_SIM_DEFAULT_MAX_VOLTAGE_V;
    }

    ctx->current_kp = current_kp;
    ctx->current_ki = current_ki;
    ctx->max_voltage_v = max_voltage_v;
    ctx->last_mechanical_angle_rad = 0.0f;
    ctx->last_mechanical_velocity_rad_s = 0.0f;
    ctx->initialized = 1u;

    current_controller_init(&ctx->current_controller, current_kp, current_ki, max_voltage_v);
}

void foc_sim_context_reset(FocSimContext *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->initialized == 0u) {
        foc_sim_context_init(ctx,
                             FOC_SIM_DEFAULT_CURRENT_KP,
                             FOC_SIM_DEFAULT_CURRENT_KI,
                             FOC_SIM_DEFAULT_MAX_VOLTAGE_V);
        return;
    }

    current_controller_reset(&ctx->current_controller);
}

int foc_sim_context_step(FocSimContext *ctx,
                         float ia_a,
                         float ib_a,
                         float ic_a,
                         float mechanical_angle_rad,
                         float mechanical_velocity_rad_s,
                         float vbus_v,
                         float id_target_a,
                         float iq_target_a,
                         float dt_s,
                         uint8_t pole_pairs,
                         float encoder_offset_rad,
                         FocSimOutput *output)
{
    float i_alpha_a = 0.0f;
    float i_beta_a = 0.0f;
    float electrical_angle_rad = 0.0f;
    SvpwmDuty duty;

    if ((ctx == NULL) || (output == NULL)) {
        return -1;
    }

    if (ctx->initialized == 0u) {
        foc_sim_context_init(ctx,
                             FOC_SIM_DEFAULT_CURRENT_KP,
                             FOC_SIM_DEFAULT_CURRENT_KI,
                             FOC_SIM_DEFAULT_MAX_VOLTAGE_V);
    }

    if (!foc_sim_input_is_valid(ia_a,
                                ib_a,
                                ic_a,
                                mechanical_angle_rad,
                                mechanical_velocity_rad_s,
                                vbus_v,
                                id_target_a,
                                iq_target_a,
                                dt_s,
                                pole_pairs,
                                encoder_offset_rad)) {
        current_controller_reset(&ctx->current_controller);
        foc_sim_fill_safe_output(output);
        return -2;
    }

    ctx->last_mechanical_angle_rad = mechanical_angle_rad;
    ctx->last_mechanical_velocity_rad_s = mechanical_velocity_rad_s;

    electrical_angle_rad = foc_electrical_angle(mechanical_angle_rad,
                                                pole_pairs,
                                                encoder_offset_rad);

    /*
     * 三相静止坐标 -> alpha/beta 静止坐标。
     * alpha/beta 是定子坐标，仍然固定在电机相坐标上。
     */
    foc_clarke(ia_a, ib_a, ic_a, &i_alpha_a, &i_beta_a);

    /*
     * alpha/beta -> d/q 同步旋转坐标。
     * d 轴一般对应磁链方向，q 轴对应转矩电流方向。
     */
    foc_park(i_alpha_a,
             i_beta_a,
             electrical_angle_rad,
             &output->id_a,
             &output->iq_a);

    /*
     * d/q 电流 PI。
     * current_controller_update() 内部已经根据 vbus/sqrt(3) 和 max_voltage_v 做电压限幅，
     * 并执行抗积分饱和。
     */
    current_controller_update(&ctx->current_controller,
                              id_target_a,
                              iq_target_a,
                              output->id_a,
                              output->iq_a,
                              vbus_v,
                              dt_s,
                              &output->vd_v,
                              &output->vq_v);

    /*
     * d/q 电压指令 -> alpha/beta 电压指令。
     * 这是逆 Park 变换，输出将送入 SVPWM。
     */
    foc_inv_park(output->vd_v,
                 output->vq_v,
                 electrical_angle_rad,
                 &output->v_alpha_v,
                 &output->v_beta_v);

    /*
     * SVPWM 只生成 duty，不访问真实 PWM 外设。
     * 因此该函数可以安全地用于 gcc/Simulink 仿真。
     */
    duty = svpwm_generate(output->v_alpha_v, output->v_beta_v, vbus_v);
    output->duty_u = duty.duty_a;
    output->duty_v = duty.duty_b;
    output->duty_w = duty.duty_c;

    return 0;
}

void foc_sim_init(void)
{
    foc_sim_context_init(&s_foc_sim_ctx,
                         FOC_SIM_DEFAULT_CURRENT_KP,
                         FOC_SIM_DEFAULT_CURRENT_KI,
                         FOC_SIM_DEFAULT_MAX_VOLTAGE_V);
}

void foc_sim_set_current_controller(float current_kp,
                                    float current_ki,
                                    float max_voltage_v)
{
    foc_sim_context_init(&s_foc_sim_ctx, current_kp, current_ki, max_voltage_v);
}

void foc_sim_reset(void)
{
    foc_sim_context_reset(&s_foc_sim_ctx);
}

int foc_sim_step(float ia_a,
                 float ib_a,
                 float ic_a,
                 float mechanical_angle_rad,
                 float mechanical_velocity_rad_s,
                 float vbus_v,
                 float id_target_a,
                 float iq_target_a,
                 float dt_s,
                 uint8_t pole_pairs,
                 float encoder_offset_rad,
                 float *id_a,
                 float *iq_a,
                 float *vd_v,
                 float *vq_v,
                 float *v_alpha_v,
                 float *v_beta_v,
                 float *duty_u,
                 float *duty_v,
                 float *duty_w)
{
    FocSimOutput output;
    const int status = foc_sim_context_step(&s_foc_sim_ctx,
                                            ia_a,
                                            ib_a,
                                            ic_a,
                                            mechanical_angle_rad,
                                            mechanical_velocity_rad_s,
                                            vbus_v,
                                            id_target_a,
                                            iq_target_a,
                                            dt_s,
                                            pole_pairs,
                                            encoder_offset_rad,
                                            &output);

    if (id_a != NULL) {
        *id_a = output.id_a;
    }
    if (iq_a != NULL) {
        *iq_a = output.iq_a;
    }
    if (vd_v != NULL) {
        *vd_v = output.vd_v;
    }
    if (vq_v != NULL) {
        *vq_v = output.vq_v;
    }
    if (v_alpha_v != NULL) {
        *v_alpha_v = output.v_alpha_v;
    }
    if (v_beta_v != NULL) {
        *v_beta_v = output.v_beta_v;
    }
    if (duty_u != NULL) {
        *duty_u = output.duty_u;
    }
    if (duty_v != NULL) {
        *duty_v = output.duty_v;
    }
    if (duty_w != NULL) {
        *duty_w = output.duty_w;
    }

    return status;
}

int foc_sim_step_wrapper(double ia_a,
                         double ib_a,
                         double ic_a,
                         double mechanical_angle_rad,
                         double mechanical_velocity_rad_s,
                         double vbus_v,
                         double id_target_a,
                         double iq_target_a,
                         double dt_s,
                         double pole_pairs_double,
                         double encoder_offset_rad,
                         double *id_a,
                         double *iq_a,
                         double *vd_v,
                         double *vq_v,
                         double *v_alpha_v,
                         double *v_beta_v,
                         double *duty_u,
                         double *duty_v,
                         double *duty_w)
{
    FocSimOutput output;
    uint8_t pole_pairs = 0u;
    int status = 0;

    /*
     * Wrapper 是 Simulink C Caller 推荐入口，因此初始化放在这里。
     * 模型不需要用 coder.ceval() 或 InitFcn 单独调用 foc_sim_init()。
     */
    if (s_foc_sim_ctx.initialized == 0u) {
        foc_sim_context_init(&s_foc_sim_ctx,
                             FOC_SIM_DEFAULT_CURRENT_KP,
                             FOC_SIM_DEFAULT_CURRENT_KI,
                             FOC_SIM_DEFAULT_MAX_VOLTAGE_V);
    }

    if (!foc_sim_pole_pairs_from_double(pole_pairs_double, &pole_pairs)) {
        current_controller_reset(&s_foc_sim_ctx.current_controller);
        foc_sim_fill_safe_output(&output);
        foc_sim_write_double_output(&output,
                                    id_a,
                                    iq_a,
                                    vd_v,
                                    vq_v,
                                    v_alpha_v,
                                    v_beta_v,
                                    duty_u,
                                    duty_v,
                                    duty_w);
        return -2;
    }

    status = foc_sim_context_step(&s_foc_sim_ctx,
                                  (float)ia_a,
                                  (float)ib_a,
                                  (float)ic_a,
                                  (float)mechanical_angle_rad,
                                  (float)mechanical_velocity_rad_s,
                                  (float)vbus_v,
                                  (float)id_target_a,
                                  (float)iq_target_a,
                                  (float)dt_s,
                                  pole_pairs,
                                  (float)encoder_offset_rad,
                                  &output);

    foc_sim_write_double_output(&output,
                                id_a,
                                iq_a,
                                vd_v,
                                vq_v,
                                v_alpha_v,
                                v_beta_v,
                                duty_u,
                                duty_v,
                                duty_w);

    return status;
}

int foc_sim_velocity_step_wrapper(double velocity_target_rad_s,
                                  double velocity_measured_rad_s,
                                  double dt_s,
                                  double velocity_kp,
                                  double velocity_ki,
                                  double current_limit_a,
                                  double velocity_limit_rad_s,
                                  double reset_integrator,
                                  double *iq_target_a)
{
    float iq_target = 0.0f;

    if (iq_target_a == NULL) {
        return -1;
    }

    if (!isfinite(velocity_target_rad_s) ||
        !isfinite(velocity_measured_rad_s) ||
        !isfinite(dt_s) ||
        !isfinite(velocity_kp) ||
        !isfinite(velocity_ki) ||
        !isfinite(current_limit_a) ||
        !isfinite(velocity_limit_rad_s) ||
        (dt_s <= 0.0) ||
        (current_limit_a <= 0.0) ||
        (velocity_limit_rad_s <= 0.0)) {
        if (s_foc_sim_velocity_ctx.initialized != 0u) {
            velocity_controller_reset(&s_foc_sim_velocity_ctx.velocity_controller);
        }
        *iq_target_a = 0.0;
        return -2;
    }

    /*
     * Simulink 常用 Constant 块在线修改 kp/ki/limit。
     * 为了让 wrapper 独立可用，这里每步同步参数；当用户给 reset_integrator 非 0 时清积分。
     */
    if (s_foc_sim_velocity_ctx.initialized == 0u) {
        velocity_controller_init(&s_foc_sim_velocity_ctx.velocity_controller,
                                 (float)velocity_kp,
                                 (float)velocity_ki,
                                 (float)current_limit_a,
                                 (float)velocity_limit_rad_s);
        s_foc_sim_velocity_ctx.initialized = 1u;
    } else {
        s_foc_sim_velocity_ctx.velocity_controller.current_limit_a = (float)current_limit_a;
        s_foc_sim_velocity_ctx.velocity_controller.integrator_limit_a = (float)current_limit_a;
        s_foc_sim_velocity_ctx.velocity_controller.velocity_limit_rad_s = (float)velocity_limit_rad_s;
        velocity_controller_set_gains(&s_foc_sim_velocity_ctx.velocity_controller,
                                      (float)velocity_kp,
                                      (float)velocity_ki);
    }

    if (reset_integrator != 0.0) {
        velocity_controller_reset(&s_foc_sim_velocity_ctx.velocity_controller);
    }

    iq_target = velocity_controller_update(&s_foc_sim_velocity_ctx.velocity_controller,
                                           (float)velocity_target_rad_s,
                                           (float)velocity_measured_rad_s,
                                           (float)dt_s);
    s_foc_sim_velocity_ctx.last_velocity_target_rad_s = (float)velocity_target_rad_s;
    s_foc_sim_velocity_ctx.last_velocity_measured_rad_s = (float)velocity_measured_rad_s;
    *iq_target_a = (double)iq_target;
    return 0;
}
