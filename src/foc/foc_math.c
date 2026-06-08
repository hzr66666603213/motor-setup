#include "foc/foc_math.h"

#include <math.h>

/*
 * foc_math.c
 *
 * 本文件只包含纯数学计算，不依赖 HAL、不访问全局硬件状态。
 * 这些函数会被 20 kHz 电流环频繁调用，因此保持实现短小、确定、无阻塞。
 *
 * 坐标系约定：
 * - alpha/beta 是定子静止坐标系，alpha 轴与 A/U 相同向；
 * - d/q 是随转子电角度旋转的同步坐标系；
 * - d 轴通常对齐转子磁链，q 轴正方向按本项目 Park 变换符号约定产生正转矩。
 *
 * 注意：这里不做 NaN/Inf 防御，调用者应在 ADC、编码器、参数入口处完成有效性检查，
 * 这样 ISR 内数学路径才能保持固定时延。
 */

static float foc_min3(float a, float b, float c)
{
    /* 三相零序注入需要快速取得三相电压最小值。 */
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

static float foc_max3(float a, float b, float c)
{
    /* 三相零序注入需要快速取得三相电压最大值。 */
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

float foc_clamp(float value, float min_value, float max_value)
{
    /* 简单饱和限幅，避免控制器输出或 duty 超过物理范围。 */
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float foc_wrap_0_2pi(float angle_rad)
{
    /* fmodf 对负数会返回负余数，因此需要补回 2*pi。 */
    float wrapped = fmodf(angle_rad, FOC_TWO_PI_F);
    if (wrapped < 0.0f) {
        wrapped += FOC_TWO_PI_F;
    }
    return wrapped;
}

float foc_wrap_minuspi_pi(float angle_rad)
{
    /*
     * 先归一化到 [0, 2*pi)，再把 pi..2*pi 区间映射为负角度。
     * 该形式适合位置误差：目标角 - 当前角 可以得到最短旋转方向。
     */
    float wrapped = foc_wrap_0_2pi(angle_rad);
    if (wrapped >= 3.14159265358979323846f) {
        wrapped -= FOC_TWO_PI_F;
    }
    return wrapped;
}

float foc_electrical_angle(float mechanical_angle_rad, uint8_t pole_pairs, float offset_rad)
{
    /* 电角度 = 机械角 * 极对数 + 电角度/编码器零偏。 */
    return foc_wrap_0_2pi(mechanical_angle_rad * (float)pole_pairs + offset_rad);
}

float foc_electrical_angle_dir(float mechanical_angle_rad,
                               uint8_t pole_pairs,
                               int direction,
                               float encoder_offset_rad)
{
    /*
     * ABZ 编码器方向可能与电机相序定义相反。
     * direction 用 +1/-1 把“机械计数增加方向”统一到 FOC 的正电角度方向，
     * encoder_offset_rad 再补偿校准时测得的电角度零位。
     */
    const float dir = (direction >= 0) ? 1.0f : -1.0f;
    return foc_wrap_0_2pi(mechanical_angle_rad * (float)pole_pairs * dir + encoder_offset_rad);
}

float normalize_0_2pi(float angle_rad)
{
    return foc_wrap_0_2pi(angle_rad);
}

float normalize_minuspi_pi(float angle_rad)
{
    return foc_wrap_minuspi_pi(angle_rad);
}

void foc_clarke(float ia_a, float ib_a, float ic_a, float *i_alpha_a, float *i_beta_a)
{
    (void)ic_a;
    /*
     * 平衡三相形式：
     * alpha = ia
     * beta  = (ia + 2*ib) / sqrt(3)
     * ic 当前未直接使用，但保留输入便于后续做三相和校验。
     */
    *i_alpha_a = ia_a;
    *i_beta_a = 0.57735026919f * (ia_a + 2.0f * ib_a);
}

void foc_park(float i_alpha_a, float i_beta_a, float electrical_angle_rad, float *id_a, float *iq_a)
{
    /*
     * 将静止坐标系电流旋转到转子磁场坐标系。
     * 当 electrical_angle_rad 正确时：
     * - id 表示磁链方向电流，表贴 PMSM/无刷电机调试初期通常目标为 0；
     * - iq 表示转矩方向电流，速度环/力矩环主要通过它控制输出力矩。
     */
    const float c = cosf(electrical_angle_rad);
    const float s = sinf(electrical_angle_rad);
    *id_a = i_alpha_a * c + i_beta_a * s;
    *iq_a = -i_alpha_a * s + i_beta_a * c;
}

void foc_inv_park(float vd_v, float vq_v, float electrical_angle_rad, float *v_alpha_v, float *v_beta_v)
{
    /*
     * 将 d/q 轴电压指令旋回静止坐标系，供 SVPWM 使用。
     * 这里使用与 foc_park 成对的符号约定，二者必须同时维护；
     * 如果后续修改电机相序或编码器方向，不应单独改这里的公式。
     */
    const float c = cosf(electrical_angle_rad);
    const float s = sinf(electrical_angle_rad);
    *v_alpha_v = vd_v * c - vq_v * s;
    *v_beta_v = vd_v * s + vq_v * c;
}

void foc_limit_voltage(float *vd_v, float *vq_v, float max_voltage_v)
{
    /*
     * 圆形限幅优于分别限幅 vd/vq，可避免电压矢量方向被严重扭曲。
     * max_voltage_v 是用户/调试层的保守限制，不一定等于 SVPWM 理论可用电压；
     * 真实硬件上还要留出死区、采样窗口、母线纹波和电流环动态裕量。
     */
    const float mag_sq = (*vd_v * *vd_v) + (*vq_v * *vq_v);
    const float limit_sq = max_voltage_v * max_voltage_v;

    if ((max_voltage_v > 0.0f) && (mag_sq > limit_sq)) {
        const float scale = max_voltage_v / sqrtf(mag_sq);
        *vd_v *= scale;
        *vq_v *= scale;
    }
}

void foc_svpwm(float v_alpha_v, float v_beta_v, float vbus_v, float *duty_u, float *duty_v, float *duty_w)
{
    if (vbus_v <= 1.0f) {
        /* 母线电压无效时输出 50% duty，交给保护模块关断功率级。 */
        *duty_u = 0.5f;
        *duty_v = 0.5f;
        *duty_w = 0.5f;
        return;
    }

    /*
     * 使用零序/共模注入实现等效 SVPWM：
     * 1. 先由 alpha/beta 得到三相相电压指令。
     * 2. 注入 offset，使最大相和最小相围绕母线中点对称。
     * 3. 转换到 0..1 duty，并最终限幅。
     */
    const float v_u = v_alpha_v;
    const float v_v = -0.5f * v_alpha_v + 0.86602540378f * v_beta_v;
    const float v_w = -0.5f * v_alpha_v - 0.86602540378f * v_beta_v;
    const float v_offset = -0.5f * (foc_max3(v_u, v_v, v_w) + foc_min3(v_u, v_v, v_w));

    *duty_u = foc_clamp(0.5f + (v_u + v_offset) / vbus_v, 0.0f, 1.0f);
    *duty_v = foc_clamp(0.5f + (v_v + v_offset) / vbus_v, 0.0f, 1.0f);
    *duty_w = foc_clamp(0.5f + (v_w + v_offset) / vbus_v, 0.0f, 1.0f);
}

float foc_lpf(float previous, float input, float alpha)
{
    /*
     * 一阶 IIR：y[n] = y[n-1] + alpha * (x[n] - y[n-1])。
     * alpha=0 表示完全保持旧值，alpha=1 表示不滤波直接使用新输入。
     * 实际项目中 alpha 通常由采样周期 dt 和目标截止频率计算，而不是手填常数。
     */
    const float a = foc_clamp(alpha, 0.0f, 1.0f);
    return previous + a * (input - previous);
}
