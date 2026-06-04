#include "core/encoder.h"
#include "foc/foc_math.h"

/*
 * encoder.c
 *
 * 编码器 mock/stub 实现。
 * 当前只根据 raw_angle_rad、direction、offset 计算机械角和电角度。
 * 移植时应在具体后端中实现 SPI 读角、ABI 计数器展开、Hall 插值等逻辑。
 */

bool encoder_init(EncoderState *encoder)
{
    /* 默认认为 mock 编码器可用，便于框架流程跑通。 */
    encoder->raw_count = 0u;
    encoder->raw_angle_rad = 0.0f;
    encoder->mechanical_angle_rad = 0.0f;
    encoder->electrical_angle_rad = 0.0f;
    encoder->velocity_rad_s = 0.0f;
    encoder->direction = ENCODER_DIR_UNKNOWN;
    encoder->offset_rad = 0.0f;
    encoder->is_ready = true;
    encoder->has_error = false;
    return true;
}

bool encoder_sample_fast(EncoderState *encoder, const MotorConfig *motor, float dt_s)
{
    (void)dt_s;
    /*
     * 快速路径只做简单换算。
     * 真实工程中速度可由差分、PLL 或观测器估计，并要处理角度跨 2*pi。
     */
    float sign = (encoder->direction == ENCODER_DIR_NEGATIVE) ? -1.0f : 1.0f;
    encoder->mechanical_angle_rad = foc_wrap_0_2pi(sign * encoder->raw_angle_rad - encoder->offset_rad);
    encoder->electrical_angle_rad = foc_electrical_angle(encoder->mechanical_angle_rad, motor->pole_pairs, 0.0f);
    return !encoder->has_error;
}

bool encoder_is_ready(const EncoderState *encoder)
{
    /* 闭环使能前必须同时满足 ready 且无错误。 */
    return encoder->is_ready && !encoder->has_error;
}
