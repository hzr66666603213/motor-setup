#include "drivers/encoder_mt6701_abz.h"

#include "foc/foc_math.h"

/*
 * encoder_mt6701_abz.c
 *
 * MT6701 ABZ 增量编码器 skeleton。
 * 当前使用 mock 计数变量；移植到 STM32F405 时应：
 * - 将 Encoder0 A/B 配置到 TIM3 Encoder Mode。
 * - 在 encoder_mt6701_abz_read_raw_count() 中读取 TIM3->CNT 并做有符号展开。
 * - 如果使用 Z/index，在 EXTI 或定时器输入捕获中置位 index_found。
 *
 * ABZ 与绝对角的区别：
 * - ABZ 只给相对位移计数，上电时不知道电机真实机械角。
 * - MT6701 即使内部知道绝对角，ABZ 输出链路也不会自动给出绝对零位。
 * - 因此第一阶段每次上电都需要 encoder offset calibration。
 *
 * 速度估算：
 * - 用本周期 count 差分 / dt 得到 raw velocity。
 * - 再做一阶低通，降低 ABZ 量化噪声。
 * - 20kHz ISR 中除法较贵，所以 cpr 的倒数 s_inv_cpr 预先计算。
 */

static int32_t s_mock_timer_count = 0;
static int32_t s_prev_count = 0;
static float s_inv_cpr = 1.0f / 4096.0f;

void encoder_mt6701_abz_set_default_config(EncoderMt6701AbzConfig *config)
{
    /*
     * MT6701 ABZ 默认常见配置为 1024 PPR。
     * 四倍频计数后 CPR = PPR * 4 = 4096。
     * 具体值可通过 MT6701 配置或实际模块资料确认。
     */
    config->ppr = 1024;
    config->cpr = 4096;
    config->direction = 1;
    config->offset_rad = 0.0f;
    config->velocity_lpf_alpha = 0.1f;
}

bool encoder_mt6701_abz_init(EncoderMt6701AbzState *state, const EncoderMt6701AbzConfig *config)
{
    if (config->cpr <= 0) {
        /* CPR 必须为正，否则角度换算会除以零。 */
        return false;
    }

    /* 初始化软件状态；真实 TIM 计数器初始化在 board/HAL 层完成。 */
    state->raw_count = 0;
    state->cpr = config->cpr;
    state->mechanical_angle_rad = 0.0f;
    state->velocity_rad_s = 0.0f;
    state->offset_rad = config->offset_rad;
    state->direction = (config->direction >= 0) ? 1 : -1;
    state->index_found = false;
    state->valid = true;

    s_inv_cpr = 1.0f / (float)config->cpr;
    s_prev_count = 0;
    return true;
}

void encoder_mt6701_abz_update(EncoderMt6701AbzState *state, float dt_s)
{
    /*
     * 读取硬件计数。
     * 真实实现要注意：
     * - TIM3->CNT 可能是 16 bit，需要做溢出展开；
     * - 方向由硬件编码器模式和软件 direction 共同决定；
     * - 如果 AB 相接反，可以通过 direction=-1 先修正。
     */
    const int32_t raw = encoder_mt6701_abz_read_raw_count();
    const int32_t delta = raw - s_prev_count;
    s_prev_count = raw;

    state->raw_count = raw;

    /*
     * mechanical_angle = count / cpr * 2*pi。
     * direction 和 offset 在这里统一处理，输出保持 0..2pi。
     * offset_rad 是机械零偏，不是电角度零偏；电角度 offset 在 FOC 中结合 pole_pairs 使用。
     */
    const float count_angle = (float)(state->direction * raw) * s_inv_cpr * FOC_TWO_PI_F;
    state->mechanical_angle_rad = foc_wrap_0_2pi(count_angle - state->offset_rad);

    if (dt_s > 0.0f) {
        /*
         * 速度估算单位：
         * delta_count * (2*pi / cpr) / dt = rad/s。
         */
        const float raw_vel = (float)(state->direction * delta) * s_inv_cpr * FOC_TWO_PI_F / dt_s;
        state->velocity_rad_s = foc_lpf(state->velocity_rad_s, raw_vel, 0.1f);
    }
}

void encoder_mt6701_abz_clear_count(EncoderMt6701AbzState *state)
{
    /*
     * 清零计数只适合 IDLE/校准前使用。
     * 闭环运行中清零会造成角度突变，必须禁止。
     */
    s_mock_timer_count = 0;
    s_prev_count = 0;
    state->raw_count = 0;
    state->mechanical_angle_rad = 0.0f;
    state->velocity_rad_s = 0.0f;
}

void encoder_mt6701_abz_set_cpr(EncoderMt6701AbzState *state, int32_t cpr)
{
    if (cpr > 0) {
        /* CPR 变化后立即更新倒数，减少 ISR 中重复除法。 */
        state->cpr = cpr;
        s_inv_cpr = 1.0f / (float)cpr;
    }
}

void encoder_mt6701_abz_set_offset(EncoderMt6701AbzState *state, float offset_rad)
{
    /* offset 统一归一化，避免长期运行时角度漂到很大。 */
    state->offset_rad = foc_wrap_0_2pi(offset_rad);
}

void encoder_mt6701_abz_set_direction(EncoderMt6701AbzState *state, int direction)
{
    state->direction = (direction >= 0) ? 1 : -1;
}

int32_t encoder_mt6701_abz_read_raw_count(void)
{
    /*
     * mock 返回静态变量。
     * STM32 移植时这里应读取 TIM3 编码器计数，并按需要做 int32 展开。
     */
    return s_mock_timer_count;
}
