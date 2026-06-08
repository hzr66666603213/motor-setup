#include "drivers/encoder_mt6701_abz.h"

#include "foc/foc_math.h"
#include "hal/hal_encoder.h"

/*
 * encoder_mt6701_abz.c
 *
 * MT6701 ABZ 增量编码器驱动。
 *
 * 职责：
 * - 将 Encoder0 的累计 count 转换成机械角度 rad；
 * - 根据 count 差分估算机械速度 rad/s；
 * - 支持 CPR、方向、机械零偏配置；
 * - 不直接访问 TIM3，不包含 stm32f4xx_hal.h。
 *
 * 调用频率：
 * - encoder_mt6701_abz_update() 可在 20 kHz ISR 中调用，也可在 1 kHz
 *   编码器任务中调用。第一版为了降低 ISR 压力，推荐 1 kHz 更新速度，
 *   ISR 使用最新角度快照。
 *
 * 重要限制：
 * - ABZ 是增量反馈，上电后绝对机械角未知；
 * - 如果没有使用 Z/index，每次上电都要做 encoder offset calibration；
 * - TIM3 16-bit 溢出展开由 hal_encoder0_get_count() 的 STM32F405 后端完成。
 */

static int32_t s_prev_count = 0;
static float s_inv_cpr = 1.0f / 4096.0f;

void encoder_mt6701_abz_set_default_config(EncoderMt6701AbzConfig *config)
{
    if (config == 0) {
        return;
    }

    config->ppr = 1024;
    config->cpr = 4096;
    config->direction = 1;
    config->offset_rad = 0.0f;
    config->velocity_lpf_alpha = 0.1f;
}

bool encoder_mt6701_abz_init(EncoderMt6701AbzState *state, const EncoderMt6701AbzConfig *config)
{
    if (state == 0 || config == 0 || config->cpr <= 0) {
        return false;
    }

    if (!hal_encoder0_init()) {
        return false;
    }

    state->raw_count = 0;
    state->cpr = config->cpr;
    state->mechanical_angle_rad = 0.0f;
    state->velocity_rad_s = 0.0f;
    state->offset_rad = foc_wrap_0_2pi(config->offset_rad);
    state->direction = (config->direction >= 0) ? 1 : -1;
    state->index_found = hal_encoder0_index_found();
    state->valid = true;

    s_inv_cpr = 1.0f / (float)config->cpr;
    s_prev_count = 0;
    return true;
}

void encoder_mt6701_abz_update(EncoderMt6701AbzState *state, float dt_s)
{
    if (state == 0 || state->cpr <= 0) {
        return;
    }

    const int32_t raw = encoder_mt6701_abz_read_raw_count();
    const int32_t delta = raw - s_prev_count;
    s_prev_count = raw;

    state->raw_count = raw;
    state->index_found = hal_encoder0_index_found();

    /*
     * mechanical_angle = count / cpr * 2pi。
     * direction 用于修正 A/B 相序或安装方向；offset_rad 是机械角零偏。
     */
    const float count_angle = (float)(state->direction * raw) * s_inv_cpr * FOC_TWO_PI_F;
    state->mechanical_angle_rad = foc_wrap_0_2pi(count_angle - state->offset_rad);

    if (dt_s > 0.0f) {
        const float raw_vel = (float)(state->direction * delta) * s_inv_cpr * FOC_TWO_PI_F / dt_s;
        state->velocity_rad_s = foc_lpf(state->velocity_rad_s, raw_vel, 0.1f);
    }
}

void encoder_mt6701_abz_clear_count(EncoderMt6701AbzState *state)
{
    hal_encoder0_reset_count();
    s_prev_count = 0;

    if (state != 0) {
        state->raw_count = 0;
        state->mechanical_angle_rad = 0.0f;
        state->velocity_rad_s = 0.0f;
        state->index_found = false;
    }
}

void encoder_mt6701_abz_set_cpr(EncoderMt6701AbzState *state, int32_t cpr)
{
    if (state != 0 && cpr > 0) {
        state->cpr = cpr;
        s_inv_cpr = 1.0f / (float)cpr;
    }
}

void encoder_mt6701_abz_set_offset(EncoderMt6701AbzState *state, float offset_rad)
{
    if (state != 0) {
        state->offset_rad = foc_wrap_0_2pi(offset_rad);
    }
}

void encoder_mt6701_abz_set_direction(EncoderMt6701AbzState *state, int direction)
{
    if (state != 0) {
        state->direction = (direction >= 0) ? 1 : -1;
    }
}

int32_t encoder_mt6701_abz_read_raw_count(void)
{
    return hal_encoder0_get_count();
}
