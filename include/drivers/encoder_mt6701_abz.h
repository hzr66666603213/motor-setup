#ifndef ENCODER_MT6701_ABZ_H
#define ENCODER_MT6701_ABZ_H

/*
 * encoder_mt6701_abz.h
 *
 * MT6701 磁编码器 ABZ 增量输出驱动。
 * 第一阶段使用 ODrive Encoder0 A/B/Z 接口，不使用 SSI。
 * A/B 由 STM32 定时器 Encoder Mode 计数，Z/index 可选。
 *
 * 重要限制：
 * - ABZ 是增量反馈，上电绝对角度未知。
 * - 如果未使用 Z/index，每次上电都必须重新做电角度零位校准。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t raw_count;              /* 当前原始计数，count */
    int32_t cpr;                    /* 每机械圈计数，count/rev，默认 4096 */
    float mechanical_angle_rad;     /* 机械角，rad，范围 0..2pi */
    float velocity_rad_s;           /* 机械速度，rad/s，低通后 */
    float offset_rad;               /* 机械零偏，rad */
    int direction;                  /* 方向，+1 或 -1 */
    bool index_found;               /* 是否检测到 Z/index */
    bool valid;                     /* 编码器数据是否可用于闭环 */
} EncoderMt6701AbzState;

typedef struct {
    int32_t ppr;                    /* MT6701 ABZ PPR，默认 1024 */
    int32_t cpr;                    /* 四倍频 CPR，默认 4096 */
    int direction;                  /* 初始方向，+1 或 -1 */
    float offset_rad;               /* 初始零偏，rad */
    float velocity_lpf_alpha;       /* 速度低通系数，0..1 */
} EncoderMt6701AbzConfig;

void encoder_mt6701_abz_set_default_config(EncoderMt6701AbzConfig *config);
bool encoder_mt6701_abz_init(EncoderMt6701AbzState *state, const EncoderMt6701AbzConfig *config);
void encoder_mt6701_abz_update(EncoderMt6701AbzState *state, float dt_s);
void encoder_mt6701_abz_clear_count(EncoderMt6701AbzState *state);
void encoder_mt6701_abz_set_cpr(EncoderMt6701AbzState *state, int32_t cpr);
void encoder_mt6701_abz_set_offset(EncoderMt6701AbzState *state, float offset_rad);
void encoder_mt6701_abz_set_direction(EncoderMt6701AbzState *state, int direction);
int32_t encoder_mt6701_abz_read_raw_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_MT6701_ABZ_H */
