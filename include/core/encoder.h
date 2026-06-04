#ifndef ENCODER_H
#define ENCODER_H

/*
 * encoder.h
 *
 * 编码器抽象接口。
 * 目标是把 SPI/ABI/Hall 等不同后端统一成机械角 rad、电角度 rad 和速度 rad/s。
 * 快速路径可被 20 kHz ISR 调用，因此必须非阻塞。
 */

#include <stdbool.h>
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化编码器状态；真实后端应完成外设配置和初始读数检查。 */
bool encoder_init(EncoderState *encoder);
/* 快速采样并更新角度/速度；20 kHz ISR 可调用，禁止阻塞。 */
bool encoder_sample_fast(EncoderState *encoder, const MotorConfig *motor, float dt_s);
/* 判断编码器是否已准备好用于闭环。 */
bool encoder_is_ready(const EncoderState *encoder);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
