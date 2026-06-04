#ifndef PROTECTION_H
#define PROTECTION_H

/*
 * protection.h
 *
 * 保护逻辑接口。
 * 快速保护可在 PWM ISR 中调用，慢速保护在后台/1 kHz/100 Hz 任务中调用。
 */

#include <stdint.h>
#include "app/axis0_types.h"
#include "drivers/drv8301.h"
#include "drivers/encoder_mt6701_abz.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float bus_undervoltage_v;       /* 母线欠压阈值，V */
    float bus_overvoltage_v;        /* 母线过压阈值，V */
    float phase_overcurrent_a;      /* 相电流过流阈值，A */
    float mos_overtemperature_c;    /* MOS/板温过温阈值，degC */
    float motor_overtemperature_c;  /* 电机温度过温阈值，degC */
    float encoder_jump_rad;         /* 慢速采样间隔内允许的角度跳变，rad */
    uint16_t saturation_limit_count;/* 输出连续饱和允许次数，慢速样本数 */
} ProtectionConfig;

/* 设置默认保护阈值；产品化时应由参数或硬件配置覆盖。 */
void protection_set_defaults(ProtectionConfig *config);
/* 快速保护检查，允许在 20 kHz ISR 中调用。 */
void protection_check_fast(Axis *axis);
/* 慢速保护检查，后台或低频任务调用。 */
void protection_check_slow(Axis *axis, const ProtectionConfig *config);
/* 记录控制输出是否饱和，用于长时间饱和故障判断。 */
void protection_note_output_saturation(Axis *axis, const ProtectionConfig *config, int saturated);

/* ODrive v3.6 Axis0 快速保护：可在 20 kHz ISR 中调用。 */
void axis0_protection_check_fast(Axis0Context *axis,
                                 const EncoderMt6701AbzState *encoder,
                                 const Drv8301 *drv);

/* ODrive v3.6 Axis0 慢速保护：1kHz 或后台调用。 */
void axis0_protection_check_slow(Axis0Context *axis,
                                 const EncoderMt6701AbzState *encoder,
                                 const Drv8301 *drv);

#ifdef __cplusplus
}
#endif

#endif /* PROTECTION_H */
