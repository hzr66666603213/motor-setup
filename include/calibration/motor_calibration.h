#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

/*
 * motor_calibration.h
 *
 * 电机和编码器非阻塞校准流程。
 * 运行频率：建议 1 kHz 状态机任务。
 * 原则：不使用 delay，不阻塞 ISR，每次 update 只推进一小步。
 */

#include <stdbool.h>
#include <stdint.h>
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALIB_STEP_IDLE = 0,           /* 未运行校准 */
    CALIB_STEP_CURRENT_OFFSET,     /* 电流采样零偏校准 */
    CALIB_STEP_PHASE_RESISTANCE,   /* 相电阻测量 */
    CALIB_STEP_PHASE_INDUCTANCE,   /* 相电感测量 */
    CALIB_STEP_ENCODER_OFFSET,     /* 编码器零位校准 */
    CALIB_STEP_ENCODER_DIRECTION,  /* 编码器方向判断 */
    CALIB_STEP_DONE,               /* 校准成功完成 */
    CALIB_STEP_FAILED              /* 校准失败 */
} CalibrationStep;

typedef enum {
    CALIB_OK = 0,                  /* 校准成功 */
    CALIB_RUNNING,                 /* 校准进行中 */
    CALIB_ERROR_TIMEOUT,           /* 步骤超时 */
    CALIB_ERROR_OVERCURRENT,       /* 校准电流超过安全阈值 */
    CALIB_ERROR_ENCODER,           /* 编码器错误 */
    CALIB_ERROR_INVALID_RESULT     /* 结果不可信 */
} CalibrationResult;

typedef struct {
    float current_offset_time_s;     /* 电流零偏采样时间，s */
    float resistance_test_voltage_v; /* 电阻测量测试电压，V */
    float inductance_pulse_voltage_v;/* 电感测量脉冲电压，V */
    float encoder_calib_current_a;   /* 编码器校准锁定电流，A */
    float max_calibration_current_a; /* 校准最大允许电流，A */
    float max_step_time_s;           /* 单步骤最大时间，s */
    uint16_t offset_sample_count;    /* 零偏采样点数，samples */
} MotorCalibrationConfig;

typedef struct {
    CalibrationStep step;            /* 当前校准步骤 */
    CalibrationResult result;        /* 当前校准结果 */
    float elapsed_s;                 /* 当前步骤累计时间，s */
    uint16_t samples;                /* 已采样点数 */
    float accum_u;                   /* U 相累计原始值或中间量 */
    float accum_v;                   /* V 相累计原始值或中间量 */
    float accum_w;                   /* W 相累计原始值或中间量 */
    float start_angle_rad;           /* 方向判断起始机械角，rad */
    float measured_current_a;        /* 校准过程中测得的电流，A */
} MotorCalibrationContext;

/* 写入保守默认校准参数。 */
void motor_calibration_set_defaults(MotorCalibrationConfig *config);
/* 启动校准流程，可指定第一个步骤。 */
void motor_calibration_start(MotorCalibrationContext *ctx, CalibrationStep first_step);
/* 周期推进校准状态机；返回 RUNNING/OK/错误。 */
CalibrationResult motor_calibration_update(Axis *axis,
                                           MotorCalibrationContext *ctx,
                                           const MotorCalibrationConfig *config,
                                           float dt_s);
/* 判断流程是否已经结束。 */
bool motor_calibration_is_finished(const MotorCalibrationContext *ctx);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIBRATION_H */
