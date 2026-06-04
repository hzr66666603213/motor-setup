#ifndef AXIS0_CALIBRATION_H
#define AXIS0_CALIBRATION_H

/*
 * calibration.h
 *
 * ODrive v3.6 Axis0 + 2804 + MT6701 ABZ 非阻塞校准流程。
 * 所有步骤由状态机和时间推进，不使用 delay。
 */

#include "app/axis0_types.h"
#include "drivers/current_sensor.h"
#include "drivers/encoder_mt6701_abz.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALIB_CURRENT_OFFSET = 0,
    CALIB_RESISTANCE,
    CALIB_INDUCTANCE,
    CALIB_ENCODER_DIRECTION,
    CALIB_ENCODER_OFFSET,
    CALIB_DONE,
    CALIB_FAILED
} Axis0CalibrationStep;

typedef enum {
    CALIB_ERROR_NONE = 0,
    CALIB_ERROR_TIMEOUT,
    CALIB_ERROR_OVERCURRENT,
    CALIB_ERROR_OFFSET_NOISE,
    CALIB_ERROR_ENCODER_NO_MOVEMENT,
    CALIB_ERROR_INVALID_RESULT
} Axis0CalibrationError;

typedef struct {
    Axis0CalibrationStep step;       /* 当前步骤 */
    Axis0CalibrationError error;     /* 错误码 */
    float step_elapsed_s;            /* 当前步骤已用时间，s */
    uint32_t sample_count;           /* 采样点数 */
    float accum_a;                   /* 累计值 A 或 count */
    float accum_b;                   /* 累计值 B 或 count */
    float accum_c;                   /* 累计值 C 或 count */
    float max_offset_span_count;     /* 零偏允许波动，count */
    int32_t start_encoder_count;     /* 编码器方向判断起始计数 */
} Axis0CalibrationContext;

void axis0_calibration_start(Axis0CalibrationContext *calib, Axis0CalibrationStep first_step);
void axis0_calibration_update(Axis0Context *axis,
                              Axis0CalibrationContext *calib,
                              CurrentSensorConfig *current_sensor,
                              EncoderMt6701AbzState *encoder,
                              float dt_s);
bool axis0_calibration_finished(const Axis0CalibrationContext *calib);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_CALIBRATION_H */
