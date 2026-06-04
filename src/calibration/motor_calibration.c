#include "calibration/motor_calibration.h"

#include <math.h>
#include "core/current_sensor.h"
#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"

/*
 * motor_calibration.c
 *
 * 非阻塞校准流程骨架。
 * 这里保留物理步骤和安全限制，但不绑定具体 PWM 电压矢量注入方式。
 * 工程化时可把“施加固定电压/电流”的动作接入开环 FOC 或专用校准输出函数。
 */

static CalibrationResult fail(MotorCalibrationContext *ctx, CalibrationResult result)
{
    /* 校准失败必须立即关闭功率级，避免电机继续受激。 */
    ctx->step = CALIB_STEP_FAILED;
    ctx->result = result;
    hal_pwm_disable();
    hal_gpio_set_gate_enable(false);
    return result;
}

void motor_calibration_set_defaults(MotorCalibrationConfig *config)
{
    /* 默认值偏保守，适合首次 bring-up；实际项目应按电机和功率级能力调整。 */
    config->current_offset_time_s = 0.1f;
    config->resistance_test_voltage_v = 1.0f;
    config->inductance_pulse_voltage_v = 2.0f;
    config->encoder_calib_current_a = 2.0f;
    config->max_calibration_current_a = 5.0f;
    config->max_step_time_s = 3.0f;
    config->offset_sample_count = 2000u;
}

void motor_calibration_start(MotorCalibrationContext *ctx, CalibrationStep first_step)
{
    /* 重置上下文，保证每次校准从干净状态开始。 */
    ctx->step = first_step;
    ctx->result = CALIB_RUNNING;
    ctx->elapsed_s = 0.0f;
    ctx->samples = 0u;
    ctx->accum_u = 0.0f;
    ctx->accum_v = 0.0f;
    ctx->accum_w = 0.0f;
    ctx->start_angle_rad = 0.0f;
    ctx->measured_current_a = 0.0f;
}

static void next_step(MotorCalibrationContext *ctx, CalibrationStep step)
{
    /* 切换步骤时清零该步骤的计时和累计量。 */
    ctx->step = step;
    ctx->elapsed_s = 0.0f;
    ctx->samples = 0u;
    ctx->accum_u = 0.0f;
    ctx->accum_v = 0.0f;
    ctx->accum_w = 0.0f;
}

CalibrationResult motor_calibration_update(Axis *axis,
                                           MotorCalibrationContext *ctx,
                                           const MotorCalibrationConfig *config,
                                           float dt_s)
{
    ctx->elapsed_s += dt_s;
    if (ctx->elapsed_s > config->max_step_time_s) {
        return fail(ctx, CALIB_ERROR_TIMEOUT);
    }

    if (ctx->step == CALIB_STEP_CURRENT_OFFSET) {
        HalAdcPhaseRaw raw;
        /* 电流零偏校准必须在 PWM/gate 关闭时进行，保证相电流真实为 0 A。 */
        hal_pwm_disable();
        hal_gpio_set_gate_enable(false);
        if (hal_adc_get_phase_current_raw(&raw)) {
            ctx->accum_u += (float)raw.u;
            ctx->accum_v += (float)raw.v;
            ctx->accum_w += (float)raw.w;
            ctx->samples++;
        }
        if (ctx->samples >= config->offset_sample_count) {
            /* 多次采样平均，降低 ADC 噪声对零偏的影响。 */
            axis->foc_config.current_offset_u_count = ctx->accum_u / (float)ctx->samples;
            axis->foc_config.current_offset_v_count = ctx->accum_v / (float)ctx->samples;
            axis->foc_config.current_offset_w_count = ctx->accum_w / (float)ctx->samples;
            next_step(ctx, CALIB_STEP_PHASE_RESISTANCE);
        }
    } else if (ctx->step == CALIB_STEP_PHASE_RESISTANCE) {
        /*
         * 物理意义：施加一个小的静止电压矢量，等待电流接近稳态后估算 R = V/I。
         * 注意：真实实现应使用受限电流/电压闭环，避免低电阻电机电流过大。
         */
        hal_gpio_set_gate_enable(true);
        hal_pwm_enable();
        hal_pwm_set_duty(0.52f, 0.48f, 0.48f);
        ctx->measured_current_a = fabsf(axis->foc_state.ia_a);
        if (ctx->measured_current_a > config->max_calibration_current_a) {
            return fail(ctx, CALIB_ERROR_OVERCURRENT);
        }
        if (ctx->elapsed_s > 0.2f && ctx->measured_current_a > 0.05f) {
            axis->motor.phase_resistance_ohm = config->resistance_test_voltage_v / ctx->measured_current_a;
            next_step(ctx, CALIB_STEP_PHASE_INDUCTANCE);
        }
    } else if (ctx->step == CALIB_STEP_PHASE_INDUCTANCE) {
        /*
         * 物理意义：施加短电压脉冲，测量电流斜率 di/dt，近似 L = V / (di/dt)。
         * 当前骨架只预留非阻塞时间片，后续应记录脉冲前后电流。
         */
        if (ctx->elapsed_s > 0.05f) {
            axis->motor.phase_inductance_h = 0.00005f;
            next_step(ctx, CALIB_STEP_DONE);
        }
    } else if (ctx->step == CALIB_STEP_ENCODER_OFFSET) {
        /*
         * 物理意义：用固定 d 轴电流或开环电角度把转子锁定到已知电角度，
         * 再记录机械角关系作为 encoder_offset。
         */
        hal_gpio_set_gate_enable(true);
        hal_pwm_enable();
        if (ctx->elapsed_s > 0.5f) {
            axis->encoder.offset_rad = axis->encoder.mechanical_angle_rad;
            next_step(ctx, CALIB_STEP_ENCODER_DIRECTION);
        }
    } else if (ctx->step == CALIB_STEP_ENCODER_DIRECTION) {
        /*
         * 物理意义：开环正向旋转电角度，观察机械角变化方向，
         * 从而判断编码器方向与电机相序是否一致。
         */
        if (ctx->samples == 0u) {
            ctx->start_angle_rad = axis->encoder.mechanical_angle_rad;
            ctx->samples = 1u;
        }
        if (ctx->elapsed_s > 0.3f) {
            axis->encoder.direction = (axis->encoder.mechanical_angle_rad >= ctx->start_angle_rad)
                                    ? ENCODER_DIR_POSITIVE
                                    : ENCODER_DIR_NEGATIVE;
            next_step(ctx, CALIB_STEP_DONE);
        }
    } else if (ctx->step == CALIB_STEP_DONE) {
        ctx->result = CALIB_OK;
        hal_pwm_disable();
        return CALIB_OK;
    } else if (ctx->step == CALIB_STEP_FAILED) {
        return ctx->result;
    }

    return CALIB_RUNNING;
}

bool motor_calibration_is_finished(const MotorCalibrationContext *ctx)
{
    return (ctx->step == CALIB_STEP_DONE) || (ctx->step == CALIB_STEP_FAILED);
}
