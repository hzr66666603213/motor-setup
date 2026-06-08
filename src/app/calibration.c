#include "app/calibration.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"

/*
 * calibration.c
 *
 * ODrive v3.6 Axis0 + 2804 + MT6701 ABZ 非阻塞校准流程。
 *
 * 职责：
 * - 电流采样零偏校准；
 * - 低压相电阻估算；
 * - 低压短脉冲相电感估算；
 * - ABZ 编码器方向判断；
 * - 电角度零位 offset 校准。
 *
 * 调用频率：
 * - 由 axis_update_1khz() 或后台状态机以 1 kHz 调用；
 * - 不使用 delay，不做阻塞通信；
 * - 真正的 PWM 输出由 board/hal 层完成。
 *
 * 安全策略：
 * - 第一次上 2804 小电机，只从 0.05V~0.10V 开始；
 * - 校准电流超过 calibration_current_a 立即失败并关闭功率级；
 * - 每一步有超时；
 * - 校准完成后保持功率级关闭，必须由用户显式请求闭环。
 */

#define CALIB_OFFSET_SAMPLES          2000u
#define CALIB_STEP_TIMEOUT_S          3.0f
#define CALIB_ENCODER_MIN_DELTA_COUNT 16
#define CALIB_RESISTANCE_START_V      0.05f
#define CALIB_INDUCTANCE_START_V      0.05f
#define CALIB_INJECTION_MAX_V         0.10f
#define CALIB_MIN_MEASURE_CURRENT_A   0.02f
#define CALIB_RESISTANCE_SETTLE_S     0.25f
#define CALIB_INDUCTANCE_PULSE_S      0.002f
#define CALIB_ENCODER_ROTATE_HZ       0.25f
#define CALIB_ENCODER_LOCK_S          0.50f

static float calib_abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float calib_limit_injection_voltage(const Axis0Context *axis, float requested_v)
{
    float limit_v = axis->config.motor.voltage_limit_v;
    if (limit_v > CALIB_INJECTION_MAX_V) {
        limit_v = CALIB_INJECTION_MAX_V;
    }
    if (requested_v > limit_v) {
        return limit_v;
    }
    if (requested_v < -limit_v) {
        return -limit_v;
    }
    return requested_v;
}

static void calib_fail(Axis0Context *axis, Axis0CalibrationContext *calib, Axis0CalibrationError error)
{
    calib->step = CALIB_FAILED;
    calib->error = error;
    board_disable_axis0_power_stage(axis);
}

static bool calib_current_within_limit(const Axis0Context *axis)
{
    const float limit = axis->config.motor.calibration_current_a;
    return calib_abs(axis->rt.ia_a) <= limit &&
           calib_abs(axis->rt.ib_a) <= limit &&
           calib_abs(axis->rt.ic_a) <= limit;
}

static bool calib_power_stage_active(void)
{
    const BoardOdriveV36Status status = board_get_status();
    return status.drv_gate_enabled && !status.pwm_disabled && !status.drv_nfault_active;
}

static void axis0_apply_open_loop_voltage(Axis0Context *axis, float v_alpha_v, float v_beta_v)
{
    /*
     * alpha/beta 是静止坐标系电压矢量。
     * 本函数只负责把低压矢量转成 SVPWM duty，不做闭环电流控制。
     */
    float duty_a = 0.5f;
    float duty_b = 0.5f;
    float duty_c = 0.5f;

    foc_svpwm(v_alpha_v, v_beta_v, axis->rt.vbus_v, &duty_a, &duty_b, &duty_c);
    axis->rt.v_alpha_v = v_alpha_v;
    axis->rt.v_beta_v = v_beta_v;
    axis->rt.duty_a = duty_a;
    axis->rt.duty_b = duty_b;
    axis->rt.duty_c = duty_c;
    board_axis0_set_pwm_duty(duty_a, duty_b, duty_c);
}

static void axis0_apply_voltage_in_electrical_frame(Axis0Context *axis,
                                                    float vd_v,
                                                    float vq_v,
                                                    float electrical_angle_rad)
{
    /*
     * d/q 是随转子电角度旋转的坐标系：
     * - d 轴对齐转子磁链，常用于锁定转子或施加校准磁场；
     * - q 轴产生转矩，校准阶段默认不用大 q 轴指令。
     */
    float v_alpha_v = 0.0f;
    float v_beta_v = 0.0f;

    vd_v = calib_limit_injection_voltage(axis, vd_v);
    vq_v = calib_limit_injection_voltage(axis, vq_v);
    foc_limit_voltage(&vd_v, &vq_v, CALIB_INJECTION_MAX_V);
    foc_inv_park(vd_v, vq_v, electrical_angle_rad, &v_alpha_v, &v_beta_v);

    axis->rt.vd_v = vd_v;
    axis->rt.vq_v = vq_v;
    axis0_apply_open_loop_voltage(axis, v_alpha_v, v_beta_v);
}

void axis0_calibration_start(Axis0CalibrationContext *calib, Axis0CalibrationStep first_step)
{
    calib->step = first_step;
    calib->error = CALIB_ERROR_NONE;
    calib->step_elapsed_s = 0.0f;
    calib->sample_count = 0u;
    calib->accum_a = 0.0f;
    calib->accum_b = 0.0f;
    calib->accum_c = 0.0f;
    calib->max_offset_span_count = 20.0f;
    calib->resistance_test_voltage_v = CALIB_RESISTANCE_START_V;
    calib->inductance_pulse_voltage_v = CALIB_INDUCTANCE_START_V;
    calib->start_encoder_count = 0;
}

static void calib_next(Axis0CalibrationContext *calib, Axis0CalibrationStep next)
{
    calib->step = next;
    calib->step_elapsed_s = 0.0f;
    calib->sample_count = 0u;
    calib->accum_a = 0.0f;
    calib->accum_b = 0.0f;
    calib->accum_c = 0.0f;
}

void axis0_calibration_update(Axis0Context *axis,
                              Axis0CalibrationContext *calib,
                              CurrentSensorConfig *current_sensor,
                              EncoderMt6701AbzState *encoder,
                              float dt_s)
{
    calib->step_elapsed_s += dt_s;
    axis->rt.vbus_v = board_read_vbus_v();

    if (calib->step_elapsed_s > CALIB_STEP_TIMEOUT_S) {
        calib_fail(axis, calib, CALIB_ERROR_TIMEOUT);
        return;
    }

    if (calib->step == CALIB_CURRENT_OFFSET) {
        uint16_t raw_a = 0u;
        uint16_t raw_b = 0u;
        uint16_t raw_c = 0u;

        /*
         * 电流零偏：功率级关闭，采集“真实 0A”下 ADC offset。
         * 这一步完成后只进入 CALIB_DONE，不自动继续测电阻。
         */
        if (board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c)) {
            calib->accum_a += (float)raw_a;
            calib->accum_b += (float)raw_b;
            calib->accum_c += (float)raw_c;
            calib->sample_count++;
        }

        if (calib->sample_count >= CALIB_OFFSET_SAMPLES) {
            current_sensor->offset_a_count = calib->accum_a / (float)calib->sample_count;
            current_sensor->offset_b_count = calib->accum_b / (float)calib->sample_count;
            current_sensor->offset_c_count = calib->accum_c / (float)calib->sample_count;
            axis->current_offset_valid = true;
            calib_next(calib, CALIB_DONE);
        }
    } else if (calib->step == CALIB_RESISTANCE) {
        /*
         * 相电阻估算：施加很小的 d 轴电压，等待电流接近稳态后用 R=V/I。
         * 这不是高精度实验室测量，只用于生成保守电流环初值。
         */
        if (!axis->current_offset_valid || !calib_power_stage_active()) {
            calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
            return;
        }
        if (!calib_current_within_limit(axis)) {
            calib_fail(axis, calib, CALIB_ERROR_OVERCURRENT);
            return;
        }

        axis0_apply_voltage_in_electrical_frame(axis,
                                                calib->resistance_test_voltage_v,
                                                0.0f,
                                                0.0f);

        if (calib->step_elapsed_s > CALIB_RESISTANCE_SETTLE_S) {
            const float id_abs = calib_abs(axis->rt.id_a);
            if (id_abs < CALIB_MIN_MEASURE_CURRENT_A) {
                calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
                return;
            }
            axis->config.motor.phase_resistance_ohm =
                calib_abs(calib->resistance_test_voltage_v) / id_abs;
            calib_next(calib, CALIB_INDUCTANCE);
        }
    } else if (calib->step == CALIB_INDUCTANCE) {
        /*
         * 相电感估算：记录脉冲前 id，施加短小 d 轴电压，根据 di/dt 估算 L。
         * 脉冲很短，所以必须每次 update 都检查过流。
         */
        if (!calib_power_stage_active()) {
            calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
            return;
        }
        if (!calib_current_within_limit(axis)) {
            calib_fail(axis, calib, CALIB_ERROR_OVERCURRENT);
            return;
        }

        if (calib->sample_count == 0u) {
            calib->accum_a = axis->rt.id_a; /* 脉冲开始电流，A */
            calib->sample_count = 1u;
        }

        axis0_apply_voltage_in_electrical_frame(axis,
                                                calib->inductance_pulse_voltage_v,
                                                0.0f,
                                                0.0f);

        if (calib->step_elapsed_s >= CALIB_INDUCTANCE_PULSE_S) {
            const float di_a = axis->rt.id_a - calib->accum_a;
            const float di_dt = di_a / calib->step_elapsed_s;
            if (calib_abs(di_dt) < 1.0f) {
                calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
                return;
            }
            axis->config.motor.phase_inductance_h =
                calib_abs(calib->inductance_pulse_voltage_v / di_dt);
            axis->motor_calibrated = true;
            calib_next(calib, CALIB_DONE);
        }
    } else if (calib->step == CALIB_ENCODER_DIRECTION) {
        /*
         * 编码器方向：开环电角度缓慢正转，观察 ABZ 计数变化方向。
         * 完成后进入 CALIB_ENCODER_OFFSET。
         */
        if (!calib_power_stage_active()) {
            calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
            return;
        }
        if (!calib_current_within_limit(axis)) {
            calib_fail(axis, calib, CALIB_ERROR_OVERCURRENT);
            return;
        }

        if (calib->sample_count == 0u) {
            calib->start_encoder_count = encoder->raw_count;
            calib->accum_a = 0.0f; /* 开环电角度，rad */
            calib->sample_count = 1u;
        }

        calib->accum_a = foc_wrap_0_2pi(calib->accum_a +
                                        FOC_TWO_PI_F * CALIB_ENCODER_ROTATE_HZ * dt_s);
        axis0_apply_voltage_in_electrical_frame(axis,
                                                calib->resistance_test_voltage_v,
                                                0.0f,
                                                calib->accum_a);

        if (calib->step_elapsed_s > 1.0f) {
            const int32_t delta = encoder->raw_count - calib->start_encoder_count;
            if (delta > CALIB_ENCODER_MIN_DELTA_COUNT) {
                axis->config.encoder.encoder_direction = 1;
                encoder->direction = 1;
            } else if (delta < -CALIB_ENCODER_MIN_DELTA_COUNT) {
                axis->config.encoder.encoder_direction = -1;
                encoder->direction = -1;
            } else {
                calib_fail(axis, calib, CALIB_ERROR_ENCODER_NO_MOVEMENT);
                return;
            }
            calib_next(calib, CALIB_ENCODER_OFFSET);
        }
    } else if (calib->step == CALIB_ENCODER_OFFSET) {
        /*
         * 电角度零位：用小 d 轴电压把转子锁到 electrical_angle=0。
         * 读取机械角后计算 encoder_offset，使后续 electrical_angle 能对齐。
         */
        if (!calib_power_stage_active()) {
            calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
            return;
        }
        if (!calib_current_within_limit(axis)) {
            calib_fail(axis, calib, CALIB_ERROR_OVERCURRENT);
            return;
        }

        axis0_apply_voltage_in_electrical_frame(axis,
                                                calib->resistance_test_voltage_v,
                                                0.0f,
                                                0.0f);

        if (calib->step_elapsed_s > CALIB_ENCODER_LOCK_S) {
            axis->config.encoder.encoder_offset_rad =
                foc_wrap_0_2pi(-encoder->mechanical_angle_rad *
                               (float)axis->config.motor.pole_pairs *
                               (float)axis->config.encoder.encoder_direction);
            encoder->offset_rad = axis->config.encoder.encoder_offset_rad;
            axis->encoder_calibrated = true;
            calib_next(calib, CALIB_DONE);
        }
    } else if (calib->step == CALIB_DONE) {
        board_disable_axis0_power_stage(axis);
    } else {
        calib_fail(axis, calib, CALIB_ERROR_INVALID_RESULT);
    }
}

bool axis0_calibration_finished(const Axis0CalibrationContext *calib)
{
    return calib != 0 && (calib->step == CALIB_DONE || calib->step == CALIB_FAILED);
}
