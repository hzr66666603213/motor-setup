#include "app/axis0_current_loop_isr.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"
#include "foc/svpwm.h"
#include "protection/fault.h"
#include "protection/protection.h"

/*
 * axis0_current_loop_isr.c
 *
 * Axis0 20 kHz 快速任务。
 *
 * 关键原则：
 * - 20 kHz ISR 可以更新测量量；
 * - 只有 CLOSED_LOOP_CONTROL 可以计算 PI 并写闭环 duty；
 * - MOTOR_CALIBRATION / ENCODER_CALIBRATION / OPEN_LOOP_VOLTAGE_TEST 的 duty
 *   由 1 kHz 校准/测试状态机写入，ISR 绝不能用 50% duty 覆盖它；
 * - IDLE / READY / FAULT / CURRENT_OFFSET / ENCODER_TEST / ADC_OFFSET_TEST
 *   不需要有效电压输出，保持安全 50% duty 或由状态机关闭功率级。
 */

static bool axis0_update_fast_measurements(Axis0IsrContext *ctx, float dt_s)
{
    Axis0Context *axis = ctx->axis;
    uint16_t raw_a = 0u;
    uint16_t raw_b = 0u;
    uint16_t raw_c = 0u;

    if (!board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c)) {
        set_fault(axis, AXIS0_FAULT_CURRENT_SENSOR_INVALID);
        axis0_fault_enter_safe_state(axis);
        return false;
    }

    const CurrentSensorSample current =
        current_sensor_convert_raw(ctx->current_sensor, raw_a, raw_b, raw_c);
    if (!current.valid) {
        set_fault(axis, AXIS0_FAULT_CURRENT_SENSOR_INVALID);
        axis0_fault_enter_safe_state(axis);
        return false;
    }

    axis->rt.ia_a = current.ia_a;
    axis->rt.ib_a = current.ib_a;
    axis->rt.ic_a = current.ic_a;
    axis->rt.vbus_v = board_read_vbus_v();

    encoder_mt6701_abz_update(ctx->encoder, dt_s);
    axis->rt.mechanical_angle_rad = ctx->encoder->mechanical_angle_rad;
    axis->rt.velocity_rad_s = ctx->encoder->velocity_rad_s;
    axis->rt.electrical_angle_rad =
        foc_electrical_angle_dir(axis->rt.mechanical_angle_rad,
                                 axis->config.motor.pole_pairs,
                                 axis->config.encoder.encoder_direction,
                                 axis->config.encoder.encoder_offset_rad);

    foc_clarke(axis->rt.ia_a,
               axis->rt.ib_a,
               axis->rt.ic_a,
               &axis->rt.i_alpha_a,
               &axis->rt.i_beta_a);
    foc_park(axis->rt.i_alpha_a,
             axis->rt.i_beta_a,
             axis->rt.electrical_angle_rad,
             &axis->rt.id_a,
             &axis->rt.iq_a);

    return true;
}

static void axis0_check_fast_protection(Axis0IsrContext *ctx)
{
    axis0_protection_check_fast(ctx->axis, ctx->encoder, ctx->drv0, ctx->drv1);
    if (ctx->axis->fault_flags != AXIS0_FAULT_NONE) {
        axis0_fault_enter_safe_state(ctx->axis);
    }
}

static void axis0_write_safe_neutral_duty(Axis0Context *axis)
{
    axis->rt.vd_v = 0.0f;
    axis->rt.vq_v = 0.0f;
    axis->rt.v_alpha_v = 0.0f;
    axis->rt.v_beta_v = 0.0f;
    axis->rt.duty_a = 0.5f;
    axis->rt.duty_b = 0.5f;
    axis->rt.duty_c = 0.5f;
    board_axis0_set_pwm_duty(0.5f, 0.5f, 0.5f);
}

static void axis0_closed_loop_update(Axis0IsrContext *ctx, float dt_s)
{
    Axis0Context *axis = ctx->axis;
    const Axis0ControlMode mode = axis->cmd.control_mode;
    float id_target_a = axis->cmd.id_target_a;
    float iq_target_a = 0.0f;

    if (mode == AXIS0_CONTROL_MODE_TORQUE) {
        if (axis->config.motor.torque_constant_nm_per_a <= 0.0f) {
            set_fault(axis, AXIS0_FAULT_MOTOR_CALIBRATION_FAILED);
            return;
        }
        iq_target_a = axis->cmd.input_torque_nm / axis->config.motor.torque_constant_nm_per_a;
    } else if (mode == AXIS0_CONTROL_MODE_VELOCITY ||
               mode == AXIS0_CONTROL_MODE_POSITION) {
        iq_target_a = axis->cmd.iq_target_a;
    } else {
        id_target_a = 0.0f;
        iq_target_a = 0.0f;
    }

    iq_target_a = foc_clamp(iq_target_a,
                            -axis->config.motor.current_limit_a,
                            axis->config.motor.current_limit_a);

    current_controller_update(ctx->current_controller,
                              id_target_a,
                              iq_target_a,
                              axis->rt.id_a,
                              axis->rt.iq_a,
                              axis->rt.vbus_v,
                              dt_s,
                              &axis->rt.vd_v,
                              &axis->rt.vq_v);
    foc_limit_voltage(&axis->rt.vd_v, &axis->rt.vq_v, axis->config.motor.voltage_limit_v);

    foc_inv_park(axis->rt.vd_v,
                 axis->rt.vq_v,
                 axis->rt.electrical_angle_rad,
                 &axis->rt.v_alpha_v,
                 &axis->rt.v_beta_v);

    const SvpwmDuty duty =
        svpwm_generate(axis->rt.v_alpha_v, axis->rt.v_beta_v, axis->rt.vbus_v);
    axis->rt.duty_a = duty.duty_a;
    axis->rt.duty_b = duty.duty_b;
    axis->rt.duty_c = duty.duty_c;
    board_axis0_set_pwm_duty(axis->rt.duty_a, axis->rt.duty_b, axis->rt.duty_c);
}

void axis0_current_loop_isr(Axis0IsrContext *ctx, float dt_s)
{
    Axis0Context *axis = ctx->axis;
    const Axis0StateId state = axis->state;

    if (state == AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        if (!axis0_update_fast_measurements(ctx, dt_s)) {
            return;
        }
        axis0_closed_loop_update(ctx, dt_s);
        axis0_check_fast_protection(ctx);
        return;
    }

    if (state == AXIS0_STATE_MOTOR_CALIBRATION ||
        state == AXIS0_STATE_ENCODER_CALIBRATION ||
        state == AXIS0_STATE_OPEN_LOOP_VOLTAGE_TEST) {
        /*
         * 校准/开环测试状态下，1 kHz 状态机负责写开环 duty。
         * ISR 只刷新 ADC/电流/角度/id/iq 并检查快速保护，不能覆盖 duty。
         */
        if (!axis0_update_fast_measurements(ctx, dt_s)) {
            return;
        }
        axis0_check_fast_protection(ctx);
        return;
    }

    /*
     * IDLE / READY / FAULT / CURRENT_OFFSET / ENCODER_TEST / ADC_OFFSET_TEST 等
     * 不应输出有效电压矢量。功率级通常已由状态机关闭；这里保持中性 duty，
     * 给未关闭 PWM 的测试状态一个确定安全的寄存器值。
     */
    axis0_write_safe_neutral_duty(axis);
}
