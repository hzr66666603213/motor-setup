#include "app/axis_state_machine.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "protection/fault.h"
#include "protection/protection.h"

/*
 * axis_state_machine.c
 *
 * Axis0 低频状态机。
 *
 * 约定：
 * - request_pending=true 表示 requested_state 还没被消费；
 * - 成功进入被请求状态后立即清 request_pending；
 * - 无效请求也会被消费，避免校准结束回 IDLE 后自动重复进入同一状态；
 * - 校准/open-loop 的功率级使能只在 enter 阶段做一次，update 里只写 duty。
 */

void axis_request_state(Axis0Context *axis, Axis0StateId requested_state)
{
    axis->requested_state = requested_state;
    axis->request_pending = true;
}

bool axis_is_ready_for_closed_loop(const Axis0Context *axis)
{
    return axis->current_offset_valid &&
           axis->motor_calibrated &&
           axis->encoder_calibrated &&
           axis->fault_flags == AXIS0_FAULT_NONE &&
           axis->state == AXIS0_STATE_READY;
}

static void axis_consume_request_if_matches(Axis0Context *axis, Axis0StateId entered_state)
{
    if (axis->request_pending && axis->requested_state == entered_state) {
        axis->request_pending = false;
    }
}

static void axis_consume_invalid_request(Axis0Context *axis)
{
    axis->request_pending = false;
}

static void axis_enter_state(Axis0Context *axis, Axis0StateMachineContext *sm, Axis0StateId next)
{
    axis->state = next;
    axis_consume_request_if_matches(axis, next);

    if (next == AXIS0_STATE_IDLE) {
        board_disable_axis0_power_stage(axis);
    } else if (next == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION) {
        if (!board_start_adc_sampling_without_power_stage(axis)) {
            set_fault(axis, AXIS0_FAULT_CURRENT_SENSOR_INVALID);
            return;
        }
        axis0_calibration_start(&sm->calibration, CALIB_CURRENT_OFFSET);
    } else if (next == AXIS0_STATE_MOTOR_CALIBRATION) {
        if (!board_enable_axis0_power_stage_for_calibration(axis)) {
            set_fault(axis, AXIS0_FAULT_PWM_NOT_ENABLED);
            return;
        }
        axis0_calibration_start(&sm->calibration, CALIB_RESISTANCE);
    } else if (next == AXIS0_STATE_ENCODER_CALIBRATION) {
        if (!board_enable_axis0_power_stage_for_calibration(axis)) {
            set_fault(axis, AXIS0_FAULT_PWM_NOT_ENABLED);
            return;
        }
        axis0_calibration_start(&sm->calibration, CALIB_ENCODER_DIRECTION);
    } else if (next == AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        if (!board_enable_axis0_power_stage(axis)) {
            set_fault(axis, AXIS0_FAULT_PWM_NOT_ENABLED);
        }
    } else if (next == AXIS0_STATE_PWM_TEST) {
        hal_gpio_set_gate_enable(false);
        hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
        hal_pwm_enable();
    } else if (next == AXIS0_STATE_ENCODER_TEST ||
               next == AXIS0_STATE_ADC_OFFSET_TEST) {
        board_disable_axis0_power_stage(axis);
    } else if (next == AXIS0_STATE_OPEN_LOOP_VOLTAGE_TEST) {
        if (!board_enable_axis0_power_stage_for_calibration(axis)) {
            set_fault(axis, AXIS0_FAULT_PWM_NOT_ENABLED);
        }
    } else if (next == AXIS0_STATE_FAULT) {
        axis0_fault_enter_safe_state(axis);
    }
}

static void axis_update_idle(Axis0Context *axis, Axis0StateMachineContext *sm)
{
    if (!axis->request_pending) {
        return;
    }

    const Axis0StateId requested = axis->requested_state;
    if (requested == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION) {
        axis_enter_state(axis, sm, AXIS0_STATE_CURRENT_OFFSET_CALIBRATION);
    } else if (requested == AXIS0_STATE_MOTOR_CALIBRATION && axis->current_offset_valid) {
        axis_enter_state(axis, sm, AXIS0_STATE_MOTOR_CALIBRATION);
    } else if (requested == AXIS0_STATE_ENCODER_CALIBRATION && axis->motor_calibrated) {
        axis_enter_state(axis, sm, AXIS0_STATE_ENCODER_CALIBRATION);
    } else if (requested == AXIS0_STATE_PWM_TEST ||
               requested == AXIS0_STATE_ENCODER_TEST ||
               requested == AXIS0_STATE_ADC_OFFSET_TEST ||
               requested == AXIS0_STATE_OPEN_LOOP_VOLTAGE_TEST) {
        axis_enter_state(axis, sm, requested);
    } else {
        axis_consume_invalid_request(axis);
    }
}

static void axis_update_calibration(Axis0Context *axis, Axis0StateMachineContext *sm, float dt_s)
{
    axis0_calibration_update(axis, &sm->calibration, sm->current_sensor, sm->encoder, dt_s);
    if (sm->calibration.step == CALIB_DONE) {
        if (axis->current_offset_valid &&
            axis->motor_calibrated &&
            axis->encoder_calibrated) {
            axis_enter_state(axis, sm, AXIS0_STATE_READY);
        } else {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        }
    } else if (sm->calibration.step == CALIB_FAILED) {
        set_fault(axis, axis->state == AXIS0_STATE_ENCODER_CALIBRATION ?
                        AXIS0_FAULT_ENCODER_CALIBRATION_FAILED :
                        AXIS0_FAULT_MOTOR_CALIBRATION_FAILED);
    }
}

static void axis_update_open_loop_test(Axis0Context *axis)
{
    float duty_a = 0.5f;
    float duty_b = 0.5f;
    float duty_c = 0.5f;
    float test_voltage_v = axis->config.motor.voltage_limit_v;

    if (test_voltage_v > 0.5f) {
        test_voltage_v = 0.5f;
    }

    foc_svpwm(test_voltage_v, 0.0f, axis->rt.vbus_v, &duty_a, &duty_b, &duty_c);
    board_axis0_set_pwm_duty(duty_a, duty_b, duty_c);
    axis->rt.v_alpha_v = test_voltage_v;
    axis->rt.v_beta_v = 0.0f;
    axis->rt.duty_a = duty_a;
    axis->rt.duty_b = duty_b;
    axis->rt.duty_c = duty_c;
}

static void axis_update_closed_loop_outer(Axis0Context *axis, Axis0StateMachineContext *sm, float dt_s)
{
    if (axis->cmd.control_mode == AXIS0_CONTROL_MODE_VELOCITY) {
        axis->cmd.iq_target_a = velocity_controller_update(sm->velocity_controller,
                                                           axis->cmd.input_velocity_rad_s,
                                                           axis->rt.velocity_rad_s,
                                                           dt_s);
    } else if (axis->cmd.control_mode == AXIS0_CONTROL_MODE_POSITION) {
        const float vel_target = position_controller_update(sm->position_controller,
                                                            axis->cmd.input_position_rad,
                                                            axis->rt.mechanical_angle_rad);
        axis->cmd.iq_target_a = velocity_controller_update(sm->velocity_controller,
                                                           vel_target,
                                                           axis->rt.velocity_rad_s,
                                                           dt_s);
    }
}

void axis_update_1khz(Axis0Context *axis, Axis0StateMachineContext *sm, float dt_s)
{
    if (axis->fault_flags != AXIS0_FAULT_NONE && axis->state != AXIS0_STATE_FAULT) {
        axis_enter_state(axis, sm, AXIS0_STATE_FAULT);
    }

    if (axis->state == AXIS0_STATE_BOOT) {
        if (board_init_power_safe(axis)) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        }
    } else if (axis->state == AXIS0_STATE_IDLE) {
        axis_update_idle(axis, sm);
    } else if (axis->state == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION ||
               axis->state == AXIS0_STATE_MOTOR_CALIBRATION ||
               axis->state == AXIS0_STATE_ENCODER_CALIBRATION) {
        if (axis->request_pending && axis->requested_state == AXIS0_STATE_IDLE) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        } else {
            if (axis->request_pending) {
                axis_consume_invalid_request(axis);
            }
            axis_update_calibration(axis, sm, dt_s);
        }
    } else if (axis->state == AXIS0_STATE_READY) {
        if (axis->request_pending &&
            axis->requested_state == AXIS0_STATE_CLOSED_LOOP_CONTROL &&
            axis_is_ready_for_closed_loop(axis)) {
            axis_enter_state(axis, sm, AXIS0_STATE_CLOSED_LOOP_CONTROL);
        } else if (axis->request_pending) {
            axis_consume_invalid_request(axis);
        }
    } else if (axis->state == AXIS0_STATE_PWM_TEST ||
               axis->state == AXIS0_STATE_ENCODER_TEST ||
               axis->state == AXIS0_STATE_ADC_OFFSET_TEST) {
        if (axis->request_pending && axis->requested_state == AXIS0_STATE_IDLE) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        } else if (axis->request_pending) {
            axis_consume_invalid_request(axis);
        }
    } else if (axis->state == AXIS0_STATE_OPEN_LOOP_VOLTAGE_TEST) {
        if (axis->request_pending && axis->requested_state == AXIS0_STATE_IDLE) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        } else {
            if (axis->request_pending) {
                axis_consume_invalid_request(axis);
            }
            axis_update_open_loop_test(axis);
        }
    } else if (axis->state == AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        if (axis->request_pending && axis->requested_state == AXIS0_STATE_IDLE) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        } else {
            if (axis->request_pending) {
                axis_consume_invalid_request(axis);
            }
            axis_update_closed_loop_outer(axis, sm, dt_s);
        }
    } else if (axis->state == AXIS0_STATE_FAULT) {
        board_disable_axis0_power_stage(axis);
    }
}

void axis_update_background(Axis0Context *axis, Axis0StateMachineContext *sm)
{
    axis0_protection_check_slow(axis, sm->encoder, sm->drv0, sm->drv1);
    if (axis->fault_flags != AXIS0_FAULT_NONE) {
        axis0_fault_enter_safe_state(axis);
    }
}
