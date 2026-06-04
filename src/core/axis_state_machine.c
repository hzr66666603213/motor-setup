#include "core/axis_state_machine.h"

#include "core/encoder.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "protection/fault.h"

/*
 * axis_state_machine.c
 *
 * 单轴状态机实现。
 * 设计要点：
 * - IDLE/FAULT 中必须保证 PWM 和 EN_GATE 处于安全状态。
 * - 校准流程通过 motor_calibration_update 非阻塞推进。
 * - 进入闭环前检查母线电压、编码器、栅极驱动和故障位。
 */

static bool safe_to_enable(const Axis *axis)
{
    /* 闭环使能的最小安全条件；具体产品可加入急停、限位、温度降额等条件。 */
    return axis->motor_state.bus_voltage_v > 10.0f &&
           axis->motor_state.bus_voltage_v < 56.0f &&
           encoder_is_ready(&axis->encoder) &&
           !axis->encoder.has_error &&
           !hal_gpio_read_fault_pin() &&
           axis->axis.error == FAULT_NONE;
}

static void transition(Axis *axis, AxisStateMachineContext *ctx, AxisStateId next)
{
    /* 统一状态切换入口：先 exit 当前状态，再 enter 新状态。 */
    switch (axis->axis.current_state) {
    case AXIS_STATE_BOOT: axis_sm_exit_boot(axis); break;
    case AXIS_STATE_IDLE: axis_sm_exit_idle(axis); break;
    case AXIS_STATE_MOTOR_CALIBRATION: axis_sm_exit_motor_calibration(axis); break;
    case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: axis_sm_exit_encoder_offset_calibration(axis); break;
    case AXIS_STATE_CLOSED_LOOP_CONTROL: axis_sm_exit_closed_loop(axis); break;
    case AXIS_STATE_FAULT: axis_sm_exit_fault(axis); break;
    default: break;
    }

    axis->axis.current_state = next;

    switch (next) {
    case AXIS_STATE_BOOT: axis_sm_enter_boot(axis); break;
    case AXIS_STATE_IDLE: axis_sm_enter_idle(axis); break;
    case AXIS_STATE_MOTOR_CALIBRATION: axis_sm_enter_motor_calibration(axis, ctx); break;
    case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: axis_sm_enter_encoder_offset_calibration(axis, ctx); break;
    case AXIS_STATE_CLOSED_LOOP_CONTROL: axis_sm_enter_closed_loop(axis); break;
    case AXIS_STATE_FAULT: axis_sm_enter_fault(axis); break;
    default: break;
    }
}

void axis_sm_init(Axis *axis, AxisStateMachineContext *ctx)
{
    /* 上电初始化默认不使能功率级。 */
    motor_calibration_set_defaults(&ctx->motor_calib_config);
    axis->axis.current_state = AXIS_STATE_BOOT;
    axis->axis.requested_state = AXIS_STATE_IDLE;
    axis->axis.error = FAULT_NONE;
    axis_sm_enter_boot(axis);
}

void axis_sm_request_state(Axis *axis, AxisStateId requested_state)
{
    axis->axis.requested_state = requested_state;
}

void axis_sm_update(Axis *axis, AxisStateMachineContext *ctx, float dt_s)
{
    /* 任意非 FAULT 状态下只要出现故障，立即转入 FAULT。 */
    if (axis->axis.error != FAULT_NONE && axis->axis.current_state != AXIS_STATE_FAULT) {
        transition(axis, ctx, AXIS_STATE_FAULT);
    }

    switch (axis->axis.current_state) {
    case AXIS_STATE_BOOT:
        axis_sm_update_boot(axis);
        transition(axis, ctx, AXIS_STATE_IDLE);
        break;
    case AXIS_STATE_IDLE:
        axis_sm_update_idle(axis);
        /* IDLE 是所有用户请求的仲裁点，真正切换前必须检查安全条件。 */
        if (axis->axis.requested_state == AXIS_STATE_MOTOR_CALIBRATION && safe_to_enable(axis)) {
            transition(axis, ctx, AXIS_STATE_MOTOR_CALIBRATION);
        } else if (axis->axis.requested_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION && safe_to_enable(axis)) {
            transition(axis, ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
        } else if (axis->axis.requested_state == AXIS_STATE_CLOSED_LOOP_CONTROL && axis->axis.calibration_valid && safe_to_enable(axis)) {
            transition(axis, ctx, AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        break;
    case AXIS_STATE_MOTOR_CALIBRATION:
        axis_sm_update_motor_calibration(axis, ctx, dt_s);
        break;
    case AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
        axis_sm_update_encoder_offset_calibration(axis, ctx, dt_s);
        break;
    case AXIS_STATE_CLOSED_LOOP_CONTROL:
        axis_sm_update_closed_loop(axis);
        break;
    case AXIS_STATE_FAULT:
        axis_sm_update_fault(axis);
        break;
    default:
        transition(axis, ctx, AXIS_STATE_FAULT);
        break;
    }
}

void axis_sm_clear_faults(Axis *axis)
{
    /* 清故障不直接进闭环，必须先回 IDLE，再由用户重新请求状态。 */
    fault_clear_all(axis);
    axis->axis.requested_state = AXIS_STATE_IDLE;
}

void axis_sm_enter_boot(Axis *axis) { (void)axis; hal_pwm_disable(); hal_gpio_set_gate_enable(false); }
void axis_sm_update_boot(Axis *axis) { (void)axis; }
void axis_sm_exit_boot(Axis *axis) { (void)axis; }

void axis_sm_enter_idle(Axis *axis) { (void)axis; hal_pwm_disable(); hal_pwm_set_all_low(); hal_gpio_set_gate_enable(false); }
void axis_sm_update_idle(Axis *axis) { (void)axis; }
void axis_sm_exit_idle(Axis *axis) { (void)axis; }

void axis_sm_enter_motor_calibration(Axis *axis, AxisStateMachineContext *ctx)
{
    (void)axis;
    /* 电机校准从电流零偏开始，然后推进到电阻、电感。 */
    motor_calibration_start(&ctx->motor_calib, CALIB_STEP_CURRENT_OFFSET);
}

void axis_sm_update_motor_calibration(Axis *axis, AxisStateMachineContext *ctx, float dt_s)
{
    CalibrationResult result = motor_calibration_update(axis, &ctx->motor_calib, &ctx->motor_calib_config, dt_s);
    if (result == CALIB_OK) {
        /* 校准完成后回 IDLE，由用户再请求闭环。 */
        axis->axis.calibration_valid = true;
        transition(axis, ctx, AXIS_STATE_IDLE);
    } else if (result != CALIB_RUNNING) {
        fault_set(axis, FAULT_CALIBRATION_FAILED);
    }
}

void axis_sm_exit_motor_calibration(Axis *axis) { (void)axis; hal_pwm_disable(); }

void axis_sm_enter_encoder_offset_calibration(Axis *axis, AxisStateMachineContext *ctx)
{
    (void)axis;
    /* 编码器校准从固定电角度锁定开始。 */
    motor_calibration_start(&ctx->motor_calib, CALIB_STEP_ENCODER_OFFSET);
}

void axis_sm_update_encoder_offset_calibration(Axis *axis, AxisStateMachineContext *ctx, float dt_s)
{
    CalibrationResult result = motor_calibration_update(axis, &ctx->motor_calib, &ctx->motor_calib_config, dt_s);
    if (result == CALIB_OK) {
        axis->axis.calibration_valid = true;
        transition(axis, ctx, AXIS_STATE_IDLE);
    } else if (result != CALIB_RUNNING) {
        fault_set(axis, FAULT_CALIBRATION_FAILED);
    }
}

void axis_sm_exit_encoder_offset_calibration(Axis *axis) { (void)axis; hal_pwm_disable(); }

void axis_sm_enter_closed_loop(Axis *axis)
{
    /* 进入闭环时先允许 gate，再允许 PWM；具体时序可按驱动芯片要求细化。 */
    axis->axis.closed_loop_allowed = true;
    hal_gpio_set_gate_enable(true);
    hal_pwm_enable();
}

void axis_sm_update_closed_loop(Axis *axis)
{
    /* 闭环运行中持续检查最小安全条件。 */
    if (!safe_to_enable(axis)) {
        fault_set(axis, FAULT_GATE_DRIVER);
    }
}

void axis_sm_exit_closed_loop(Axis *axis)
{
    axis->axis.closed_loop_allowed = false;
    hal_pwm_disable();
}

void axis_sm_enter_fault(Axis *axis) { fault_enter_safe_state(axis); }
void axis_sm_update_fault(Axis *axis) { fault_enter_safe_state(axis); }
void axis_sm_exit_fault(Axis *axis) { (void)axis; }
