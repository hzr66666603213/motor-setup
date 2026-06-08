#include "app/axis_state_machine.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"
#include "protection/fault.h"
#include "protection/protection.h"

/*
 * axis_state_machine.c
 *
 * Axis0 状态机实现。
 * 任意严重故障都进入 FAULT；只有 clear_faults 后才能回 IDLE。
 *
 * 状态机设计目标：
 * - 上电默认不使能 DRV8301，不开 PWM。
 * - 所有高风险状态都必须由用户命令显式请求。
 * - 没有完成电流零偏、电机校准、编码器校准，禁止进入闭环。
 * - FAULT 是吸收态，必须 clear_faults 才能离开。
 */

void axis_request_state(Axis0Context *axis, Axis0StateId requested_state)
{
    /*
     * 这里只记录请求，不立即切换。
     * 真正切换在 axis_update_1khz() 中集中检查安全条件。
     */
    axis->requested_state = requested_state;
}

bool axis_is_ready_for_closed_loop(const Axis0Context *axis)
{
    /*
     * 闭环准入条件：
     * 1. 电流零偏有效；
     * 2. 电机 R/L 或等效参数已校准；
     * 3. 编码器方向和 offset 已校准；
     * 4. 当前无故障；
     * 5. 当前处于 READY。
     */
    return axis->current_offset_valid &&
           axis->motor_calibrated &&
           axis->encoder_calibrated &&
           axis->fault_flags == AXIS0_FAULT_NONE &&
           axis->state == AXIS0_STATE_READY;
}

static void axis_enter_state(Axis0Context *axis, Axis0StateMachineContext *sm, Axis0StateId next)
{
    /*
     * 所有状态进入动作集中在这里，便于审查“进入某状态时功率级是什么状态”。
     */
    axis->state = next;
    if (next == AXIS0_STATE_IDLE) {
        /* IDLE 必须安全：PWM off，DRV off。 */
        board_disable_axis0_power_stage(axis);
    } else if (next == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION) {
        /* 电流零偏校准从关闭功率级开始。 */
        axis0_calibration_start(&sm->calibration, CALIB_CURRENT_OFFSET);
    } else if (next == AXIS0_STATE_MOTOR_CALIBRATION) {
        /* 电机校准要求电流零偏已经完成。准入检查在 update 中做。 */
        axis0_calibration_start(&sm->calibration, CALIB_RESISTANCE);
    } else if (next == AXIS0_STATE_ENCODER_CALIBRATION) {
        /* 编码器校准要求电机可以低风险开环转动。 */
        axis0_calibration_start(&sm->calibration, CALIB_ENCODER_DIRECTION);
    } else if (next == AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        /*
         * 进入闭环时才真正使能功率级。
         * board_enable_axis0_power_stage() 会检查 vbus、fault、ADC、encoder 等条件。
         */
        if (!board_enable_axis0_power_stage(axis)) {
            set_fault(axis, AXIS0_FAULT_PWM_NOT_ENABLED);
        }
    } else if (next == AXIS0_STATE_FAULT) {
        axis0_fault_enter_safe_state(axis);
    }
}

void axis_update_1khz(Axis0Context *axis, Axis0StateMachineContext *sm, float dt_s)
{
    /*
     * 1kHz 主更新：
     * - 推进状态机；
     * - 推进非阻塞校准；
     * - 计算速度/位置外环；
     * - 不执行 20kHz 电流环。
     */
    if (axis->fault_flags != AXIS0_FAULT_NONE && axis->state != AXIS0_STATE_FAULT) {
        axis_enter_state(axis, sm, AXIS0_STATE_FAULT);
    }

    if (axis->state == AXIS0_STATE_BOOT) {
        /* BOOT：板级安全初始化，确认基础电源/故障状态。 */
        if (board_init_power_safe(axis)) {
            axis_enter_state(axis, sm, AXIS0_STATE_IDLE);
        }
    } else if (axis->state == AXIS0_STATE_IDLE) {
        /*
         * IDLE：等待用户命令。
         * 注意 motor_calibration 必须在 current_offset_valid 后才允许。
         */
        if (axis->requested_state == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION) {
            axis_enter_state(axis, sm, AXIS0_STATE_CURRENT_OFFSET_CALIBRATION);
        } else if (axis->requested_state == AXIS0_STATE_MOTOR_CALIBRATION && axis->current_offset_valid) {
            axis_enter_state(axis, sm, AXIS0_STATE_MOTOR_CALIBRATION);
        } else if (axis->requested_state == AXIS0_STATE_ENCODER_CALIBRATION && axis->motor_calibrated) {
            axis_enter_state(axis, sm, AXIS0_STATE_ENCODER_CALIBRATION);
        }
    } else if (axis->state == AXIS0_STATE_CURRENT_OFFSET_CALIBRATION ||
               axis->state == AXIS0_STATE_MOTOR_CALIBRATION ||
               axis->state == AXIS0_STATE_ENCODER_CALIBRATION) {
        /* 校准状态：每个 1kHz tick 推进一步，不阻塞。 */
        axis0_calibration_update(axis, &sm->calibration, sm->current_sensor, sm->encoder, dt_s);
        if (sm->calibration.step == CALIB_DONE) {
            /*
             * 单项校准完成后先检查总体准入：
             * 只有电流零偏、电机参数和编码器校准都有效，才允许进入 READY。
             */
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
    } else if (axis->state == AXIS0_STATE_READY) {
        /* READY：校准已完成，允许用户显式请求 closed_loop。 */
        if (axis->requested_state == AXIS0_STATE_CLOSED_LOOP_CONTROL &&
            axis_is_ready_for_closed_loop(axis)) {
            axis_enter_state(axis, sm, AXIS0_STATE_CLOSED_LOOP_CONTROL);
        }
    } else if (axis->state == AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        /*
         * CLOSED_LOOP_CONTROL：
         * 20kHz ISR 负责电流环。
         * 这里仅做 1kHz 外环，把输出写到 axis->cmd.iq_target_a。
         */
        if (axis->cmd.control_mode == AXIS0_CONTROL_MODE_VELOCITY) {
            axis->cmd.iq_target_a = velocity_controller_update(sm->velocity_controller,
                                                               axis->cmd.input_velocity_rad_s,
                                                               axis->rt.velocity_rad_s,
                                                               dt_s);
        } else if (axis->cmd.control_mode == AXIS0_CONTROL_MODE_POSITION) {
            /*
             * 位置模式采用串级结构：
             * position P -> velocity target -> velocity PI -> iq target。
             */
            const float vel_target = position_controller_update(sm->position_controller,
                                                                axis->cmd.input_position_rad,
                                                                axis->rt.mechanical_angle_rad);
            axis->cmd.iq_target_a = velocity_controller_update(sm->velocity_controller,
                                                               vel_target,
                                                               axis->rt.velocity_rad_s,
                                                               dt_s);
        }
    } else if (axis->state == AXIS0_STATE_FAULT) {
        board_disable_axis0_power_stage(axis);
    }
}

void axis_update_background(Axis0Context *axis, Axis0StateMachineContext *sm)
{
    /*
     * 后台任务：
     * - 慢速保护检查；
     * - 后续可加入 DRV8301 SPI 状态读取、USB/CAN 通信、Flash 保存等。
     */
    axis0_protection_check_slow(axis, sm->encoder, sm->drv0, sm->drv1);
    if (axis->fault_flags != AXIS0_FAULT_NONE) {
        axis0_fault_enter_safe_state(axis);
    }
}
