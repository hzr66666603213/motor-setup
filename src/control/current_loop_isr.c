#include "control/current_loop_isr.h"

#include "core/current_sensor.h"
#include "core/encoder.h"
#include "foc/foc_math.h"
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "protection/fault.h"
#include "protection/protection.h"

/*
 * current_loop_isr.c
 *
 * 20 kHz FOC 电流环骨架。
 * 注意：
 * - 所有调用链必须非阻塞。
 * - 不允许 printf、malloc、等待 SPI/CAN/UART。
 * - 真实 STM32 工程中 ADC 原始值通常来自 DMA 或注入通道结果寄存器。
 * - 与 1 kHz/后台共享的指令应使用 volatile 对象、临界区或双缓冲。
 */

void pwm_current_loop_isr(Axis *axis, CurrentController *current_controller, float dt_s)
{
    HalAdcPhaseRaw raw_current;

    /*
     * In the final firmware, fields written by 1 kHz/background tasks and read
     * here should be protected by volatile shared objects, IRQ masking, or a
     * double-buffered command handoff. The ISR works from a local snapshot.
     */
    AxisStateId state_snapshot = axis->axis.current_state;
    ControlMode mode_snapshot = axis->axis.control_mode;
    ControlInput input_snapshot = axis->input;

    if (state_snapshot != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        /* 非闭环时保持中点 duty；状态机/故障模块负责真正关闭 PWM 和 EN_GATE。 */
        hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    /* 1. 读取 ADC 转换完成后的三相电流原始值。 */
    if (!hal_adc_get_phase_current_raw(&raw_current) || !hal_adc_samples_valid()) {
        fault_set(axis, FAULT_ADC_ERROR);
    }

    /* 2. 原始 ADC count 转换为三相电流 A。 */
    current_sensor_raw_to_phase(&axis->foc_config, &raw_current,
                                &axis->foc_state.ia_a,
                                &axis->foc_state.ib_a,
                                &axis->foc_state.ic_a);

    /* 3. 读取母线电压；用于电压限幅和欠压/过压保护。 */
    axis->motor_state.bus_voltage_v = current_sensor_vbus_from_raw(hal_adc_get_vbus_raw());

    /* 4. 快速采样编码器，并更新机械角、电角度和速度。 */
    if (!encoder_sample_fast(&axis->encoder, &axis->motor, dt_s)) {
        fault_set(axis, FAULT_ENCODER_ERROR);
    }

    axis->motor_state.mechanical_angle_rad = axis->encoder.mechanical_angle_rad;
    axis->motor_state.mechanical_velocity_rad_s = axis->encoder.velocity_rad_s;
    axis->motor_state.electrical_angle_rad = axis->encoder.electrical_angle_rad;

    /* 5. Clarke 变换：三相电流 -> alpha/beta。 */
    foc_clarke(axis->foc_state.ia_a,
               axis->foc_state.ib_a,
               axis->foc_state.ic_a,
               &axis->foc_state.i_alpha_a,
               &axis->foc_state.i_beta_a);

    /* 6. Park 变换：alpha/beta -> d/q。 */
    foc_park(axis->foc_state.i_alpha_a,
             axis->foc_state.i_beta_a,
             axis->motor_state.electrical_angle_rad,
             &axis->foc_state.id_a,
             &axis->foc_state.iq_a);

    /* 7. 根据控制模式得到 id/iq 目标；第一版 id 固定为 0 A。 */
    float id_target_a = 0.0f;
    float iq_target_a = 0.0f;
    const float kt = axis->motor.torque_constant_nm_per_a;
    if (kt <= 0.0f) {
        fault_set(axis, FAULT_CALIBRATION_FAILED);
    } else if (mode_snapshot == CONTROL_MODE_TORQUE) {
        iq_target_a = input_snapshot.torque_nm / kt;
    } else {
        /* Velocity/position loops write torque_nm or iq target through a double-buffered command in production. */
        iq_target_a = input_snapshot.torque_nm / kt;
    }
    iq_target_a = foc_clamp(iq_target_a, -axis->motor.current_limit_a, axis->motor.current_limit_a);

    /* 8. 电流 PI 输出 vd/vq，并在控制器内部做电压矢量限幅。 */
    current_controller_update(current_controller,
                              id_target_a,
                              iq_target_a,
                              axis->foc_state.id_a,
                              axis->foc_state.iq_a,
                              axis->motor_state.bus_voltage_v,
                              dt_s,
                              &axis->foc_state.vd_v,
                              &axis->foc_state.vq_v);

    /* 9. 反 Park：d/q 电压 -> alpha/beta 电压。 */
    foc_inv_park(axis->foc_state.vd_v,
                 axis->foc_state.vq_v,
                 axis->motor_state.electrical_angle_rad,
                 &axis->foc_state.v_alpha_v,
                 &axis->foc_state.v_beta_v);

    /* 10. SVPWM：alpha/beta 电压和 vbus -> 三相 duty。 */
    foc_svpwm(axis->foc_state.v_alpha_v,
              axis->foc_state.v_beta_v,
              axis->motor_state.bus_voltage_v,
              &axis->foc_state.duty_u,
              &axis->foc_state.duty_v,
              &axis->foc_state.duty_w);

    /* 11. 写入 PWM 比较值；具体寄存器操作由 HAL 后端完成。 */
    hal_pwm_set_duty(axis->foc_state.duty_u, axis->foc_state.duty_v, axis->foc_state.duty_w);

    /* 12. 快速保护检查；一旦有故障，立即进入安全输出。 */
    protection_check_fast(axis);
    if (fault_has_any(axis)) {
        fault_enter_safe_state(axis);
    }
}
