#include "app/axis0_current_loop_isr.h"

#include "board/board_odrive_v36.h"
#include "foc/foc_math.h"
#include "foc/svpwm.h"
#include "protection/fault.h"
#include "protection/protection.h"

/*
 * axis0_current_loop_isr.c
 *
 * 20kHz 高速 FOC 电流环。
 * 禁止：printf、malloc、delay、阻塞式 SPI/I2C/CAN/USB。
 *
 * 典型调用位置：
 * - PWM 定时器更新中断之后，或者 ADC 注入转换完成中断中。
 * - 必须保证进入本函数时三相电流 ADC 已经是“当前 PWM 周期”的有效样本。
 *
 * 数据所有权约定：
 * - ISR 写 axis->rt：电流、角度、电压指令、duty、vbus 等实时观测量。
 * - 1kHz 外环写 axis->cmd.iq_target_a / id_target_a。
 * - 后台通信写 axis->cmd.input_*、control_mode、requested_state 和配置。
 * - 后续工程化时建议把 axis->cmd 做成双缓冲，避免后台写到一半被 ISR 读取。
 */

void axis0_current_loop_isr(Axis0IsrContext *ctx, float dt_s)
{
    Axis0Context *axis = ctx->axis;
    uint16_t raw_a = 0u;
    uint16_t raw_b = 0u;
    uint16_t raw_c = 0u;

    /*
     * 本地快照：
     * 1. 减少 ISR 中多次读取共享变量。
     * 2. 保证本周期控制计算使用同一组命令。
     * 3. 在真实固件中，读取快照前后可用极短临界区或序号锁保护。
     */
    const Axis0StateId state = axis->state;
    const Axis0ControlMode mode = axis->cmd.control_mode;
    const float torque_target_nm = axis->cmd.input_torque_nm;
    const float iq_from_outer_a = axis->cmd.iq_target_a;
    const float id_from_outer_a = axis->cmd.id_target_a;

    if (state != AXIS0_STATE_CLOSED_LOOP_CONTROL) {
        /*
         * 非闭环状态不输出有效电压矢量。
         * 这里写 50% duty 是“中点占空比”的软安全输出；
         * 真正关 PWM 和 EN_GATE 由状态机/故障路径负责。
         */
        board_axis0_set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    /*
     * Step 1：读取 Axis0 三相电流 ADC 原始值。
     * ODrive v3.x 常见两路相电流采样，第三相可由 ia+ib+ic=0 推算。
     * 采样时刻应落在 PWM 中点附近，避开 MOSFET 换相和死区造成的尖峰；
     * 真实 STM32 实现通常由 TIM1 触发 ADC injected conversion，再在转换完成中断进本函数。
     * 如果 ADC DMA/注入通道未完成，必须立即置故障，不能继续用旧样本。
     */
    if (!board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c)) {
        set_fault(axis, AXIS0_FAULT_CURRENT_SENSOR_INVALID);
    }

    /*
     * Step 2：ADC count -> A。
     * current_sensor 内部使用零偏和 A/count 比例。
     * 零偏来自“PWM 关闭且相电流约为 0A”时的多次平均；比例来自采样电阻、
     * 运放增益和 ADC 参考电压。第一次上电必须先做 current offset calibration，
     * 否则静态电流会带偏置，PI 积分项可能在电机未动时就积累错误电压。
     */
    CurrentSensorSample current = current_sensor_convert_raw(ctx->current_sensor, raw_a, raw_b, raw_c);
    axis->rt.ia_a = current.ia_a;
    axis->rt.ib_a = current.ib_a;
    axis->rt.ic_a = current.ic_a;

    /*
     * Step 3：读取母线电压。
     * vbus 同时用于：
     * - 电压矢量限幅；
     * - 欠压/过压保护；
     * - SVPWM duty 归一化。
     */
    axis->rt.vbus_v = board_read_vbus_v();

    /*
     * Step 4：更新 MT6701 ABZ 编码器状态。
     * 当前 ABZ 是增量反馈，因此 mechanical_angle_rad 是相对角；
     * 如果不使用 Z/index，上电后的机械零点没有绝对意义。
     * FOC 真正需要的是 electrical_angle_rad，因此必须先完成 encoder calibration，
     * 得到方向 direction 和电角度零位 encoder_offset_rad。
     */
    encoder_mt6701_abz_update(ctx->encoder, dt_s);
    axis->rt.mechanical_angle_rad = ctx->encoder->mechanical_angle_rad;
    axis->rt.velocity_rad_s = ctx->encoder->velocity_rad_s;
    axis->rt.electrical_angle_rad = foc_electrical_angle_dir(axis->rt.mechanical_angle_rad,
                                                             axis->config.motor.pole_pairs,
                                                             axis->config.encoder.encoder_direction,
                                                             axis->config.encoder.encoder_offset_rad);

    /*
     * Step 5：Clarke 变换。
     * 三相静止坐标系 ia/ib/ic -> 两相静止坐标系 alpha/beta。
     * alpha/beta 可以理解为定子平面内的正交坐标。
     */
    foc_clarke(axis->rt.ia_a, axis->rt.ib_a, axis->rt.ic_a,
               &axis->rt.i_alpha_a, &axis->rt.i_beta_a);

    /*
     * Step 6：Park 变换。
     * alpha/beta -> 转子同步旋转坐标 d/q。
     * d 轴对齐转子磁链，q 轴负责产生电磁转矩。
     */
    foc_park(axis->rt.i_alpha_a, axis->rt.i_beta_a, axis->rt.electrical_angle_rad,
             &axis->rt.id_a, &axis->rt.iq_a);

    /*
     * Step 7：生成 id/iq 目标。
     * - TORQUE：直接 torque / Kt。
     * - VELOCITY/POSITION：由 1kHz 外环预先算好 iq_target。
     * - IDLE：目标为 0。
     * 第一版 id_target 固定为 0，后续弱磁/MTPA 会改变 id_target。
     * 对表贴小电机而言，调试早期保持 id=0 最容易观察问题：
     * 如果电角度 offset 错误，iq 会投影到 d 轴，表现为噪声、抖动或过流。
     */
    float id_target_a = 0.0f;
    float iq_target_a = 0.0f;
    if (mode == AXIS0_CONTROL_MODE_TORQUE) {
        if (axis->config.motor.torque_constant_nm_per_a <= 0.0f) {
            set_fault(axis, AXIS0_FAULT_MOTOR_CALIBRATION_FAILED);
        } else {
            iq_target_a = torque_target_nm / axis->config.motor.torque_constant_nm_per_a;
        }
    } else if (mode == AXIS0_CONTROL_MODE_VELOCITY ||
               mode == AXIS0_CONTROL_MODE_POSITION) {
        iq_target_a = iq_from_outer_a;
        id_target_a = id_from_outer_a;
    } else {
        id_target_a = 0.0f;
        iq_target_a = 0.0f;
    }
    iq_target_a = foc_clamp(iq_target_a,
                            -axis->config.motor.current_limit_a,
                            axis->config.motor.current_limit_a);

    /*
     * Step 8：d/q 电流 PI。
     * PI 输出是 vd/vq 电压指令，单位 V。
     * current_controller 内部已有抗积分饱和，这里再按用户 voltage_limit 做二次限幅。
     */
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

    /*
     * Step 9：反 Park。
     * 将转子坐标系的 vd/vq 旋回定子 alpha/beta 坐标，供 SVPWM 合成。
     */
    foc_inv_park(axis->rt.vd_v, axis->rt.vq_v, axis->rt.electrical_angle_rad,
                 &axis->rt.v_alpha_v, &axis->rt.v_beta_v);

    /*
     * Step 10：SVPWM。
     * 根据 alpha/beta 电压和母线电压生成三相 duty。
     * duty=0.5 表示桥臂平均电压在母线中点；三相 duty 的相对差值才形成电机端电压。
     * 输出 duty 最终限制在 0..1，真实 PWM 后端还要考虑死区、互补输出极性、
     * 最小导通时间以及低边/高边采样窗口。
     */
    SvpwmDuty duty = svpwm_generate(axis->rt.v_alpha_v, axis->rt.v_beta_v, axis->rt.vbus_v);
    axis->rt.duty_a = duty.duty_a;
    axis->rt.duty_b = duty.duty_b;
    axis->rt.duty_c = duty.duty_c;

    /*
     * Step 11：写 PWM duty。
     * 这里不能直接写 TIM 寄存器，而是通过 board/HAL 抽象。
     */
    board_axis0_set_pwm_duty(axis->rt.duty_a, axis->rt.duty_b, axis->rt.duty_c);

    /*
     * Step 12：快速保护。
     * 包括过流、母线异常、DRV nFAULT、ADC 无效、编码器无效。
     * 任意故障都必须在本 ISR 周期内关 PWM 和 EN_GATE。
     */
    axis0_protection_check_fast(axis, ctx->encoder, ctx->drv0, ctx->drv1);
    if (axis->fault_flags != AXIS0_FAULT_NONE) {
        axis0_fault_enter_safe_state(axis);
    }
}
