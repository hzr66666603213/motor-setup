#include "config/axis0_default_config.h"

/*
 * axis0_default_config.c
 *
 * 低风险默认配置生成。
 * 这些值不是“最佳性能参数”，而是为了第一次上电更安全：
 * - 小电流；
 * - 小电压；
 * - 低速度；
 * - 不自动闭环；
 * - 电机 R/L 初始为 0，强制用户走校准流程。
 */

Axis0Config axis0_default_config_make(void)
{
    Axis0Config cfg;

    /*
     * 电机参数：
     * 2804 常见极对数可能是 7，但不同厂家可能不同。
     * 如果 pole_pairs 错误，电角度会错，闭环会抖动甚至过流。
     */
    cfg.motor.pole_pairs = AXIS0_DEFAULT_POLE_PAIRS;
    cfg.motor.phase_resistance_ohm = 0.0f;
    cfg.motor.phase_inductance_h = 0.0f;
    /*
     * torque_constant 先给一个很小电机常见量级的占位值。
     * 真正力矩控制前必须通过电机资料或实验标定修正。
     */
    cfg.motor.torque_constant_nm_per_a = 0.02f;
    cfg.motor.current_limit_a = AXIS0_DEFAULT_CURRENT_LIMIT_A;
    cfg.motor.calibration_current_a = AXIS0_DEFAULT_CALIB_CURRENT_A;
    cfg.motor.voltage_limit_v = AXIS0_DEFAULT_VOLTAGE_LIMIT_V;
    cfg.motor.velocity_limit_rad_s = AXIS0_DEFAULT_VELOCITY_LIMIT_RAD_S;
    cfg.motor.position_limit_enabled = false;
    cfg.motor.position_min_rad = -0.0f;
    cfg.motor.position_max_rad = 0.0f;

    /*
     * MT6701 ABZ：
     * 默认 PPR=1024，四倍频 CPR=4096。
     * 如果你的 MT6701 模块配置不同，需要通过 set encoder_cpr 或修改默认值。
     */
    cfg.encoder.encoder_ppr = AXIS0_DEFAULT_MT6701_PPR;
    cfg.encoder.encoder_cpr = AXIS0_DEFAULT_MT6701_CPR;
    cfg.encoder.encoder_direction = 1;
    cfg.encoder.encoder_offset_rad = 0.0f;
    cfg.encoder.use_index_z = false;

    /*
     * 控制频率：
     * ODrive v3.6 硬件足以跑 20kHz 电流环。
     * 第一版速度/位置环保守运行，方便观察和调参。
     */
    cfg.control.pwm_frequency_hz = AXIS0_DEFAULT_PWM_HZ;
    cfg.control.current_loop_hz = AXIS0_DEFAULT_CURRENT_LOOP_HZ;
    cfg.control.velocity_loop_hz = AXIS0_DEFAULT_VELOCITY_LOOP_HZ;
    cfg.control.position_loop_hz = AXIS0_DEFAULT_POSITION_LOOP_HZ;
    cfg.control.current_kp = AXIS0_DEFAULT_CURRENT_KP;
    cfg.control.current_ki = AXIS0_DEFAULT_CURRENT_KI;
    cfg.control.velocity_kp = AXIS0_DEFAULT_VELOCITY_KP;
    cfg.control.velocity_ki = AXIS0_DEFAULT_VELOCITY_KI;
    cfg.control.position_kp = AXIS0_DEFAULT_POSITION_KP;

    /*
     * 保护阈值：
     * 默认按 12V/24V 低压测试，不面向 48V 高功率运行。
     * 过流阈值 2A，略高于默认 current_limit=1A。
     */
    cfg.protection.vbus_min_v = AXIS0_DEFAULT_VBUS_MIN_V;
    cfg.protection.vbus_max_v = AXIS0_DEFAULT_VBUS_MAX_V;
    cfg.protection.phase_overcurrent_a = AXIS0_DEFAULT_OVERCURRENT_A;
    cfg.protection.overtemperature_c = AXIS0_DEFAULT_OVERTEMP_C;
    cfg.protection.encoder_error_enable = true;
    cfg.protection.drv_fault_enable = true;

    /* 上电默认 IDLE，不允许自动进入闭环。 */
    cfg.startup_state = AXIS0_STATE_IDLE;
    return cfg;
}
