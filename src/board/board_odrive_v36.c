#include "board/board_odrive_v36.h"

#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"

/*
 * board_odrive_v36.c
 *
 * ODrive v3.6 板级安全抽象 stub。
 * 本文件不直接调用 STM32 HAL；移植到 CubeIDE/Makefile 工程时，可以在 HAL 层绑定 TIM/ADC/GPIO/SPI。
 *
 * TODO_PIN_CHECK：
 * - M0_TEMP、Encoder0 Z、CAN、Brake resistor 等引脚需要用户对照 ODrive v3.6 原理图确认。
 * - ADC 分压比例、采样电阻、放大器增益必须按实际硬件修正。
 *
 * 这个模块的职责不是实现 FOC，而是回答三个问题：
 * 1. 当前板子是否处于功率安全状态？
 * 2. 什么时候允许使能 Axis0 功率级？
 * 3. 故障时如何最快关闭 PWM 和 EN_GATE？
 */

static BoardOdriveV36Status s_board_status;
static bool s_debug_led_on = false;
static float s_vbus_scale_v_per_count = 60.0f / 4095.0f;
static uint32_t s_last_phase_adc_seq = 0u;

static bool board_vbus_in_safe_range(const Axis0Context *axis, float vbus_v)
{
    /*
     * 母线电压准入检查。
     * 默认按 12V/24V 学习调试设置，不鼓励第一阶段使用 48V。
     */
    return (vbus_v >= axis->config.protection.vbus_min_v) &&
           (vbus_v <= axis->config.protection.vbus_max_v);
}

bool board_init_power_safe(Axis0Context *axis)
{
    /*
     * 上电安全初始化：
     * - 先关 PWM，避免定时器输出未知状态；
     * - 三相 duty 清到安全值；
     * - 拉低 EN_GATE，确保 DRV8301 不驱动 MOS；
     * - 再读取 nFAULT、ADC 和 vbus。
     */
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);

    s_board_status.pwm_disabled = true;
    s_board_status.drv_gate_enabled = false;
    s_board_status.drv_nfault_active = hal_gpio_read_fault_pin();
    s_board_status.drv0_status_valid = false;
    s_board_status.drv1_status_valid = false;
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.encoder_valid = false;
    s_board_status.vbus_v = board_read_vbus_v();

    if (!board_vbus_in_safe_range(axis, s_board_status.vbus_v)) {
        /* VBUS 不在测试范围内时，不允许进入后续功率流程。 */
        return false;
    }
    return !s_board_status.drv_nfault_active;
}

bool board_enable_axis0_power_stage(Axis0Context *axis)
{
    /*
     * 使能功率级前重新采样关键条件，不能只依赖旧状态：
     * - fault_flags 必须为 0；
     * - 共享 DRV nFAULT 不得有效。ODrive v3.6 EN_GATE/nFAULT 是 M0/M1 共用，
     *   Axis0-only 也要确保 M1 DRV8301 已初始化、无故障、保持安全；
     * - ADC 样本有效；
     * - 编码器已校准；
     * - VBUS 在安全范围。
     */
    s_board_status.drv_nfault_active = hal_gpio_read_fault_pin();
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.encoder_valid = axis->encoder_calibrated;
    s_board_status.vbus_v = board_read_vbus_v();

    if (axis->fault_flags != 0u ||
        s_board_status.drv_nfault_active ||
        !s_board_status.adc_valid ||
        !s_board_status.encoder_valid ||
        !board_vbus_in_safe_range(axis, s_board_status.vbus_v)) {
        /* 任一条件失败都立即回到安全输出。 */
        board_disable_axis0_power_stage(axis);
        return false;
    }

    /*
     * 推荐使能顺序：
     * 1. duty 设为 50%，对应零平均相电压；
     * 2. 使能 EN_GATE；
     * 3. 使能 PWM 输出。
     * 这样可以避免 EN_GATE 拉高瞬间三相输出处于随机 duty。
     */
    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    hal_gpio_set_gate_enable(true);
    hal_pwm_enable();

    s_board_status.pwm_disabled = false;
    s_board_status.drv_gate_enabled = true;
    return true;
}

bool board_enable_axis0_power_stage_for_calibration(Axis0Context *axis)
{
    /*
     * 校准期间需要开环小电压注入，此时 encoder_calibrated 可能尚未完成。
     * 这里故意不检查 encoder_valid，但仍检查 fault、nFAULT、ADC 和 VBUS。
     * 该接口只能用于 CURRENT/MOTOR/ENCODER 校准流程，不能作为闭环准入。
     */
    s_board_status.drv_nfault_active = hal_gpio_read_fault_pin();
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.vbus_v = board_read_vbus_v();

    if (axis->fault_flags != 0u ||
        s_board_status.drv_nfault_active ||
        !s_board_status.adc_valid ||
        !board_vbus_in_safe_range(axis, s_board_status.vbus_v)) {
        board_disable_axis0_power_stage(axis);
        return false;
    }

    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    hal_gpio_set_gate_enable(true);
    hal_pwm_enable();

    s_board_status.pwm_disabled = false;
    s_board_status.drv_gate_enabled = true;
    return true;
}

bool board_start_adc_sampling_without_power_stage(Axis0Context *axis)
{
    /*
     * 电流零偏校准需要“ADC 同步采样仍在跑”，但绝不能驱动 MOS：
     * - EN_GATE 拉低，DRV8301 不驱动外部功率管；
     * - TIM1 继续运行，用于产生 ADC injected trigger；
     * - TIM1 MOE 保持 0，PWM 输出级关闭。
     *
     * 这样 board_axis0_read_phase_current_raw() 仍能看到 ADC seq 持续更新。
     */
    s_board_status.drv_nfault_active = hal_gpio_read_fault_pin();

    if (axis->fault_flags != 0u || s_board_status.drv_nfault_active) {
        board_disable_axis0_power_stage(axis);
        return false;
    }

    hal_gpio_set_gate_enable(false);
    hal_pwm_start_adc_trigger_only();

    s_board_status.pwm_disabled = true;
    s_board_status.drv_gate_enabled = false;
    s_board_status.adc_valid = hal_adc_samples_valid();

    /*
     * 刚启动 TIM1 ADC trigger-only 后，真实 ADC 可能还没有完成第一轮 injected
     * conversion，因此这里不能要求 adc_valid 已经为 true。current_offset_calibration
     * 会在 update 阶段等待 board_axis0_read_phase_current_raw() 看到新的 seq。
     *
     * 如果已经有有效快照，则顺便更新 VBUS 并做一次范围检查；没有快照时保持触发运行，
     * 等后续 ADC 回调产生样本。
     */
    if (s_board_status.adc_valid) {
        s_board_status.vbus_v = board_read_vbus_v();
        if (!board_vbus_in_safe_range(axis, s_board_status.vbus_v)) {
            board_disable_axis0_power_stage(axis);
            return false;
        }
    }

    return true;
}

void board_disable_axis0_power_stage(Axis0Context *axis)
{
    (void)axis;
    /*
     * 关断顺序要尽量简单可靠，允许从故障路径重复调用。
     * 重复调用必须是安全的。
     */
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);

    s_board_status.pwm_disabled = true;
    s_board_status.drv_gate_enabled = false;
}

void board_axis0_set_pwm_duty(float duty_a, float duty_b, float duty_c)
{
    /*
     * 板级层只转发 duty 到 HAL。
     * 真正移植时，HAL PWM 后端需要：
     * - 限制 duty 0..1；
     * - 写 TIM1 CCR1/CCR2/CCR3；
     * - 处理互补输出和死区。
     */
    hal_pwm_set_duty(duty_a, duty_b, duty_c);
}

bool board_axis0_read_phase_current_raw(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c)
{
    /*
     * 读取 ADC 原始值。
     * ODrive v3.6 Axis0 公开资料明确两路电流采样。
     * 不伪造 raw_c=2048；第三相原始值在 two-shunt mode 下无效，由 current_sensor
     * 明确使用 ic=-ia-ib 推算。
     */
    HalAdcSnapshot snapshot;
    if (!hal_adc_get_snapshot(&snapshot) || !snapshot.valid) {
        return false;
    }

    /*
     * seq 必须更新，否则说明 ISR 可能拿到了上一个 PWM 周期的旧样本。
     * 真实功率级闭环不能在这种情况下继续计算 FOC。
     */
    if (snapshot.seq == s_last_phase_adc_seq) {
        return false;
    }
    s_last_phase_adc_seq = snapshot.seq;

    *raw_a = snapshot.raw_u;
    *raw_b = snapshot.raw_v;
    *raw_c = 0u;
    return true;
}

bool board_axis0_has_third_current_sample(void)
{
    return false;
}

float board_read_vbus_v(void)
{
    /*
     * mock 换算：真实项目必须按 ODrive v3.6 母线分压比例修正。
     * 这个比例会直接影响欠压/过压保护，不能凭感觉填写。
     */
    return (float)hal_adc_get_vbus_raw() * s_vbus_scale_v_per_count;
}

void board_set_vbus_scale_v_per_count(float scale_v_per_count)
{
    if (scale_v_per_count > 0.0f) {
        s_vbus_scale_v_per_count = scale_v_per_count;
    }
}

bool board_read_drv_nfault(void)
{
    return hal_gpio_read_fault_pin();
}

BoardOdriveV36Status board_get_status(void)
{
    /* 后台调试可调用该函数查看板级基础状态。 */
    s_board_status.drv_nfault_active = board_read_drv_nfault();
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.vbus_v = board_read_vbus_v();
    return s_board_status;
}

void board_set_debug_led(bool on)
{
    /* TODO_PIN_CHECK：调试 LED 引脚需对照板卡实际版本确认。 */
    s_debug_led_on = on;
    (void)s_debug_led_on;
}

void board_debug_pulse_isr(void)
{
    /*
     * 移植时可翻转一个测试点 GPIO，用示波器测量 ISR 占空和抖动。
     * 注意真实 ISR 中翻转 GPIO 也有开销，只建议调试阶段启用。
     */
    s_debug_led_on = !s_debug_led_on;
}
