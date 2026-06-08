#include "board/board_odrive_v36.h"

#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"

/*
 * board_odrive_v36.c
 *
 * ODrive v3.6 板级安全抽象。
 * 本文件不直接调用 STM32 HAL；TIM/ADC/GPIO/SPI 的真实绑定由 src/hal/stm32f405 完成。
 *
 * 这个模块回答三个问题：
 * 1. 当前板子是否处于功率安全状态；
 * 2. 什么条件下允许使能 Axis0 功率级；
 * 3. 故障时如何最快关闭 PWM 和 EN_GATE。
 */

static BoardOdriveV36Status s_board_status;
static bool s_debug_led_on = false;
static float s_vbus_scale_v_per_count = 60.0f / 4095.0f;
static uint32_t s_last_phase_adc_seq = 0u;

static bool board_vbus_in_safe_range(const Axis0Context *axis, float vbus_v)
{
    return (vbus_v >= axis->config.protection.vbus_min_v) &&
           (vbus_v <= axis->config.protection.vbus_max_v);
}

bool board_init_power_safe(Axis0Context *axis)
{
    /*
     * 上电安全初始化：
     * - EN_GATE 拉低，DRV8301 不驱动 MOS；
     * - TIM1 MOE 关闭，三相功率输出关闭；
     * - TIM1/ADC trigger-only 启动，让同步 ADC 尽早开始产生样本；
     * - 不因为第一帧 ADC/VBUS 还没 valid 而卡在 BOOT。
     *
     * 严格的 ADC/VBUS 检查放在 board_enable_axis0_power_stage() 和
     * board_enable_axis0_power_stage_for_calibration() 中执行。
     */
    (void)axis;
    hal_gpio_set_gate_enable(false);
    hal_pwm_start_adc_trigger_only();

    s_board_status.pwm_disabled = true;
    s_board_status.drv_gate_enabled = false;
    s_board_status.drv_nfault_active = hal_gpio_read_fault_pin();
    s_board_status.drv0_status_valid = false;
    s_board_status.drv1_status_valid = false;
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.encoder_valid = false;
    s_board_status.vbus_v = s_board_status.adc_valid ? board_read_vbus_v() : 0.0f;

    return !s_board_status.drv_nfault_active;
}

bool board_enable_axis0_power_stage(Axis0Context *axis)
{
    /*
     * 闭环功率级准入检查必须严格：
     * - fault_flags 必须为 0；
     * - ODrive v3.6 EN_GATE/nFAULT 为 M0/M1 共享，Axis0-only 也要确认 nFAULT 未触发；
     * - ADC 样本必须 valid；
     * - 编码器必须已校准；
     * - VBUS 必须在配置的安全范围内。
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

bool board_enable_axis0_power_stage_for_calibration(Axis0Context *axis)
{
    /*
     * 校准/开环小电压注入允许编码器尚未校准，但 fault、nFAULT、ADC、VBUS 仍要严格检查。
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
     * 电流零偏校准需要 ADC 同步采样继续运行，但绝不能驱动 MOS：
     * - EN_GATE=0；
     * - TIM1 MOE=0；
     * - TIM1 仅保留 ADC trigger-only。
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
     * 刚启动 ADC trigger-only 后，真实 ADC 可能还没完成第一帧 conversion。
     * 此处不要求 adc_valid=true；校准 update 会等待新的 ADC seq。
     */
    if (s_board_status.adc_valid) {
        s_board_status.vbus_v = board_read_vbus_v();
        if (!board_vbus_in_safe_range(axis, s_board_status.vbus_v)) {
            board_disable_axis0_power_stage(axis);
            return false;
        }
    } else {
        s_board_status.vbus_v = 0.0f;
    }

    return true;
}

void board_disable_axis0_power_stage(Axis0Context *axis)
{
    (void)axis;
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);

    s_board_status.pwm_disabled = true;
    s_board_status.drv_gate_enabled = false;
}

void board_axis0_set_pwm_duty(float duty_a, float duty_b, float duty_c)
{
    hal_pwm_set_duty(duty_a, duty_b, duty_c);
}

bool board_axis0_read_phase_current_raw(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c)
{
    HalAdcSnapshot snapshot;
    if (!hal_adc_get_snapshot(&snapshot) || !snapshot.valid) {
        return false;
    }

    if (snapshot.seq == s_last_phase_adc_seq) {
        return false;
    }
    s_last_phase_adc_seq = snapshot.seq;

    *raw_a = snapshot.raw_u;
    *raw_b = snapshot.raw_v;
    *raw_c = 0u; /* two-shunt mode：第三相原始 ADC 无效，由 current_sensor 推算 ic=-ia-ib。 */
    return true;
}

bool board_axis0_has_third_current_sample(void)
{
    return false;
}

float board_read_vbus_v(void)
{
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
    s_board_status.drv_nfault_active = board_read_drv_nfault();
    s_board_status.adc_valid = hal_adc_samples_valid();
    s_board_status.vbus_v = s_board_status.adc_valid ? board_read_vbus_v() : 0.0f;
    return s_board_status;
}

void board_set_debug_led(bool on)
{
    s_debug_led_on = on;
    (void)s_debug_led_on;
}

void board_debug_pulse_isr(void)
{
    s_debug_led_on = !s_debug_led_on;
}
