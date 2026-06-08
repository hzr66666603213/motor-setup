#include "protection/protection.h"

#include <math.h>
#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "board/board_odrive_v36.h"
#include "protection/fault.h"

static ProtectionConfig s_default_config;
static uint16_t s_saturation_count = 0u;
static float s_previous_encoder_angle_rad = 0.0f;

/*
 * protection.c
 *
 * 保护模块分为：
 * - fast：相电流、nFAULT、ADC 有效性，适合 PWM ISR。
 * - slow：母线电压、温度、编码器状态和角度跳变，适合低频任务。
 */

void protection_set_defaults(ProtectionConfig *config)
{
    /* 默认阈值适合 24~48 V 母线的早期开发，实际产品需按硬件额定值设置。 */
    config->bus_undervoltage_v = 10.0f;
    config->bus_overvoltage_v = 56.0f;
    config->phase_overcurrent_a = 60.0f;
    config->mos_overtemperature_c = 90.0f;
    config->motor_overtemperature_c = 100.0f;
    config->encoder_jump_rad = 1.0f;
    config->saturation_limit_count = 100u;
    s_default_config = *config;
}

void protection_check_fast(Axis *axis)
{
    const ProtectionConfig *config = &s_default_config;
    /* 过流属于快速故障，应尽可能靠近 ADC/PWM ISR 检查。 */
    if (config->phase_overcurrent_a > 0.0f &&
        (fabsf(axis->foc_state.ia_a) > config->phase_overcurrent_a ||
         fabsf(axis->foc_state.ib_a) > config->phase_overcurrent_a ||
         fabsf(axis->foc_state.ic_a) > config->phase_overcurrent_a)) {
        fault_set(axis, FAULT_PHASE_OVERCURRENT);
    }

    if (hal_gpio_read_fault_pin()) {
        /* 栅极驱动 nFAULT 通常代表过流、欠压、过温或驱动内部错误。 */
        fault_set(axis, FAULT_GATE_DRIVER);
    }

    if (!hal_adc_samples_valid()) {
        fault_set(axis, FAULT_ADC_ERROR);
    }
}

void protection_check_slow(Axis *axis, const ProtectionConfig *config)
{
    /* 电压和温度变化较慢，低频检查即可。 */
    if (axis->motor_state.bus_voltage_v < config->bus_undervoltage_v) {
        fault_set(axis, FAULT_BUS_UNDERVOLTAGE);
    }
    if (axis->motor_state.bus_voltage_v > config->bus_overvoltage_v) {
        fault_set(axis, FAULT_BUS_OVERVOLTAGE);
    }
    if (axis->motor_state.mos_temperature_c > config->mos_overtemperature_c) {
        fault_set(axis, FAULT_MOS_OVERTEMPERATURE);
    }
    if (axis->motor_state.motor_temperature_c > config->motor_overtemperature_c) {
        fault_set(axis, FAULT_MOTOR_OVERTEMPERATURE);
    }
    if (axis->encoder.has_error) {
        fault_set(axis, FAULT_ENCODER_ERROR);
    }
    if (!axis->encoder.is_ready) {
        fault_set(axis, FAULT_ENCODER_NO_RESPONSE);
    }

    const float delta = fabsf(axis->encoder.mechanical_angle_rad - s_previous_encoder_angle_rad);
    /* 简单角度跳变检查；实际实现应考虑 2*pi 跨界和最大机械速度。 */
    if (delta > config->encoder_jump_rad && axis->axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        fault_set(axis, FAULT_ENCODER_ANGLE_JUMP);
    }
    s_previous_encoder_angle_rad = axis->encoder.mechanical_angle_rad;
}

void protection_note_output_saturation(Axis *axis, const ProtectionConfig *config, int saturated)
{
    /* 长时间电压/电流输出饱和可能代表参数错误、机械卡滞或供电能力不足。 */
    if (saturated) {
        if (s_saturation_count < UINT16_MAX) {
            s_saturation_count++;
        }
    } else {
        s_saturation_count = 0u;
    }

    if (s_saturation_count > config->saturation_limit_count) {
        fault_set(axis, FAULT_CONTROL_SATURATION);
    }
}

void axis0_protection_check_fast(Axis0Context *axis,
                                 const EncoderMt6701AbzState *encoder,
                                 const Drv8301 *drv0,
                                 const Drv8301 *drv1)
{
    if (axis->rt.vbus_v < axis->config.protection.vbus_min_v) {
        set_fault(axis, AXIS0_FAULT_VBUS_UNDERVOLTAGE);
    }
    if (axis->rt.vbus_v > axis->config.protection.vbus_max_v) {
        set_fault(axis, AXIS0_FAULT_VBUS_OVERVOLTAGE);
    }
    if (axis->rt.ia_a > axis->config.protection.phase_overcurrent_a ||
        axis->rt.ia_a < -axis->config.protection.phase_overcurrent_a ||
        axis->rt.ib_a > axis->config.protection.phase_overcurrent_a ||
        axis->rt.ib_a < -axis->config.protection.phase_overcurrent_a ||
        axis->rt.ic_a > axis->config.protection.phase_overcurrent_a ||
        axis->rt.ic_a < -axis->config.protection.phase_overcurrent_a) {
        set_fault(axis, AXIS0_FAULT_PHASE_OVERCURRENT);
    }
    if (!hal_adc_samples_valid()) {
        set_fault(axis, AXIS0_FAULT_CURRENT_SENSOR_INVALID);
    }
    if (axis->config.protection.encoder_error_enable && !encoder->valid) {
        set_fault(axis, AXIS0_FAULT_ENCODER_INVALID);
    }
    if (axis->config.protection.drv_fault_enable &&
        ((drv0 && drv8301_has_fault(drv0)) ||
         (drv1 && drv8301_has_fault(drv1)))) {
        set_fault(axis, AXIS0_FAULT_DRV8301_FAULT);
    }
    if (board_read_drv_nfault()) {
        set_fault(axis, AXIS0_FAULT_DRV8301_FAULT);
    }
}

void axis0_protection_check_slow(Axis0Context *axis,
                                 const EncoderMt6701AbzState *encoder,
                                 Drv8301 *drv0,
                                 Drv8301 *drv1)
{
    if (axis->config.protection.encoder_error_enable && !encoder->valid) {
        set_fault(axis, AXIS0_FAULT_ENCODER_INVALID);
    }
    /*
     * ODrive v3.6 的 EN_GATE/nFAULT 是 M0/M1 共用。
     * 即使第一阶段只控制 Axis0，也要周期读取/处理 M1 DRV8301 状态：
     * - M1 必须初始化；
     * - M1 gate 保持关闭；
     * - M1 SPI 或状态故障同样会让共享 nFAULT 触发。
     */
    if (drv0 && !drv8301_read_status(drv0)) {
        set_fault(axis, AXIS0_FAULT_DRV8301_SPI_ERROR);
    }
    if (drv1 && !drv8301_read_status(drv1)) {
        set_fault(axis, AXIS0_FAULT_DRV8301_SPI_ERROR);
    }
    if ((drv0 && drv0->status.spi_error) ||
        (drv1 && drv1->status.spi_error)) {
        set_fault(axis, AXIS0_FAULT_DRV8301_SPI_ERROR);
    }
    if ((drv0 && drv8301_has_fault(drv0)) ||
        (drv1 && drv8301_has_fault(drv1))) {
        set_fault(axis, AXIS0_FAULT_DRV8301_FAULT);
    }
}
