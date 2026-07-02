#include "protection/fault.h"

#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "drivers/drv8301.h"

/*
 * fault.c
 *
 * 故障处理实现。
 * 任何快速故障都应尽快调用 fault_enter_safe_state()，
 * 确保功率级输出关闭，而不是等待后台通信或状态机慢速轮询。
 */

void fault_set(Axis *axis, FaultFlags fault)
{
    /* 故障使用 bit mask 累积，便于保留多个故障原因。 */
    axis->axis.error |= (uint32_t)fault;
    axis->axis.current_state = AXIS_STATE_FAULT;
}

void fault_clear_all(Axis *axis)
{
    /* 只清故障位，不自动重新使能闭环。 */
    axis->axis.error = FAULT_NONE;
}

bool fault_has(Axis *axis, FaultFlags fault)
{
    return (axis->axis.error & (uint32_t)fault) != 0u;
}

bool fault_has_any(Axis *axis)
{
    return axis->axis.error != FAULT_NONE;
}

void fault_enter_safe_state(Axis *axis)
{
    /* 故障状态下输出关断顺序保持简单明确：状态置 FAULT、关 PWM、all low、关 gate。 */
    axis->axis.current_state = AXIS_STATE_FAULT;
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);
}

void set_fault(Axis0Context *axis, Axis0FaultFlags fault)
{
    axis->fault_flags |= (uint32_t)fault;
    axis0_fault_enter_safe_state(axis);
}

void clear_faults(Axis0Context *axis)
{
    axis->fault_flags = AXIS0_FAULT_NONE;
    axis->state = AXIS0_STATE_IDLE;
    axis->requested_state = AXIS0_STATE_IDLE;
    axis->request_pending = false;
}

bool has_fault(const Axis0Context *axis, Axis0FaultFlags fault)
{
    return (axis->fault_flags & (uint32_t)fault) != 0u;
}

const char *get_fault_string(Axis0FaultFlags fault)
{
    switch (fault) {
    case AXIS0_FAULT_NONE: return "none";
    case AXIS0_FAULT_VBUS_UNDERVOLTAGE: return "vbus_undervoltage";
    case AXIS0_FAULT_VBUS_OVERVOLTAGE: return "vbus_overvoltage";
    case AXIS0_FAULT_PHASE_OVERCURRENT: return "phase_overcurrent";
    case AXIS0_FAULT_CURRENT_SENSOR_INVALID: return "current_sensor_invalid";
    case AXIS0_FAULT_ENCODER_INVALID: return "encoder_invalid";
    case AXIS0_FAULT_ENCODER_NO_MOVEMENT: return "encoder_no_movement";
    case AXIS0_FAULT_ENCODER_DIRECTION_ERROR: return "encoder_direction_error";
    case AXIS0_FAULT_DRV8301_FAULT: return "drv8301_fault";
    case AXIS0_FAULT_DRV8301_SPI_ERROR: return "drv8301_spi_error";
    case AXIS0_FAULT_PWM_NOT_ENABLED: return "pwm_not_enabled";
    case AXIS0_FAULT_MOTOR_CALIBRATION_FAILED: return "motor_calibration_failed";
    case AXIS0_FAULT_ENCODER_CALIBRATION_FAILED: return "encoder_calibration_failed";
    case AXIS0_FAULT_OVERTEMPERATURE: return "overtemperature";
    case AXIS0_FAULT_CONTROL_SATURATION: return "control_saturation";
    case AXIS0_FAULT_CURRENT_PROTECTION: return "current_protection";
    default: return "unknown";
    }
}

void axis0_fault_enter_safe_state(Axis0Context *axis)
{
    axis->state = AXIS0_STATE_FAULT;
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);
}
