#ifndef FAULT_H
#define FAULT_H

/*
 * fault.h
 *
 * 故障位管理和安全状态入口。
 * fault 模块允许调用 HAL 抽象关闭 PWM/EN_GATE，但不直接访问 STM32 寄存器。
 */

#include <stdbool.h>
#include <stdint.h>
#include "app/axis0_types.h"
#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 设置故障位，并把轴状态置为 FAULT。 */
void fault_set(Axis *axis, FaultFlags fault);
/* 清除所有故障位；通常只允许在用户确认安全后调用。 */
void fault_clear_all(Axis *axis);
/* 查询指定故障位是否存在。 */
bool fault_has(Axis *axis, FaultFlags fault);
/* 查询是否存在任意故障。 */
bool fault_has_any(Axis *axis);
/* 立即进入安全输出：关闭 PWM、all low、拉低 EN_GATE。 */
void fault_enter_safe_state(Axis *axis);

typedef enum {
    AXIS0_FAULT_NONE                       = 0u,
    AXIS0_FAULT_VBUS_UNDERVOLTAGE         = 1u << 0,
    AXIS0_FAULT_VBUS_OVERVOLTAGE          = 1u << 1,
    AXIS0_FAULT_PHASE_OVERCURRENT         = 1u << 2,
    AXIS0_FAULT_CURRENT_SENSOR_INVALID    = 1u << 3,
    AXIS0_FAULT_ENCODER_INVALID           = 1u << 4,
    AXIS0_FAULT_ENCODER_NO_MOVEMENT       = 1u << 5,
    AXIS0_FAULT_ENCODER_DIRECTION_ERROR   = 1u << 6,
    AXIS0_FAULT_DRV8301_FAULT             = 1u << 7,
    AXIS0_FAULT_DRV8301_SPI_ERROR         = 1u << 8,
    AXIS0_FAULT_PWM_NOT_ENABLED           = 1u << 9,
    AXIS0_FAULT_MOTOR_CALIBRATION_FAILED  = 1u << 10,
    AXIS0_FAULT_ENCODER_CALIBRATION_FAILED= 1u << 11,
    AXIS0_FAULT_OVERTEMPERATURE           = 1u << 12,
    AXIS0_FAULT_CONTROL_SATURATION        = 1u << 13,
    AXIS0_FAULT_CURRENT_PROTECTION        = 1u << 14,
    AXIS0_FAULT_ADC_HARDWARE_UNPAIRED    = 1u << 15,
    AXIS0_FAULT_ADC_PUBLISH_SEQUENCE      = 1u << 16,
    AXIS0_FAULT_DIAGNOSTIC_ISR_OVERRUN    = 1u << 17,
    AXIS0_FAULT_ADC_CALLBACK_OVERRUN      = 1u << 18,
    AXIS0_FAULT_DIAGNOSTIC_DATA_INVALID   = 1u << 19,
    AXIS0_FAULT_CURRENT_SENSOR_NOISE_REJECTED = 1u << 20,
    AXIS0_FAULT_CURRENT_SENSOR_ADMISSION_OVERRUN = 1u << 21,
    AXIS0_FAULT_ELECTRICAL_OFFSET_CALIBRATION_FAILED = 1u << 22
} Axis0FaultFlags;

/* ODrive v3.6 Axis0 专用故障接口。 */
void set_fault(Axis0Context *axis, Axis0FaultFlags fault);
void clear_faults(Axis0Context *axis);
bool has_fault(const Axis0Context *axis, Axis0FaultFlags fault);
const char *get_fault_string(Axis0FaultFlags fault);
void axis0_fault_enter_safe_state(Axis0Context *axis);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
