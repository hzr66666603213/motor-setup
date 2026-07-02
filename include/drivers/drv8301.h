#ifndef DRV8301_H
#define DRV8301_H

/*
 * drv8301.h
 *
 * TI DRV8301 三相栅极驱动抽象。
 * ODrive v3.6 使用 DRV8301 类预驱，SPI 配置，EN_GATE 使能，nFAULT 故障输入。
 * 第一阶段只控制 Axis0。
 *
 * 安全约束：
 * - 上电默认 disable。
 * - 初始化和 SPI 配置在启动/低频状态机中执行。
 * - PWM ISR 内禁止阻塞式 SPI。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool over_current;        /* 过流故障 */
    bool over_temperature;    /* 过温故障 */
    bool under_voltage;       /* 驱动欠压故障 */
    bool gate_driver_fault;   /* 栅极驱动故障 */
    bool phase_fault;         /* 相故障 */
    bool unknown_fault;       /* 未解析故障 */
    bool spi_error;           /* SPI 通信失败 */
    uint16_t status1_raw;     /* 状态寄存器 1 原始值 */
    uint16_t status2_raw;     /* 状态寄存器 2 原始值 */
} Drv8301Status;

typedef struct {
    bool initialized;         /* SPI 配置是否完成 */
    bool enabled;             /* EN_GATE 是否使能 */
    uint8_t axis_index;       /* 0=M0/Axis0 DRV8301，1=M1/Axis1 DRV8301 */
    uint8_t spi_device_id;    /* SPI 片选设备 ID，HAL 后端据此选择 PC13/PC14 */
    float shunt_amp_gain_v_v; /* DRV8301 内部电流放大器增益，V/V，必须和 current_sensor 绑定 */
    uint16_t control1_config; /* Last successful CONTROL1 config without temporary GATE_RESET */
    uint16_t control2_config; /* Last successful CONTROL2 config */
    Drv8301Status status;     /* 最近一次状态 */
} Drv8301;

typedef struct {
    uint16_t status1;
    uint16_t status2;
    uint16_t control1;
    uint16_t control2;
} Drv8301Registers;

typedef struct {
    bool status1_ok;
    bool status2_ok;
    uint16_t status1_frame;
    uint16_t status2_frame;
    uint16_t status1_data;
    uint16_t status2_data;
} Drv8301StatusReadback;

#define DRV8301_GATE_CURRENT_1P7A       0u
#define DRV8301_GATE_CURRENT_0P7A       1u
#define DRV8301_GATE_CURRENT_0P25A      2u

#define DRV8301_OCP_MODE_CURRENT_LIMIT  0u
#define DRV8301_OCP_MODE_LATCH_SHUTDOWN 1u
#define DRV8301_OCP_MODE_REPORT_ONLY    2u
#define DRV8301_OCP_MODE_DISABLED       3u

#define DRV8301_SHUNT_GAIN_10V_PER_V    0u
#define DRV8301_SHUNT_GAIN_20V_PER_V    1u
#define DRV8301_SHUNT_GAIN_40V_PER_V    2u
#define DRV8301_SHUNT_GAIN_80V_PER_V    3u

#define DRV8301_CONTROL2_DC_CAL_CH1     (1u << 0)
#define DRV8301_CONTROL2_DC_CAL_CH2     (1u << 1)
#define DRV8301_CONTROL2_GAIN_SHIFT     2u
#define DRV8301_CONTROL2_GAIN_MASK      (3u << DRV8301_CONTROL2_GAIN_SHIFT)

bool drv8301_init(Drv8301 *drv);
bool drv8301_init_axis(Drv8301 *drv, uint8_t axis_index);
void drv8301_prepare_axis(Drv8301 *drv, uint8_t axis_index);
bool drv8301_enable(Drv8301 *drv);
void drv8301_disable(Drv8301 *drv);
bool drv8301_read_status(Drv8301 *drv);
bool drv8301_clear_faults(Drv8301 *drv);
bool drv8301_gate_reset(Drv8301 *drv);
bool drv8301_configure_for_6pwm(Drv8301 *drv);
bool drv8301_read_status_raw(Drv8301 *drv, Drv8301StatusReadback *readback);
bool drv8301_read_registers(Drv8301 *drv, Drv8301Registers *regs);
bool drv8301_configure_and_verify(Drv8301 *drv, Drv8301Registers *regs);
uint16_t drv8301_default_control1(void);
uint16_t drv8301_default_control2(void);
uint16_t drv8301_control1_with_gate_reset(uint16_t control1_config);
uint16_t drv8301_make_control2(uint8_t gain_code, bool dc_cal_ch1, bool dc_cal_ch2);
uint8_t drv8301_control2_gain_field(uint16_t control2);
bool drv8301_set_ocp_threshold(Drv8301 *drv, uint8_t threshold_code);
bool drv8301_set_gate_current(Drv8301 *drv, uint8_t gate_current_code);
bool drv8301_set_shunt_amp_gain(Drv8301 *drv, uint8_t gain_code);
bool drv8301_set_control2(Drv8301 *drv, uint16_t control2);
bool drv8301_set_dc_cal(Drv8301 *drv, bool ch1, bool ch2);
bool drv8301_has_fault(const Drv8301 *drv);

#ifdef __cplusplus
}
#endif

#endif /* DRV8301_H */
