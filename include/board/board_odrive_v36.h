#ifndef BOARD_ODRIVE_V36_H
#define BOARD_ODRIVE_V36_H

/*
 * board_odrive_v36.h
 *
 * ODrive v3.6 板级抽象。
 * 第一阶段只启用 Axis0，Axis1 资源保留但默认禁用。
 *
 * 引脚来源说明：
 * - STM32F405RG、DRV8301、TIM1/TIM3/SPI3 等信息需对照 ODrive v3.6 原理图最终确认。
 * - 已从公开 ODrive v3.6 pinout 资料核对到的引脚在宏中直接列出。
 * - 资料不充分或不同硬件版本可能有差异的资源使用 TODO_PIN_CHECK。
 */

#include <stdbool.h>
#include <stdint.h>
#include "app/axis0_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TODO_PIN_CHECK (-1)

/* Axis0 三相 6PWM：ODrive v3.6 Axis0 使用 TIM1。 */
#define ODRV36_AXIS0_PWM_TIMER        "TIM1"
#define ODRV36_AXIS0_AH_PORT_PIN      "PA8"   /* M0_AH, TIM1_CH1 */
#define ODRV36_AXIS0_BH_PORT_PIN      "PA9"   /* M0_BH, TIM1_CH2 */
#define ODRV36_AXIS0_CH_PORT_PIN      "PA10"  /* M0_CH, TIM1_CH3 */
#define ODRV36_AXIS0_AL_PORT_PIN      "PB13"  /* M0_AL, TIM1_CH1N */
#define ODRV36_AXIS0_BL_PORT_PIN      "PB14"  /* M0_BL, TIM1_CH2N */
#define ODRV36_AXIS0_CL_PORT_PIN      "PB15"  /* M0_CL, TIM1_CH3N */

/* Axis0 三相电流采样：公开资料明确 M0_SO1/M0_SO2，第三相通常由 ia+ib+ic=0 推算。 */
#define ODRV36_AXIS0_SO1_PORT_PIN     "PC0"   /* M0_SO1, ADC2_IN10，TODO: 核对采样相对应关系 */
#define ODRV36_AXIS0_SO2_PORT_PIN     "PC1"   /* M0_SO2, ADC2_IN11，TODO: 核对采样相对应关系 */
#define ODRV36_AXIS0_SO3_PORT_PIN     "TODO_PIN_CHECK" /* ODrive v3.x 常见两路电流采样，第三相由计算得到 */

/* 母线电压和温度采样。 */
#define ODRV36_VBUS_S_PORT_PIN        "PA6"   /* VBUS_S, ADC1_IN6，需按原理图核对分压比例 */
#define ODRV36_AXIS0_TEMP_PORT_PIN    "PC5"   /* M0_TEMP, ADC1_IN15 */

/* DRV8301 SPI：Axis0/Axis1 共用 SPI3，片选分开。 */
#define ODRV36_DRV_SPI                "SPI3"
#define ODRV36_DRV_SPI_SCK_PORT_PIN   "PC10"
#define ODRV36_DRV_SPI_MISO_PORT_PIN  "PC11"
#define ODRV36_DRV_SPI_MOSI_PORT_PIN  "PC12"
#define ODRV36_DRV0_NCS_PORT_PIN      "PC13"  /* Axis0 DRV8301 CS */
#define ODRV36_DRV1_NCS_PORT_PIN      "PC14"  /* Axis1 DRV8301 CS，第一阶段禁用 */
#define ODRV36_DRV_EN_GATE_PORT_PIN   "PB12"  /* Axis0/Axis1 DRV8301 EN_GATE */
#define ODRV36_DRV_NFAULT_PORT_PIN    "PD2"   /* DRV8301 nFAULT */

/* Encoder0 ABZ：A/B 使用 TIM3 Encoder Mode，Z/index 需人工核对。 */
#define ODRV36_ENCODER0_TIMER         "TIM3"
#define ODRV36_ENCODER0_A_PORT_PIN    "PB4"   /* M0_ENC_A, TIM3_CH1 */
#define ODRV36_ENCODER0_B_PORT_PIN    "PB5"   /* M0_ENC_B, TIM3_CH2 */
#define ODRV36_ENCODER0_Z_PORT_PIN    "PC9"   /* M0_ENC_Z */

/* 调试/通信资源。 */
#define ODRV36_USB_DM_PORT_PIN        "PA11"
#define ODRV36_USB_DP_PORT_PIN        "PA12"
#define ODRV36_UART_TX_PORT_PIN       "PA2"   /* USART2_TX，可选 */
#define ODRV36_UART_RX_PORT_PIN       "PA3"   /* USART2_RX，可选 */
#define ODRV36_CAN_RX_PORT_PIN        "PB8"   /* CAN_R，可选 */
#define ODRV36_CAN_TX_PORT_PIN        "PB9"   /* CAN_D，可选 */
#define ODRV36_BRAKE_RES_PORT_PIN     "TODO_PIN_CHECK"

typedef struct {
    bool pwm_disabled;               /* PWM 是否处于关闭状态 */
    bool drv_gate_enabled;           /* DRV8301 EN_GATE 是否使能 */
    bool drv_nfault_active;          /* nFAULT 是否有效，true 表示故障 */
    bool adc_valid;                  /* ADC 采样是否有效 */
    bool encoder_valid;              /* 编码器是否有效 */
    float vbus_v;                    /* 母线电压，V */
} BoardOdriveV36Status;

/* 板级初始化到功率安全状态：PWM off、EN_GATE off、duty 清零、读取基础状态。 */
bool board_init_power_safe(Axis0Context *axis);

/* 安全使能 Axis0 功率级；必须满足无故障、VBUS 正常、ADC/编码器有效。 */
bool board_enable_axis0_power_stage(Axis0Context *axis);

/* 立即关闭 Axis0 功率级；可从故障路径调用。 */
void board_disable_axis0_power_stage(Axis0Context *axis);

/* 设置 Axis0 三相 duty；输入范围 0..1，底层应再做限幅。 */
void board_axis0_set_pwm_duty(float duty_a, float duty_b, float duty_c);

/* 读取 Axis0 两/三相电流 ADC 原始值；第三相可由 current_sensor 模块推算。 */
bool board_axis0_read_phase_current_raw(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c);

/* 读取母线电压，单位 V。 */
float board_read_vbus_v(void);

/* 读取 DRV8301 nFAULT，true 表示故障有效。 */
bool board_read_drv_nfault(void);

/* 读取并缓存板级状态。 */
BoardOdriveV36Status board_get_status(void);

/* 调试 LED，具体引脚请在移植时按硬件确认。 */
void board_set_debug_led(bool on);

/* ISR 调试脉冲，可用于示波器观察 20kHz 中断执行时间。 */
void board_debug_pulse_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_ODRIVE_V36_H */
