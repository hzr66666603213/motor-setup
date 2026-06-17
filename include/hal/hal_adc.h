#ifndef HAL_ADC_H
#define HAL_ADC_H

/*
 * hal_adc.h
 *
 * ADC 硬件抽象接口。
 *
 * 职责：
 * - 向 FOC/board 层提供 PWM 同步采样后的 ADC 原始 count；
 * - 不在 HAL 层做电流、电压、温度物理量换算；
 * - 真实 STM32F405 后端可以使用 injected ADC、regular DMA 或混合方式实现。
 *
 * 调用频率：
 * - phase current / snapshot：20 kHz PWM ISR 内读取，必须非阻塞；
 * - temperature：1 kHz 或后台读取；
 * - init：启动阶段调用。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t u; /* U/A 相电流 ADC 原始值，count */
    uint16_t v; /* V/B 相电流 ADC 原始值，count */
    uint16_t w; /* W/C 相电流 ADC 原始值，count；two-shunt 模式下可为 0 */
} HalAdcPhaseRaw;

/*
 * PWM 同步 ADC 快照。
 *
 * seq 每完成一次同步采样递增一次。电流环 ISR 可以保存上一周期 seq，
 * 如果本周期 seq 没有变化或 valid=false，应触发 ADC invalid fault，
 * 禁止继续使用旧电流样本计算 FOC。
 */
typedef struct {
    uint16_t raw_u;        /* U/A 相电流 ADC 原始值，count */
    uint16_t raw_v;        /* V/B 相电流 ADC 原始值，count */
    uint16_t raw_w;        /* W/C 相电流 ADC 原始值，count；two-shunt 模式下可为 0 */
    uint16_t raw_vbus;     /* 母线电压 ADC 原始值，count */
    uint16_t raw_mos_temp; /* MOS/板温 ADC 原始值，count */
    uint32_t seq;          /* 同步采样序号 */
    bool valid;            /* 当前快照是否有效 */
} HalAdcSnapshot;

typedef struct {
    uint32_t injected_start_adc1_status;
    uint32_t injected_start_adc2_status;
    uint32_t irq_count;
    uint32_t adc1_callback_count;
    uint32_t adc2_callback_count;
    uint32_t snapshot_count;
} HalAdcDiagnostics;

bool hal_adc_init(void);
bool hal_adc_get_phase_current_raw(HalAdcPhaseRaw *raw);
bool hal_adc_get_snapshot(HalAdcSnapshot *snapshot);
uint16_t hal_adc_get_vbus_raw(void);
uint16_t hal_adc_get_mos_temperature_raw(void);
uint16_t hal_adc_get_motor_temperature_raw(void);
bool hal_adc_samples_valid(void);
void hal_adc_get_diagnostics(HalAdcDiagnostics *diagnostics);

/*
 * STM32F405 真实后端的 ADC injected conversion 完成通知。
 * CubeMX 的 HAL_ADCEx_InjectedConvCpltCallback() 中调用该函数，
 * seq 只在这里递增，hal_adc_get_snapshot() 只复制快照。
 */
/*
 * 真实 STM32 后端请从 HAL_ADCEx_InjectedConvCpltCallback(hadc) 调用本函数。
 * 参数使用 void *，避免公共 HAL 头文件依赖 stm32f4xx_hal.h。
 * STM32F405 后端只有在 hadc1/hadc2 本周期数据都有效后才会递增 snapshot.seq。
 */
void hal_adc_stm32f405_on_injected_complete(void *hadc);
void hal_adc_stm32f405_on_irq_enter(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H */
