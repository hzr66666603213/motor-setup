#include "hal/hal_gpio.h"

/*
 * hal_gpio_stm32f405.c
 *
 * ODrive v3.6 / STM32F405RG GPIO 真实后端。
 *
 * 已按当前公开资料和前期板级定义使用：
 * - EN_GATE：PB12，M0/M1 两颗 DRV8301 共用；
 * - nFAULT：PD2，M0/M1 共用故障输入，低有效。
 *
 * 注意：
 * - Axis0-only 调试时也必须把共用 EN_GATE/nFAULT 当成整板资源处理；
 * - 该文件只能在 STM32 固件工程中编译，PC/Simulink 测试不要编译它；
 * - CubeMX 工程必须启用 GPIOB/GPIOD 时钟并配置 PB12 输出、PD2 输入上拉。
 */

#include "stm32f4xx_hal.h"

#define ODRV36_EN_GATE_GPIO_Port GPIOB
#define ODRV36_EN_GATE_Pin       GPIO_PIN_12
#define ODRV36_NFAULT_GPIO_Port  GPIOD
#define ODRV36_NFAULT_Pin        GPIO_PIN_2

bool hal_gpio_read_fault_pin(void)
{
    /* DRV8301 nFAULT 为低有效：读到 RESET 表示至少一个 DRV 报故障。 */
    return HAL_GPIO_ReadPin(ODRV36_NFAULT_GPIO_Port, ODRV36_NFAULT_Pin) == GPIO_PIN_RESET;
}

void hal_gpio_set_gate_enable(bool enabled)
{
    /*
     * EN_GATE 为共用使能。
     * fault/idle/disable 路径必须传 false，让两颗 DRV8301 都停止驱动 MOS。
     */
    HAL_GPIO_WritePin(ODRV36_EN_GATE_GPIO_Port,
                      ODRV36_EN_GATE_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t hal_gpio_read_limit_inputs(void)
{
    /* 第一版没有接限位/急停输入，保留接口给后续机器人关节模组扩展。 */
    return 0u;
}
