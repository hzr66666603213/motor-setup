#include "hal/hal_gpio.h"

/*
 * hal_gpio_mock.c
 *
 * GPIO mock 后端。
 * 默认 EN_GATE 关闭、nFAULT 无故障，仅用于 PC/框架测试。
 */

static bool s_gate_enabled = false;

bool hal_gpio_read_fault_pin(void)
{
    return false;
}

void hal_gpio_set_gate_enable(bool enabled)
{
    s_gate_enabled = enabled;
    (void)s_gate_enabled;
}

uint32_t hal_gpio_read_limit_inputs(void)
{
    return 0u;
}
