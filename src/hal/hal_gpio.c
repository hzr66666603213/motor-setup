#include "hal/hal_gpio.h"

/*
 * hal_gpio.c
 *
 * GPIO mock/stub 实现。
 * 当前不访问真实引脚；移植时应绑定 EN_GATE、nFAULT、限位开关等 IO。
 */

static bool s_gate_enabled = false;

bool hal_gpio_read_fault_pin(void)
{
    /* mock 默认无栅极驱动故障。 */
    return false;
}

void hal_gpio_set_gate_enable(bool enabled)
{
    /* 真实硬件中这里应直接控制 EN_GATE 引脚，故障路径必须快速可靠。 */
    s_gate_enabled = enabled;
    (void)s_gate_enabled;
}

uint32_t hal_gpio_read_limit_inputs(void)
{
    return 0u;
}
