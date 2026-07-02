#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board/board_odrive_v36.h"
#include "hal/hal_adc.h"

static int s_failures = 0;

static void check_true(int cond, const char *name)
{
    if (!cond) {
        ++s_failures;
        printf("[FAIL] %s\n", name);
    } else {
        printf("[ OK ] %s\n", name);
    }
}

int main(void)
{
    Axis0Context axis;
    uint16_t raw_a = 0u;
    uint16_t raw_b = 0u;
    uint16_t raw_c = 0u;

    memset(&axis, 0, sizeof(axis));
    axis.config.protection.vbus_min_v = 0.0f;
    axis.config.protection.vbus_max_v = 60.0f;

    check_true(hal_adc_init(), "mock ADC init");
    check_true(board_init_power_safe(&axis),
               "BOOT does not wait for first ADC/VBUS frame");

    check_true(board_start_adc_sampling_without_power_stage(&axis),
               "start ADC sampling without power stage");

    const BoardOdriveV36Status status = board_get_status();
    check_true(!status.drv_gate_enabled, "EN_GATE remains disabled");
    check_true(status.pwm_disabled, "power PWM remains disabled");

    /*
     * 关键验证：
     * 即使功率级关闭，ADC trigger-only 模式仍应允许 ADC seq 更新。
     * mock 后端用 hal_adc_stm32f405_on_injected_complete(NULL) 模拟 injected conversion
     * 完成；真实后端必须在 HAL_ADCEx_InjectedConvCpltCallback() 中调用同名函数。
     */
    hal_adc_stm32f405_on_injected_complete(0);
    check_true(board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c),
               "first ADC raw read gets fresh seq");
    hal_adc_stm32f405_on_injected_complete(0);
    check_true(board_axis0_read_phase_current_raw(&raw_a, &raw_b, &raw_c),
               "second ADC raw read gets newer seq");

    if (s_failures == 0) {
        printf("board_adc_sampling_test passed\n");
        return 0;
    }
    return 1;
}
