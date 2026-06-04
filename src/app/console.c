#include "app/console.h"

#include <stdio.h>
#include <string.h>
#include "hal/hal_uart.h"
#include "protection/fault.h"

/*
 * console.c
 *
 * 简单文本命令：
 * get vbus
 * set current_limit 1.0
 * request_state encoder_calibration
 * clear_faults
 *
 * 运行上下文：
 * - 后台任务或低频调度器中调用 console_poll()。
 * - 不允许在 PWM ISR 中调用，因为这里会解析字符串、snprintf、UART 发送。
 *
 * 设计目标：
 * - 用普通串口助手/USB CDC 就能调试；
 * - 命令保持可读；
 * - 危险写入通过 parameter_table 做范围检查；
 * - 高风险状态切换只写 requested_state，真正切换由状态机做安全检查。
 */

void console_init(Axis0Console *console, Axis0Context *axis, EncoderMt6701AbzState *encoder)
{
    /*
     * console 只保存指针，不拥有 axis/encoder 对象。
     * 这些对象应在 main/app 层静态分配，避免动态内存。
     */
    console->params.axis = axis;
    console->params.encoder = encoder;
    console->axis = axis;
    console->len = 0u;
    console->line[0] = '\0';
}

bool console_handle_line(Axis0Console *console, const char *line, char *response, size_t response_len)
{
    /*
     * 最简单的三段式解析：
     * cmd name value
     * 例如：
     * - get vbus
     * - set current_limit 1.0
     * - request_state closed_loop
     */
    char cmd[24] = {0};
    char name[48] = {0};
    char value[48] = {0};
    int n = sscanf(line, "%23s %47s %47s", cmd, name, value);

    if (n <= 0) {
        /* 空行不执行任何动作。 */
        snprintf(response, response_len, "err empty\r\n");
        return false;
    }

    if (strcmp(cmd, "get") == 0 && n >= 2) {
        if (strcmp(name, "state") == 0) {
            /* state 用字符串输出，比直接输出枚举值更适合串口调试。 */
            snprintf(response, response_len, "state %s\r\n", axis0_state_to_string(console->axis->state));
            return true;
        }
        float out = 0.0f;
        if (axis0_param_get(&console->params, name, &out)) {
            /* 统一用浮点格式输出观测量，方便复制到日志或脚本。 */
            snprintf(response, response_len, "%s %.6g\r\n", name, out);
            return true;
        }
        snprintf(response, response_len, "err unknown_param\r\n");
        return false;
    }

    if (strcmp(cmd, "set") == 0 && n >= 3) {
        /*
         * 闭环运行中禁止改 pole_pairs / encoder_cpr。
         * 这些参数会直接影响电角度或角度比例，运行中修改会造成角度突变。
         */
        if (console->axis->state == AXIS0_STATE_CLOSED_LOOP_CONTROL &&
            (strcmp(name, "pole_pairs") == 0 || strcmp(name, "encoder_cpr") == 0)) {
            snprintf(response, response_len, "err unsafe_state\r\n");
            return false;
        }
        if (axis0_param_set(&console->params, name, value)) {
            /* 真正的范围检查在 axis0_param_set() 中完成。 */
            snprintf(response, response_len, "ok\r\n");
            return true;
        }
        snprintf(response, response_len, "err range_or_name\r\n");
        return false;
    }

    if (strcmp(cmd, "request_state") == 0 && n >= 2) {
        /*
         * request_state 只写 requested_state。
         * 例如 closed_loop 是否允许，会由 axis_update_1khz() 检查校准和故障状态。
         */
        Axis0StateId state;
        if (!axis0_parse_state(name, &state)) {
            snprintf(response, response_len, "err bad_state\r\n");
            return false;
        }
        console->axis->requested_state = state;
        snprintf(response, response_len, "ok\r\n");
        return true;
    }

    if (strcmp(cmd, "clear_faults") == 0) {
        /*
         * clear_faults 只清软件故障并回 IDLE。
         * 如果硬件 nFAULT 仍然存在，后台保护会再次置故障。
         */
        clear_faults(console->axis);
        snprintf(response, response_len, "ok\r\n");
        return true;
    }

    if (strcmp(cmd, "save_config") == 0) {
        /* 第一版先做 stub；后续应写 Flash，并带 CRC 和版本号。 */
        snprintf(response, response_len, "ok stub_save_config\r\n");
        return true;
    }

    if (strcmp(cmd, "reboot") == 0) {
        /* 第一版先做 stub；移植后可调用 NVIC_SystemReset()。 */
        snprintf(response, response_len, "ok stub_reboot\r\n");
        return true;
    }

    snprintf(response, response_len, "err command\r\n");
    return false;
}

void console_poll(Axis0Console *console)
{
    /*
     * 非阻塞轮询 UART/USB CDC。
     * hal_uart_receive() 应从 ring buffer 中取数据，不能在这里等待。
     */
    uint8_t byte = 0u;
    char response[128];
    while (hal_uart_receive(&byte, 1u) == 1u) {
        if (byte == '\r' || byte == '\n') {
            /* 收到行结束符后处理一整行命令。 */
            console->line[console->len] = '\0';
            if (console->len > 0u) {
                console_handle_line(console, console->line, response, sizeof(response));
                hal_uart_send((const uint8_t *)response, strlen(response));
            }
            console->len = 0u;
        } else if (console->len < sizeof(console->line) - 1u) {
            /* 固定长度行缓冲，避免 malloc。 */
            console->line[console->len++] = (char)byte;
        } else {
            /* 超长命令丢弃，防止缓冲区溢出。 */
            console->len = 0u;
            hal_uart_send((const uint8_t *)"err line_too_long\r\n", 19u);
        }
    }
}
