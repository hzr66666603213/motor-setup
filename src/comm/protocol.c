#include "comm/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal/hal_can.h"
#include "hal/hal_uart.h"

/*
 * protocol.c
 *
 * 简单 UART 文本协议实现。
 * 特点：
 * - 固定行缓冲，不使用动态内存。
 * - 后台任务轮询，不进入 20 kHz ISR。
 * - 写入参数时委托 parameter_table 做范围和权限检查。
 */

static int parse_control_mode(const char *text, float *value)
{
    /* 支持可读性更好的文本模式名，也允许调用者直接写数字。 */
    if (strcmp(text, "torque") == 0) { *value = (float)CONTROL_MODE_TORQUE; return 1; }
    if (strcmp(text, "velocity") == 0) { *value = (float)CONTROL_MODE_VELOCITY; return 1; }
    if (strcmp(text, "position") == 0) { *value = (float)CONTROL_MODE_POSITION; return 1; }
    if (strcmp(text, "trajectory") == 0) { *value = (float)CONTROL_MODE_TRAJECTORY; return 1; }
    return 0;
}

static int parse_axis_state(const char *text, float *value)
{
    /* 状态请求文本映射；真正切换仍由状态机检查安全条件。 */
    if (strcmp(text, "idle") == 0) { *value = (float)AXIS_STATE_IDLE; return 1; }
    if (strcmp(text, "motor_calibration") == 0) { *value = (float)AXIS_STATE_MOTOR_CALIBRATION; return 1; }
    if (strcmp(text, "encoder_offset_calibration") == 0) { *value = (float)AXIS_STATE_ENCODER_OFFSET_CALIBRATION; return 1; }
    if (strcmp(text, "closed_loop") == 0) { *value = (float)AXIS_STATE_CLOSED_LOOP_CONTROL; return 1; }
    return 0;
}

void protocol_init(ProtocolContext *ctx, ParameterContext params)
{
    /* 行缓冲固定在上下文中，避免 malloc。 */
    ctx->params = params;
    ctx->rx_len = 0u;
    ctx->rx_line[0] = '\0';
}

bool protocol_handle_line(ProtocolContext *ctx, const char *line, char *response, size_t response_len)
{
    /* 命令格式：get <name> 或 set <name> <value>。 */
    char cmd[8] = {0};
    char name[48] = {0};
    char value_text[32] = {0};

    if (sscanf(line, "%7s %47s %31s", cmd, name, value_text) < 2) {
        snprintf(response, response_len, "err syntax\r\n");
        return false;
    }

    const ParamDescriptor *desc = parameter_find_by_name(name);
    if (!desc) {
        snprintf(response, response_len, "err unknown\r\n");
        return false;
    }

    if (strcmp(cmd, "get") == 0) {
        /* 读参数统一输出十进制文本；后续可按类型格式化。 */
        float value = 0.0f;
        if (!parameter_read(&ctx->params, desc->id, &value)) {
            snprintf(response, response_len, "err read\r\n");
            return false;
        }
        snprintf(response, response_len, "%s %.6g\r\n", desc->name, value);
        return true;
    }

    if (strcmp(cmd, "set") == 0) {
        /* set 对控制模式和状态请求支持文本别名。 */
        float value = 0.0f;
        if (desc->id == PARAM_AXIS_CONTROL_MODE && parse_control_mode(value_text, &value)) {
            /* parsed named mode */
        } else if (desc->id == PARAM_AXIS_REQUESTED_STATE && parse_axis_state(value_text, &value)) {
            /* parsed named state */
        } else {
            value = strtof(value_text, 0);
        }

        if (!parameter_write(&ctx->params, desc->id, value)) {
            snprintf(response, response_len, "err write\r\n");
            return false;
        }
        snprintf(response, response_len, "ok\r\n");
        return true;
    }

    snprintf(response, response_len, "err command\r\n");
    return false;
}

void protocol_poll_uart(ProtocolContext *ctx)
{
    /* 非阻塞轮询 UART，一次处理所有当前可读字节。 */
    uint8_t byte = 0u;
    char response[96];
    while (hal_uart_receive(&byte, 1u) == 1u) {
        if (byte == '\n' || byte == '\r') {
            ctx->rx_line[ctx->rx_len] = '\0';
            if (ctx->rx_len > 0u) {
                protocol_handle_line(ctx, ctx->rx_line, response, sizeof(response));
                hal_uart_send((const uint8_t *)response, strlen(response));
            }
            ctx->rx_len = 0u;
        } else if (ctx->rx_len < sizeof(ctx->rx_line) - 1u) {
            /* 固定行缓冲，超长命令会被丢弃并重新同步。 */
            ctx->rx_line[ctx->rx_len++] = (char)byte;
        } else {
            ctx->rx_len = 0u;
        }
    }
}

void protocol_poll_can(ProtocolContext *ctx)
{
    HalCanFrame frame;
    (void)ctx;
    while (hal_can_receive(&frame)) {
        /* CAN 二进制参数协议预留：目标工程可将 frame.id 映射到 ParamId。 */
    }
}
