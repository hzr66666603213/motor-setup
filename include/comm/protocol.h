#ifndef PROTOCOL_H
#define PROTOCOL_H

/*
 * protocol.h
 *
 * 简单通信协议框架。
 * 第一版提供 UART 文本命令：
 *   get bus_voltage
 *   set motor.current_limit 10.0
 * CAN 接口预留为二进制参数协议入口。
 */

#include <stddef.h>
#include "comm/parameter_table.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ParameterContext params; /* 参数访问上下文 */
    char rx_line[96];        /* UART 行缓冲，固定长度，避免动态内存 */
    size_t rx_len;           /* 当前行缓冲长度 */
} ProtocolContext;

/* 初始化协议上下文。 */
void protocol_init(ProtocolContext *ctx, ParameterContext params);
/* 轮询 UART 接收并处理完整行，后台任务调用。 */
void protocol_poll_uart(ProtocolContext *ctx);
/* 轮询 CAN 接收，预留二进制参数协议。 */
void protocol_poll_can(ProtocolContext *ctx);
/* 处理一行文本命令，便于测试和 UART 复用。 */
bool protocol_handle_line(ProtocolContext *ctx, const char *line, char *response, size_t response_len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
