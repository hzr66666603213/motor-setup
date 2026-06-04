#ifndef HAL_CAN_H
#define HAL_CAN_H

/*
 * hal_can.h
 *
 * CAN/FDCAN 硬件抽象接口。
 * 第一版协议只预留 CAN 参数通道，后续可扩展为自定义协议或 CANopen。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;   /* 标准或扩展 ID，具体格式由后端/协议约定 */
    uint8_t dlc;   /* 数据长度，经典 CAN 为 0..8 */
    uint8_t data[8]; /* 数据区 */
} HalCanFrame;

/* 初始化 CAN/FDCAN 外设、过滤器和中断/DMA。 */
bool hal_can_init(void);
/* 发送一帧 CAN；后台任务调用。 */
bool hal_can_send(const HalCanFrame *frame);
/* 非阻塞接收一帧 CAN；无帧返回 false。 */
bool hal_can_receive(HalCanFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* HAL_CAN_H */
