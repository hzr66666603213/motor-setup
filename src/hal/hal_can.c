#include "hal/hal_can.h"

/*
 * hal_can.c
 *
 * CAN mock/stub 实现。
 * 当前发送直接返回成功，接收始终无数据。
 * 移植时应绑定 bxCAN 或 FDCAN。
 */

bool hal_can_init(void)
{
    return true;
}

bool hal_can_send(const HalCanFrame *frame)
{
    /* mock 不保存帧；真实后端应检查邮箱/FIFO 空间。 */
    (void)frame;
    return true;
}

bool hal_can_receive(HalCanFrame *frame)
{
    /* 非阻塞接口，无数据时返回 false。 */
    (void)frame;
    return false;
}
