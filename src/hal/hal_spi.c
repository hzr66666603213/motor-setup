#include "hal/hal_spi.h"

/*
 * hal_spi.c
 *
 * SPI mock/stub 实现。
 * 当前将 tx 回环到 rx，便于上层接口测试。
 * 移植时应替换为 STM32 SPI HAL/LL 或 DMA 后端。
 */

bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)bus_id;
    for (size_t i = 0u; i < length; ++i) {
        if (rx) {
            rx[i] = tx ? tx[i] : 0u;
        }
    }
    return true;
}

bool hal_spi_transfer_dma(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    /* mock 中 DMA 与普通传输等价；真实工程应异步返回并由回调置位完成标志。 */
    return hal_spi_transfer(bus_id, tx, rx, length);
}
