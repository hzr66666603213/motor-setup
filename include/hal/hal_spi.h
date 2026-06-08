#ifndef HAL_SPI_H
#define HAL_SPI_H

/*
 * hal_spi.h
 *
 * SPI 硬件抽象接口。
 * 主要用于 SPI 磁编码器和可配置栅极驱动芯片。
 * 注意：20 kHz ISR 内不应调用阻塞 SPI；快速传感器采样应使用 DMA 或预读结果。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 阻塞式 SPI 传输接口；仅建议在后台/初始化/低频任务中使用。 */
bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length);
bool hal_spi_transfer_device(uint8_t bus_id, uint8_t device_id, const uint8_t *tx, uint8_t *rx, size_t length);
/* DMA SPI 传输接口；具体完成通知由目标工程补充。 */
bool hal_spi_transfer_dma(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SPI_H */
