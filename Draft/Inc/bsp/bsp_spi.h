#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "struct_typedef.h"

extern void SPI1_DMA_init(uint64_t tx_buf, uint64_t rx_buf, uint16_t num);
extern void SPI1_DMA_enable(uint64_t tx_buf, uint64_t rx_buf, uint16_t ndtr);

#endif
