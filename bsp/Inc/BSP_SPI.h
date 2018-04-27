#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_SPI_InitConfig(void);
int spi1_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);
int spi1_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);


#ifdef __cplusplus
}
#endif
