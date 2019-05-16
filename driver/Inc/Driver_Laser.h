#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LASER_RCC_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define LASER_GPIO_PIN                 GPIO_PIN_9
#define LASER_GPIO                     GPIOC

#define LASER_ON                       HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_SET)
#define LASER_OFF                      HAL_GPIO_WritePin(LASER_GPIO,LASER_GPIO_PIN,GPIO_PIN_RESET)
#define LASER_TOGGLE                   HAL_GPIO_TogglePin(LASER_GPIO,LASER_GPIO_PIN)

void Laser_InitConfig(void);

#ifdef __cplusplus
}
#endif
