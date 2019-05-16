#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint64_t Tick;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern uint64_t time(void);
extern void usleep(uint32_t us);
void BSP_TIM_InitConfig(void);

#ifdef __cplusplus
}
#endif
