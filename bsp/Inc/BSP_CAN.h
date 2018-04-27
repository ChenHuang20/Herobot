#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void BSP_CAN_InitConfig(void);
void can_filter_config(CAN_HandleTypeDef* _hcan);


#ifdef __cplusplus
}
#endif
