#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define abs(x) ((x)>0? (x):(-(x)))
extern void CanSend_InitConfig(void);
HAL_StatusTypeDef send_cm_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
HAL_StatusTypeDef send_gimbal_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2);
HAL_StatusTypeDef send_shoot_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
HAL_StatusTypeDef CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
HAL_StatusTypeDef CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
HAL_StatusTypeDef CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);

#ifdef __cplusplus
}
#endif
