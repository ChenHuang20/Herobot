#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//大摩擦轮（3510）电机参数结构体
typedef struct
{
    float TargetSpeed[2];               //电机目标速度
    int16_t RealSpeed[2];               //电机实际速度
	int16_t TargetCurrent[2];             //电机需求电路
	int16_t MaxStirSpeed[2];
	float kp[2];
	float ki[2];
	float kd[2];
	uint8_t mode;
}Fric42Param_t;

//typedef struct
//{
//	uint8_t state;
//	uint8_t mode;
//}Fric17Param_t;

extern Fric42Param_t _Fric42Param;
//extern Fric17Param_t _Fric17Param;

void Fric_InitConfig(void);

#ifdef __cplusplus
}
#endif
