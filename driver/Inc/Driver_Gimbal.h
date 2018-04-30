#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//云台电机参数结构体
typedef struct
{
    uint16_t RealEncoderAngle; //实际角度编码器值
	float RealSpeed;         //实际速度
    float RealABSAngle;        //实际角度（绝对值）
    float TargetAngle;         //目标角度
	float TargetRates;		   //目标角速度
	float TargetCurrent;	   //目标电流
    uint16_t FrameCounter;     //帧率计数器
    uint16_t FrameRate;        //帧率

	float kp[2];
	float ki[2];
	float kd[2];
}GimbalParam_t;

extern GimbalParam_t PitchParam;
extern GimbalParam_t YawParam;

void Gimbal_InitConfig(void);

#ifdef __cplusplus
}
#endif
