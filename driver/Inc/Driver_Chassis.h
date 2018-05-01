#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


//单电机参数
typedef struct
{
    float TargetSpeed;               //电机目标速度
    int16_t RealSpeed;               //电机实际速度
    uint16_t LimitCurrent;           //限制电流
    uint16_t RealCurrent;            //实际电流
    uint16_t NeedCurrent;            //需求电流
}OneMotorParam_t;

//底盘电机参数结构体
typedef struct
{
    OneMotorParam_t Motor[4];
    float TargetVX;
    float TargetVY;
    float TargetOmega;
    float TargetABSAngle;
	float MaxWheelSpeed;
	int16_t ChassisCurrent[4];
	int16_t ChassisMaxSumCurrent;
	float kp[5];
	float ki[5];
	float kd[5];
}ChassisParam_t;


extern OneMotorParam_t _OneMotorParam;
extern ChassisParam_t _ChassisParam;

void Current_Distribution(int16_t *current, uint16_t total_current);
void Chassis_SpeedSet(float XSpeed, float YSpeed, float OmegaSpeed);
void Chassis_InitConfig(void);

#ifdef __cplusplus
}
#endif
