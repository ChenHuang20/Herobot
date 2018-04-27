#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    float Real_Power[3]; //2最新 1上次 0上上次
	float RemainPower[3];
	float Set_Power;
	float MaxSpeed;  //最大速度值
	uint8_t Cut;        //是否切断底盘输出
	float Zoom;    //缩放比率防止方向错误
	float ZoomTemp;//缩放暂存值
	uint8_t ZoomCount;//缩放For循环计数
	uint8_t Flag;//标志位，用来检测是否退出限速区
	uint8_t K_stall;//用来存储刚进入限速区的速度
	int16_t pidTemp;//自动降档的时候用来存储PID输出值
	float RealSpeed;
	float v;
} PowerLimit_t;

typedef struct
{
	int16_t Bias[5];//差值=期望速度-实际速度
	int16_t MaxBias;  //最大差值
	uint8_t MotorID;//最大差值电机的ID
	float Zoom[5];    //缩放比率防止方向错误
	float ZoomTemp;//缩放暂存值
	uint8_t ZoomCount;//缩放For循环计数
	float Increment[5];//电机增量
} Increment_t;

void CMControlLoop(void);
void Moving_Trial(float *MotoSpeed);
void Power_Deal(void);

void PowerLimitLoop(void);
int32_t Motor_Increment_Deal(uint8_t Motor_ID,int32_t Set_Val,float Increment);
void Motor_Increment_Self_Deal(float* MotoSpeed);
void PowerLimit_InitConfig(void);

extern PowerLimit_t PowerLimit;
extern Increment_t Increment;
extern int32_t Real_Speed[7];


#ifdef __cplusplus
}
#endif
