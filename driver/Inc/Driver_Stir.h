#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


	
//大拨弹电机3508参数结构体
typedef struct
{
    float TargetSpeed;               //电机目标速度
    int16_t RealSpeed;               //电机实际速度
	int16_t TargetCurrent;             //电机需求电流
	int16_t MaxStirSpeed;
	float kp;
	float ki;
	float kd;
}Stir42Param_t;

//小拨弹电机2310参数结构体
typedef struct
{
    float TargetSpeed;               //电机目标速度
    float RealSpeed;               //电机实际速度
	float PrePositon;
	float TargetPosition;
	float RealPosition;
	int32_t Round_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;
	int32_t Blocked_tick;			//堵转时间
	uint8_t Reverse_task;           //反转模式标志位
	uint16_t Reverse_tick;			//反转后堵转时间
	uint8_t Shoot_task;				//执行发射任务标志位
	uint8_t Shoot_num;				//连续发生子弹数
	uint16_t Shoot_count;            //记录已发射子弹数 （0~正无穷）
	uint16_t Shoot_lastcount;        //记录上一次子弹数
	uint16_t Shoot_time;			//连续发生间隔时间
	uint16_t Shoot_tick;			//记录发射间隔时间
	float kp_p;
	float ki_p;
	float kd_p;
	
	float kp_v;
	float ki_v;
	float kd_v;

}Stir17Param_t;

extern Stir42Param_t _Stir42Param;
extern Stir17Param_t _Stir17Param;
void Stir_InitConfig(void);

#ifdef __cplusplus
}
#endif
