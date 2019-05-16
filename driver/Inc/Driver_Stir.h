#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


	
//大拨弹电机3508参数结构体
typedef struct
{
    float TargetSpeed;             //电机目标速度
    float RealSpeed;               //电机实际速度rpm
    float CurrentSpeed;
	float PreBlockPositon;
    float PrePositon;
//	float TargetPosition;
	float RealPosition;
    float CurrentPosition;
    float LastPosition;
    uint16_t EncoderPosition;
//	float offset_angle;
	int32_t Round_cnt;
	uint32_t msg_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;
    
    uint8_t Sensor;             
	uint8_t Sensor_last;  	         
	uint8_t Bullet_count;
	uint8_t Stop_flag;        
	uint16_t Bullet_tick;         
	uint16_t Shoot_count;
	uint16_t Blocked_tick;

//	float kp_p;
//	float ki_p;
//	float kd_p;
	
	float kp_v;
	float ki_v;
	float kd_v;

}Stir42Param_t;

//小拨弹电机2310参数结构体
typedef struct
{
    float TargetSpeed;             //电机目标速度
    float RealSpeed;               //电机实际速度rpm
    float CurrentSpeed;
	float PrePositon;
	float TargetPosition;
	float RealPosition;
    float CurrentPosition;

    uint16_t EncoderPosition;
	float offset_angle;
	int32_t Round_cnt;
	uint32_t msg_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;

	int32_t Blocked_tick;			//堵转时间
	uint8_t Reverse_task;           //反转模式标志位
	uint16_t Reverse_tick;			//反转后堵转时间
	uint8_t Shoot_task;				//执行发射任务标志位
	uint8_t Shoot_num;				//连续发生子弹数
	uint16_t Shoot_count;            //记录已发射子弹数 
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
void Stir42_ON(void);
void Stir42_OFF(void);
void Stir17_ON(void);
void Stir17_OFF(void);

#ifdef __cplusplus
}
#endif
