#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YAW 0 
#define PIT 1

#define EULER 0
#define RATES 1
#define ANGLE 2
#define SPEED 3

//单电机参数
typedef struct
{
    uint16_t RealEncodeAngle;       //实际编码器反馈机械角度
    uint16_t MiddleEncoderAngle;     //中间位置的机械角度
    uint16_t FrameCounter;          //帧率计数器
    uint16_t FrameRate;             //帧率
    int16_t SendCurrent;           //CAN发送报文中的电流值
	float kp[4];     
	float ki[4];
	float kd[4];
}GimbalParam_t;

extern GimbalParam_t _GimbalParam[2];

void Gimbal_InitConfig(void);



typedef struct {
    uint64_t timestamp;
    float _euler_sp[3];
    float _rates_sp[3];
    float _velocity_sp[4];
    float _rotate_sp;
    float _tri_angle_sp;
    float _tri_rates_sp;
    float _gimbalfollow_angle_sp;
    float _gimbalfollow_rate_sp;

    float _euler_prev[3];
    float _rates_prev[3];
    float _velocity_prev[3];
    float _rotate_prev;
    float _tri_angle_prev;
    float _tri_rates_prev;
    float _gimbalfollow_prev;

    float _rates_int[3];
    float _euler_int[3];
    float _velocity_int[3];
    float _rotate_int;
    float _tri_rates_int;
    float _tri_angle_int;
    float _gimbalfollow_int;
    
    float err_wheel_now[4];
    float err_wheel_prev[4];
    float err_wheel_pprev[4];
    float err_gimbal_now[4];
    float err_gimbal_prev[4];
    float err_gimbal_pprev[4];
    float err_rotate_now;
    float err_rotate_prev;
    float err_rotate_pprev;
    float err_tri_rates_now;
    float err_tri_tates_prev;
    float err_tri_rates_pprev;
    float err_gimbalfollow_now;
    float err_gimbalfollow_prev;
    float err_gimbalfollow_pprev;

    float delta_wheel[4];
    float output_wheel[4];
    float output_gimbal[3];
    float output_rotate;
    float output_trigger;
    float output_tri_angle;
    float output_gimbalfollow;
} pid_t;

extern pid_t _pid;

#ifdef __cplusplus
}
#endif
