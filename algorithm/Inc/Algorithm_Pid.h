#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    LLAST = 0,
    LAST  = 1,
    NOW   = 2,
    POSITION_PID,
    DELTA_PID,
};

typedef struct {

	float kp;
	float ki;
	float kd;

	float err[3];
	float output;

	float maxout;
	float intergral;
	float IntegralLimit;
	float intergral_band;
	float dead_band;

} Pid_TypeDef;

extern void PID_Init(  Pid_TypeDef * pid,
				float kp,
				float ki,
				float kd,
				float max_out,
				float dead_band,
				float i_band,
				float i_max_out);

extern void PID_Calc(  Pid_TypeDef * pid,
				float get,
				float set,
				uint8_t PID_mode);

extern Pid_TypeDef CM_speed_pid[4];
extern Pid_TypeDef CM_rotate_pid;
extern Pid_TypeDef Yaw_speed_pid;
extern Pid_TypeDef Yaw_position_pid;
extern Pid_TypeDef Pitch_speed_pid;
extern Pid_TypeDef Pitch_position_pid;
extern Pid_TypeDef Stir42_speed_pid;           
extern Pid_TypeDef Stir17_speed_pid;  
extern Pid_TypeDef Stir17_position_pid;
extern Pid_TypeDef Fric42_speed_pid[2] ;    
extern Pid_TypeDef Fric17_speed_pid[2] ;
extern Pid_TypeDef Power_Limit_pid;
extern Pid_TypeDef Yaw_speed_encode_pid;
extern Pid_TypeDef Yaw_position_encode_pid;
extern Pid_TypeDef Pitch_speed_encode_pid;
extern Pid_TypeDef Pitch_position_encode_pid;
extern Pid_TypeDef Stir42_position_pid;

#ifdef __cplusplus
}
#endif
