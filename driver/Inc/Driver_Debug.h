#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct 
{
	int16_t send_motopwm;
	int16_t send_check;
} send_flag_t;

typedef struct {
    float p;
    float i;
    float d;
}pid_debug_t;

typedef struct {
    int16_t i16_data[4];
    float f_data[4];
}send_data_t;

void Write_PID(pid_debug_t *pid1,pid_debug_t *pid2,pid_debug_t *pid3);

extern void Debug_InitConfig(void);
extern void debug_task(void);

#ifdef __cplusplus
}
#endif
