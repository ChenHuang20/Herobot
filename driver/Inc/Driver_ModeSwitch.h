#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    RC_Ctrl_Mode,
	PC_Ctrl_Mode,
	ShutDown
};

typedef struct {

    struct {
        int8_t status;
        int8_t flag;
    } Freeview;
    
    struct {
        int8_t status;
        int8_t flag;
    } Shoot;
    
    struct {
        int8_t status;
        int8_t flag;
    } TwistWaist;
        
    struct {
        int8_t status;
        int8_t flag;
    } WatchBack;
    
    struct {
        int8_t Gear;
        int8_t Rising_flag;
        int8_t Falling_flag;
    } Speed;

} mode_t; 

extern mode_t mode;
extern uint8_t GlobalMode;

void ModeSwitch_InitConfig(void);
#ifdef __cplusplus
}
#endif
