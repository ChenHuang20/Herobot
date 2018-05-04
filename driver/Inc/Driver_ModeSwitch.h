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

extern uint8_t GlobalMode;

void ModeSwitch_InitConfig(void);
#ifdef __cplusplus
}
#endif
