#include "Driver_ModeSwitch.h"

uint8_t GlobalMode;

/**
  * @brief  模式初始化
  * @param  void 
  * @retval void
  */
void ModeSwitch_InitConfig(void)
{
	GlobalMode = RC_Ctrl_Mode;
}
