#include "Driver_ModeSwitch.h"

uint8_t GlobalMode;

/**
  * @brief  ģʽ��ʼ��
  * @param  void 
  * @retval void
  */
void ModeSwitch_InitConfig(void)
{
	GlobalMode = RC_Ctrl_Mode;
}
