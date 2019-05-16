
#include "Driver_Laser.h"

/**
  * @brief  º§π‚≥ı ºªØ
  * @param  void
  * @retval void
  */
void Laser_InitConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	LASER_RCC_CLK_ENABLE();

	GPIO_InitStruct.Pin = LASER_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LASER_GPIO, &GPIO_InitStruct);

	LASER_OFF;
}
