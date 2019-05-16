#include "BSP_TIM.h"
#include "Driver_Steering.h"

uint8_t Steering_tick=0;
uint8_t Steering_flag=0;


void SteeringShoot_ON(void)
{
	TIM4->CCR4 = 280;
}

void SteeringShoot_OFF(void)
{
	TIM4->CCR4 =407;
}

