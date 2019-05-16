#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t Steering_tick;
extern uint8_t Steering_flag;

void SteeringShoot_ON(void);
void SteeringShoot_OFF(void);

#ifdef __cplusplus
}
#endif
