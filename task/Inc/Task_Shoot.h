#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

    uint8_t shooting_off17;
	uint8_t reverse17;
	uint8_t reverse_count17;
	uint32_t reverse_tick17;
	uint32_t blocking_tick17;
	uint8_t flag17;
	
	uint8_t friction17;
	uint16_t dalay_ms17;
	uint16_t count_t17;
	uint16_t count_c17;
	uint64_t shooting_tick17;
	uint8_t arti_shoot17;
	uint8_t continued_reverse17;
	uint8_t super_shoot_count17;
	float shooting_speed17;
	uint16_t heatmax17;

    uint8_t shooting_off42;
	uint8_t reverse42;
	uint8_t reverse_count42;
	uint32_t reverse_tick42;
	uint32_t blocking_tick42;
	uint8_t flag42;
	uint8_t friction42;

	uint16_t dalay_ms42;
	uint16_t count_t42;
	uint16_t count_c42;
	uint64_t shooting_tick42;
	uint8_t arti_shoot42;
	uint8_t continued_reverse42;
	uint8_t super_shoot_count42;
	float shooting_speed42;
	uint16_t heatmax42;
    
    uint8_t steering;
    
} Shoot_t;



extern Shoot_t _shoot;
void Shooting_Judge17(void);
void Shooting_Judge42(void);
void Task_Shoot(void *Parameters);

#ifdef __cplusplus
}
#endif
