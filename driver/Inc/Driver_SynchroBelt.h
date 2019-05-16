#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t RealCurrent;
    int16_t RealVelocity;
    int32_t RealPosition;
}SynchroBeltParam_t;
typedef struct
{
    int16_t RealCurrent;
    int16_t RealVelocity;
    int32_t RealPosition;
}ArmParam_t;

extern SynchroBeltParam_t _SynchroBeltParam;
extern ArmParam_t         _ArmParam;

#ifdef __cplusplus
}
#endif
