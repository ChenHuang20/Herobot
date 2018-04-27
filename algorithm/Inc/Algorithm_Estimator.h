#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t timestamp;
    float euler[3];
    float rates[3];
} attitude_t;

extern attitude_t _attitude;
extern void estimator_task(void);

#ifdef __cplusplus
}
#endif
