#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_offset[3];
    float accel_scale[3];
    float gyro_offset[3];
    float gyro_scale[3];
} params_t;

typedef struct {
    uint64_t timestamp;
    float temperature;
    int16_t x_raw, y_raw, z_raw;
    float x, y, z;
} accel_t;

typedef struct {
    uint64_t timestamp;
    float temperature;
    int16_t x_raw, y_raw, z_raw;
    float x, y, z;
} gyro_t;

typedef struct {
    float _cutoff_freq;
    float _a1, _a2;
    float _b0;
    float _delay_element_1, _delay_element_2;
} low_pass_filter_t;

extern accel_t _accel;
extern gyro_t _gyro;
extern params_t _params;

extern void lpf_set_cutoff_frequency(low_pass_filter_t *p, float sample_freq, float cutoff_freq);
extern float lpf_allpy(low_pass_filter_t *p, float sample);
extern float lpf_reset(low_pass_filter_t *p, float sample);

void icm20600_task(void);

#ifdef __cplusplus
}
#endif
