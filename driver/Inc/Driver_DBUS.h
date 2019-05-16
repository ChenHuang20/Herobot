#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Usart_Handle huart2
#define DMA_Handle hdma_usart2_rx
#define UART2_MAX_LEN 100

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
};

typedef struct {
    uint64_t timestamp;

    struct {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        uint8_t s1;
        uint8_t s2;
    } rc;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct {
        uint8_t v_h;
        uint8_t v_l;
    } key;

	uint32_t lastUpdate;

    float quality;
} radio_raw_t;

typedef struct {
    uint64_t timestamp;

    struct {
    float x;
    float y;
    float pitch;
    float yaw;
    uint8_t mode;
    uint8_t shoot;
	uint8_t shoot_pre;
    } rc;

    struct {
        int16_t yaw;
        int16_t pitch;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct {
        uint8_t W;//0x01 low
        uint8_t S;//0x02 low
        uint8_t A;//0x04 low
        uint8_t D;//0x08 low
        uint8_t Shift;//0x10 low
        uint8_t Ctrl;//0x20 low
        uint8_t Q;//0x40 low
        uint8_t E;//0x80 low
        uint8_t R;//0x01 high
        uint8_t F;//0x02 high
        uint8_t G;//0x04 high
        uint8_t Z;//0x08 high
        uint8_t X;//0x10 high
        uint8_t C;//0x20 high
        uint8_t V;//0x40 high
        uint8_t B;//0x80 high
		
		uint8_t W_pre;//0x01 low
        uint8_t S_pre;//0x02 low
        uint8_t A_pre;//0x04 low
        uint8_t D_pre;//0x08 low
        uint8_t Shift_pre;//0x10 low
        uint8_t Ctrl_pre;//0x20 low
        uint8_t Q_pre;//0x40 low
        uint8_t E_pre;//0x80 low
        uint8_t R_pre;//0x01 high
        uint8_t F_pre;//0x02 high
        uint8_t G_pre;//0x04 high
        uint8_t Z_pre;//0x08 high
        uint8_t X_pre;//0x10 high
        uint8_t C_pre;//0x20 high
        uint8_t V_pre;//0x40 high
        uint8_t B_pre;//0x80 high
    } key;

} radio_t;

extern radio_raw_t _radio_raw;
extern radio_t _radio;

extern void DBUS_InitConfig(void);
extern void DBUS_DataProcessing(void);

static void key_read(void);
extern uint8_t single_press(uint8_t key);
extern uint8_t double_press(uint8_t key);
extern uint8_t combine_press(uint8_t key1, uint8_t key2);
extern uint8_t long_press(uint8_t key, uint64_t t);

#ifdef __cplusplus
}
#endif
