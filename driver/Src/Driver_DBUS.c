
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "Driver_DBUS.h"

radio_raw_t _radio_raw = { 0 };
radio_t _radio = { 0 };
uint8_t uart2_rx[UART2_MAX_LEN];

/**
  * @brief  DBUS初始化
  * @param  void
  * @retval void
  */
void DBUS_InitConfig(void)
{
	UART_DMA_RX_INIT(&Usart_Handle,uart2_rx,UART2_MAX_LEN);
}

/**
  * @brief  DBUS数据解码
  * @param  void
  * @retval void
  */
void DBUS_DataProcessing(void)
{
		uint8_t* buf = uart2_rx;
		if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0 && buf[4] == 0 && buf[5] == 0)
			return;

//		_radio_raw.timestamp = time();

		_radio_raw.rc.ch0 = ((buf[0] | (int16_t)buf[1] << 8) & 0x07FF) - 1024;
		_radio_raw.rc.ch1 = ((buf[1] >> 3 | (int16_t)buf[2] << 5) & 0x07FF) - 1024;
		_radio_raw.rc.ch2 = ((buf[2] >> 6 | (int16_t)buf[3] << 2 | (int16_t)buf[4] << 10) & 0x07FF) - 1024;
		_radio_raw.rc.ch3 = ((buf[4] >> 1 | (int16_t)buf[5] << 7) & 0x07FF) - 1024;
		if(_radio_raw.rc.ch0 <= 7 && _radio_raw.rc.ch0 >= -7)
			_radio_raw.rc.ch0 = 0;
		if(_radio_raw.rc.ch1 <= 7 && _radio_raw.rc.ch1 >= -7)
			_radio_raw.rc.ch1 = 0;
		if(_radio_raw.rc.ch2 <= 7 && _radio_raw.rc.ch2 >= -7)
			_radio_raw.rc.ch2 = 0;
		if(_radio_raw.rc.ch3 <= 7 && _radio_raw.rc.ch3 >= -7)
			_radio_raw.rc.ch3 = 0;
		_radio_raw.rc.s1 = ((buf[5] >> 4) & 0x000C) >> 2;
		_radio_raw.rc.s2 = ((buf[5] >> 4) & 0x0003);

		_radio_raw.mouse.x = buf[6] | ((int16_t)buf[7] << 8);
		_radio_raw.mouse.y = buf[8] | ((int16_t)buf[9] << 8);
		_radio_raw.mouse.z = buf[10] | ((int16_t)buf[11] << 8);

		_radio_raw.mouse.press_l = buf[12];
		_radio_raw.mouse.press_r = buf[13];

		_radio_raw.key.v_l = buf[14];
		_radio_raw.key.v_h = buf[15];

//		_radio.timestamp = time();

		_radio.rc.y = _radio_raw.rc.ch1 / 660.0f;
		_radio.rc.x = _radio_raw.rc.ch0 / 660.0f;
		_radio.rc.pitch = -_radio_raw.rc.ch3 / 660.0f;
		_radio.rc.yaw = _radio_raw.rc.ch2 / 660.0f;
		_radio.rc.mode = _radio_raw.rc.s2;
		_radio.rc.shoot = _radio_raw.rc.s1;

		_radio.mouse.yaw = _radio_raw.mouse.x * 10;
		_radio.mouse.pitch= _radio_raw.mouse.y * 10;
		_radio.mouse.z = _radio_raw.mouse.z * 10;
		_radio.mouse.press_l = _radio_raw.mouse.press_l;
		_radio.mouse.press_r = _radio_raw.mouse.press_r;
}
