
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_TIM.h"
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

		key_read();
}

/**
  * @brief  获取按键值
  * @param  void
  * @retval void
  */
static void key_read(void)
{
//    //低位建
//    _radio.key.W = ((_radio_raw.key.v_l << 7) == 0x80 ? 1 : 0);    //0x01 low
//    _radio.key.S = ((_radio_raw.key.v_l << 6) == 0x40 ? 1 : 0);    //0x02 low
//    _radio.key.A = ((_radio_raw.key.v_l << 5) == 0x20 ? 1 : 0);    //0x04 low
//    _radio.key.D = ((_radio_raw.key.v_l << 4) == 0x10 ? 1 : 0);    //0x08 low
//    _radio.key.Shift = ((_radio_raw.key.v_l << 3) == 0x08 ? 1 : 0);//0x10 low
//    _radio.key.Ctrl = ((_radio_raw.key.v_l << 2) == 0x04 ? 1 : 0); //0x20 low
//    _radio.key.Q = ((_radio_raw.key.v_l << 1) == 0x02 ? 1 : 0);    //0x40 low
//    _radio.key.E = ((_radio_raw.key.v_l << 0) == 0x01 ? 1 : 0);    //0x80 low
//    //高位键
//    _radio.key.R = ((_radio_raw.key.v_l << 7) == 0x80 ? 1 : 0);    //0x01 high
//    _radio.key.F = ((_radio_raw.key.v_l << 6) == 0x40 ? 1 : 0);    //0x02 high
//    _radio.key.G = ((_radio_raw.key.v_l << 5) == 0x20 ? 1 : 0);    //0x04 high
//    _radio.key.Z = ((_radio_raw.key.v_l << 4) == 0x10 ? 1 : 0);    //0x08 high
//    _radio.key.X = ((_radio_raw.key.v_l << 3) == 0x08 ? 1 : 0);    //0x10 high
//    _radio.key.C = ((_radio_raw.key.v_l << 2) == 0x04 ? 1 : 0);    //0x20 high
//    _radio.key.V = ((_radio_raw.key.v_l << 1) == 0x02 ? 1 : 0);    //0x40 high
//    _radio.key.B = ((_radio_raw.key.v_l << 0) == 0x01 ? 1 : 0);    //0x80 high
    
     //低位建
    _radio.key.W = (_radio_raw.key.v_l );    //0x01 low
    _radio.key.S = (_radio_raw.key.v_l >> 1);    //0x02 low
    _radio.key.A = (_radio_raw.key.v_l >> 2);    //0x04 low
    _radio.key.D = (_radio_raw.key.v_l >> 3);    //0x08 low
    _radio.key.Shift = (_radio_raw.key.v_l >> 4);//0x10 low
    _radio.key.Ctrl = (_radio_raw.key.v_l >> 5); //0x20 low
    _radio.key.Q = (_radio_raw.key.v_l >> 6);    //0x40 low
    _radio.key.E = (_radio_raw.key.v_l >> 7);    //0x80 low
    //高位键
    _radio.key.R = (_radio_raw.key.v_h );    //0x01 high
    _radio.key.F = (_radio_raw.key.v_h >> 1);    //0x02 high
    _radio.key.G = (_radio_raw.key.v_h >> 2);    //0x04 high
    _radio.key.Z = (_radio_raw.key.v_h >> 3);    //0x08 high
    _radio.key.X = (_radio_raw.key.v_h >> 4);    //0x10 high
    _radio.key.C = (_radio_raw.key.v_h >> 5);    //0x20 high
    _radio.key.V = (_radio_raw.key.v_h >> 6);    //0x40 high
    _radio.key.B = (_radio_raw.key.v_h >> 7);    //0x80 high
}

/**
  * @brief  单击
  * @param  按键
  * @retval 按下：1  未按下：2
  */
uint8_t single_press(uint8_t key)
{
    if(key == 1)
        return 1;
    else
        return 0;
}

/**
  * @brief  双击
  * @param  按键
  * @retval 按下：1  未按下：2
  */
uint8_t double_press(uint8_t key)
{
    return 0;
}

/**
  * @brief  组合按键，只包括一个高位键和一个低位键的组合，两个高位键或两个低位键的组合需自行编写。组合按键为或运算逻辑
  * @param  按键1
  * @param  按键2
  * @retval 按下：1  未按下：2
  */
uint8_t combine_press(uint8_t key1, uint8_t key2)
{
    if(_radio_raw.key.v_l != 0 && _radio_raw.key.v_h != 0) {
        if(key1&key2) {
            return 1;
        }
    }
    return 0;
}

/**
  * @brief  长按
  * @param  按键
  * @param  延时时间
  * @retval 按下：1  未按下：2
  */
uint8_t long_press(uint8_t key, uint64_t set_time)
{
    uint64_t t = time();
    //若按键按下
    while(!single_press(key)) {
        if(time() - t >= set_time)
            return 1;
    }
    return 0;
}
