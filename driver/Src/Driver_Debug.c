
#include "Driver_Debug.h"

#include "BSP_UART.h"
#include "BSP_DMA.h"


#define LENGTH 100
#define true 1
#define false 0

#define UART_DEBUG huart5
extern UART_HandleTypeDef UART_DEBUG;
#define DEBUG_IRQHandler UART5_IRQHandler

//数据拆分宏定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

//串口接收缓存
uint8_t uart_rx[LENGTH];

//串口发送缓存
uint8_t uart_tx[LENGTH];

//串口DMA发送标志
int16_t uart_tx_flag = false;

//发送帧帧头
uint8_t txhead = 0;

//发送帧和校验
uint8_t txsum = 0;

//上位机通信校验标志
send_flag_t _send_flag = { false };

//下位机PID缓存
pid_debug_t _pid_debug[18] = { 0 };

//发送数据
send_data_t _send = { 
	.i16_data[0] = 1,
	.i16_data[1] = 2,
	.i16_data[2] = 3,
	.i16_data[3] = 4,

	.f_data[0] = 1.1f,
	.f_data[1] = 2.2f,
	.f_data[2] = 3.3f,
	.f_data[3] = 4.4f
};

/**
  * @brief  上位机通信初始化
  * @param  void
  * @retval void
  */
void Debug_InitConfig(void)
{
	//使能串口DMA接收并开启串口空闲中断
    UART_DMA_RX_INIT(&UART_DEBUG,uart_rx,LENGTH);

    //使能串口DMA发送并开启DMA发送中断
    UART_DMA_TX_INIT(&UART_DEBUG,10);

    //初始化发送帧帧头
    uart_tx[0] = 0xAA;
	uart_tx[1] = 0xAA;

    _send_flag.send_check = false;
    _send_flag.send_motopwm = false;
}



/*********************************串口接收中断*************************************/

void DEBUG_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&UART_DEBUG,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&UART_DEBUG);

		__HAL_DMA_DISABLE(UART_DEBUG.hdmarx);
		DMA_CLEAR_FLAG_ALL(UART_DEBUG.hdmarx);

        //记录接收到的字节数
        uint8_t Num = LENGTH - __HAL_DMA_GET_COUNTER(UART_DEBUG.hdmarx);

		do
		{
			if(uart_rx[0] != 0xAA || uart_rx[1] != 0xAF || uart_rx[3] != Num-5)
                break;
			uint8_t num = uart_rx[3] + 5;
			txsum = 0;
			for(uint8_t i = 0;i < Num - 1;i++)
                txsum += uart_rx[i];
			if(txsum != uart_rx[num-1])break;//判断sum
			txhead = uart_rx[2];//帧头
			switch(txhead)  //功能字，判断帧头
			{
				case 0x01U:
//					switch(uart_rx[4])  //校准
//					{
//						case 0x01:gimbal.fric_wheel_run=!gimbal.fric_wheel_run;break;   //ACC校准
//						case 0x02:imu.GyrCal=true;break;    //陀螺仪校准
//						case 0x04:gimbal.trigger_run=!gimbal.trigger_run;break;    //磁力计校准
//						case 0x05:gimbal.shoot_num=1;break;    //气压计校准
//						default:  imu.AccCalFlag=uart_rx[4]-0x20;imu.AccCal=true;break;  //六面校准
//					}
					break;
				case 0x02U:
					switch(uart_rx[4])  //读取
					{
//						case 0x01U://读取pid
//							f.send_pid1 = true;
//							f.send_pid2 = true;
//							f.send_pid3 = true;
//							f.send_pid4 = true;
//							f.send_pid5 = true;
//							f.send_pid6 = true;
//							break;
						default: break;
					}
					break;
				case 0x03U://读取遥控数据
//					f.send_rcdata = true;
					break;
				case 0x10U://写入第1组pid
					Write_PID(&_pid_debug[0],&_pid_debug[1],&_pid_debug[2]);
					break;
				case 0x11U://写入第2组pid
					Write_PID(&_pid_debug[3],&_pid_debug[4],&_pid_debug[5]);
					break;
				case 0x12U://写入第3组pid
					Write_PID(&_pid_debug[6],&_pid_debug[7],&_pid_debug[8]);
					break;
				case 0x13U://写入第4组pid
					Write_PID(&_pid_debug[9],&_pid_debug[10],&_pid_debug[11]);
					break;
				case 0x14U://写入第5组pid
                    Write_PID(&_pid_debug[12],&_pid_debug[13],&_pid_debug[14]);
					break;
				case 0x15U://写入第6组pid
					Write_PID(&_pid_debug[15],&_pid_debug[16],&_pid_debug[17]);
					break;
				default:break;
			}
		}while(0);

		__HAL_DMA_SET_COUNTER(UART_DEBUG.hdmarx,LENGTH);

		__HAL_DMA_ENABLE(UART_DEBUG.hdmarx);
	}
}

/*****************************串口DMA发送函数*********************************/

void DMA1_Stream7_IRQHandler(void)//uart5 tx
{
	DMA_CLEAR_FLAG_ALL(UART_DEBUG.hdmatx);
	__HAL_DMA_DISABLE(UART_DEBUG.hdmatx);
    uart_tx_flag = false;
}

/******************************上位机初始化函数*******************************/

void debug_init(void)
{
    //使能串口DMA接收并开启串口空闲中断
    UART_DMA_RX_INIT(&UART_DEBUG,uart_rx,LENGTH);

    //使能串口DMA发送并开启DMA发送中断
    UART_DMA_TX_INIT(&UART_DEBUG,10);

    //初始化发送帧帧头
    uart_tx[0] = 0xAA;
	uart_tx[1] = 0xAA;

    _send_flag.send_check = false;
    _send_flag.send_motopwm = false;
}

/******************************上位机校验函数**********************************/

int16_t Send_Check(void)
{
    if(uart_tx_flag == true)
        return false;
    uart_tx[2] = 0xEF;
	uart_tx[3] = 2;
	uart_tx[4] = txhead;
	uart_tx[5] = txsum;

    uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
        sum += uart_tx[i];
	uart_tx[6]=sum;

	Uart_Transmit_DMA(&UART_DEBUG,(uint8_t *)uart_tx, 7);
    uart_tx_flag = true;
    return true;
}

/*****************************数据发送函数**************************************/

int16_t Send_int16_data(void)
{
    if(uart_tx_flag == true)
        return false;
	uart_tx[2]=0xF1;
	uart_tx[3]=8;

	int16_t temp = _send.i16_data[0];
	uart_tx[4] =BYTE1(temp);
	uart_tx[5] =BYTE0(temp);

	temp = _send.i16_data[1];
	uart_tx[6] =BYTE1(temp);
	uart_tx[7] =BYTE0(temp);

	temp = _send.i16_data[2];
	uart_tx[8] =BYTE1(temp);
	uart_tx[9] =BYTE0(temp);

	temp = _send.i16_data[3];
	uart_tx[10]=BYTE1(temp);
	uart_tx[11]=BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0;i<12;i++)sum += uart_tx[i];
	uart_tx[12]=sum;

    Uart_Transmit_DMA(&UART_DEBUG,(uint8_t *)uart_tx, 13);
    uart_tx_flag = true;
    return true;

}

int16_t Send_float_data(void)
{
    if(uart_tx_flag == true)
        return false;
	uart_tx[2]=0xF1;
	uart_tx[3]=16;

	float temp = _send.f_data[0];
	uart_tx[4] =BYTE3(temp);
	uart_tx[5] =BYTE2(temp);
    uart_tx[6] =BYTE1(temp);
	uart_tx[7] =BYTE0(temp);

	temp = _send.f_data[1];
	uart_tx[8] =BYTE3(temp);
	uart_tx[9] =BYTE2(temp);
    uart_tx[10] =BYTE1(temp);
	uart_tx[11] =BYTE0(temp);

	temp = _send.f_data[2];
	uart_tx[12] =BYTE3(temp);
	uart_tx[13] =BYTE2(temp);
    uart_tx[14] =BYTE1(temp);
	uart_tx[15] =BYTE0(temp);

	temp = _send.f_data[3];
	uart_tx[16] =BYTE3(temp);
	uart_tx[17] =BYTE2(temp);
    uart_tx[18] =BYTE1(temp);
	uart_tx[19] =BYTE0(temp);

	uint8_t sum = 0;
	for(uint8_t i=0;i<20;i++)sum += uart_tx[i];
	uart_tx[20]=sum;

    Uart_Transmit_DMA(&UART_DEBUG,(uint8_t *)uart_tx, 21);
	uart_tx_flag = true;
    return true;
}

/******************************写入PID函数*************************************/

uint8_t debug_write = 0;

void Write_PID(pid_debug_t *pid1,pid_debug_t *pid2,pid_debug_t *pid3)
{
	pid1->p = 0.01f*( (int16_t)(*(uart_rx+4 )<<8)|*(uart_rx+5 ) );
	pid1->i = 0.01f*( (int16_t)(*(uart_rx+6 )<<8)|*(uart_rx+7 ) );
	pid1->d = 0.01f*( (int16_t)(*(uart_rx+8 )<<8)|*(uart_rx+9 ) );
	pid2->p = 0.01f*( (int16_t)(*(uart_rx+10)<<8)|*(uart_rx+11) );
	pid2->i = 0.01f*( (int16_t)(*(uart_rx+12)<<8)|*(uart_rx+13) );
	pid2->d = 0.01f*( (int16_t)(*(uart_rx+14)<<8)|*(uart_rx+15) );
	pid3->p = 0.01f*( (int16_t)(*(uart_rx+16)<<8)|*(uart_rx+17) );
	pid3->i = 0.01f*( (int16_t)(*(uart_rx+18)<<8)|*(uart_rx+19) );
	pid3->d = 0.01f*( (int16_t)(*(uart_rx+20)<<8)|*(uart_rx+21) );

	debug_write = 1;

	_send_flag.send_check = !Send_Check();
}

/******************************上位机任务状态监测函数**************************/

