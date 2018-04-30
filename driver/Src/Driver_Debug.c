
#include "Driver_Debug.h"

#include "BSP_UART.h"
#include "BSP_DMA.h"


#define LENGTH 100
#define true 1
#define false 0

#define UART_DEBUG huart5
extern UART_HandleTypeDef UART_DEBUG;
#define DEBUG_IRQHandler UART5_IRQHandler

//���ݲ�ֺ궨��
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

//���ڽ��ջ���
uint8_t uart_rx[LENGTH];

//���ڷ��ͻ���
uint8_t uart_tx[LENGTH];

//����DMA���ͱ�־
int16_t uart_tx_flag = false;

//����֡֡ͷ
uint8_t txhead = 0;

//����֡��У��
uint8_t txsum = 0;

//��λ��ͨ��У���־
send_flag_t _send_flag = { false };

//��λ��PID����
pid_debug_t _pid_debug[18] = { 0 };

//��������
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
  * @brief  ��λ��ͨ�ų�ʼ��
  * @param  void
  * @retval void
  */
void Debug_InitConfig(void)
{
	//ʹ�ܴ���DMA���ղ��������ڿ����ж�
    UART_DMA_RX_INIT(&UART_DEBUG,uart_rx,LENGTH);

    //ʹ�ܴ���DMA���Ͳ�����DMA�����ж�
    UART_DMA_TX_INIT(&UART_DEBUG,10);

    //��ʼ������֡֡ͷ
    uart_tx[0] = 0xAA;
	uart_tx[1] = 0xAA;

    _send_flag.send_check = false;
    _send_flag.send_motopwm = false;
}



/*********************************���ڽ����ж�*************************************/

void DEBUG_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&UART_DEBUG,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&UART_DEBUG);

		__HAL_DMA_DISABLE(UART_DEBUG.hdmarx);
		DMA_CLEAR_FLAG_ALL(UART_DEBUG.hdmarx);

        //��¼���յ����ֽ���
        uint8_t Num = LENGTH - __HAL_DMA_GET_COUNTER(UART_DEBUG.hdmarx);

		do
		{
			if(uart_rx[0] != 0xAA || uart_rx[1] != 0xAF || uart_rx[3] != Num-5)
                break;
			uint8_t num = uart_rx[3] + 5;
			txsum = 0;
			for(uint8_t i = 0;i < Num - 1;i++)
                txsum += uart_rx[i];
			if(txsum != uart_rx[num-1])break;//�ж�sum
			txhead = uart_rx[2];//֡ͷ
			switch(txhead)  //�����֣��ж�֡ͷ
			{
				case 0x01U:
//					switch(uart_rx[4])  //У׼
//					{
//						case 0x01:gimbal.fric_wheel_run=!gimbal.fric_wheel_run;break;   //ACCУ׼
//						case 0x02:imu.GyrCal=true;break;    //������У׼
//						case 0x04:gimbal.trigger_run=!gimbal.trigger_run;break;    //������У׼
//						case 0x05:gimbal.shoot_num=1;break;    //��ѹ��У׼
//						default:  imu.AccCalFlag=uart_rx[4]-0x20;imu.AccCal=true;break;  //����У׼
//					}
					break;
				case 0x02U:
					switch(uart_rx[4])  //��ȡ
					{
//						case 0x01U://��ȡpid
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
				case 0x03U://��ȡң������
//					f.send_rcdata = true;
					break;
				case 0x10U://д���1��pid
					Write_PID(&_pid_debug[0],&_pid_debug[1],&_pid_debug[2]);
					break;
				case 0x11U://д���2��pid
					Write_PID(&_pid_debug[3],&_pid_debug[4],&_pid_debug[5]);
					break;
				case 0x12U://д���3��pid
					Write_PID(&_pid_debug[6],&_pid_debug[7],&_pid_debug[8]);
					break;
				case 0x13U://д���4��pid
					Write_PID(&_pid_debug[9],&_pid_debug[10],&_pid_debug[11]);
					break;
				case 0x14U://д���5��pid
                    Write_PID(&_pid_debug[12],&_pid_debug[13],&_pid_debug[14]);
					break;
				case 0x15U://д���6��pid
					Write_PID(&_pid_debug[15],&_pid_debug[16],&_pid_debug[17]);
					break;
				default:break;
			}
		}while(0);

		__HAL_DMA_SET_COUNTER(UART_DEBUG.hdmarx,LENGTH);

		__HAL_DMA_ENABLE(UART_DEBUG.hdmarx);
	}
}

/*****************************����DMA���ͺ���*********************************/

void DMA1_Stream7_IRQHandler(void)//uart5 tx
{
	DMA_CLEAR_FLAG_ALL(UART_DEBUG.hdmatx);
	__HAL_DMA_DISABLE(UART_DEBUG.hdmatx);
    uart_tx_flag = false;
}

/******************************��λ����ʼ������*******************************/

void debug_init(void)
{
    //ʹ�ܴ���DMA���ղ��������ڿ����ж�
    UART_DMA_RX_INIT(&UART_DEBUG,uart_rx,LENGTH);

    //ʹ�ܴ���DMA���Ͳ�����DMA�����ж�
    UART_DMA_TX_INIT(&UART_DEBUG,10);

    //��ʼ������֡֡ͷ
    uart_tx[0] = 0xAA;
	uart_tx[1] = 0xAA;

    _send_flag.send_check = false;
    _send_flag.send_motopwm = false;
}

/******************************��λ��У�麯��**********************************/

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

/*****************************���ݷ��ͺ���**************************************/

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

/******************************д��PID����*************************************/

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

/******************************��λ������״̬��⺯��**************************/

