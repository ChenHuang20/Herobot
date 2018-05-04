
#include "Handle.h"

#include "BSP_TIM.h"
#include "BSP_DMA.h"
#include "BSP_UART.h"
#include "Driver_Judge.h"
#include "Driver_CRC.h"
#include <string.h>

#define buffer_size 1000
#define judgement_huart huart6
//����ϵͳ���ݻ���
uint8_t ring_buffer[buffer_size] = { 0 };
uint8_t* write_hand = ring_buffer;
uint8_t* read_hand = ring_buffer;
uint16_t  leftbuffer_size = 0;

//DMA����
uint8_t judgement_uart_rx[100] = { 0 };
uint16_t num = 0;

//����ϵͳ����
extern UART_HandleTypeDef judgement_huart;

//2018����ϵͳ�ṹ��
Judge_t _Judge = { 0 };

/**
  * @brief  ����ϵͳ��ʼ��
  * @param  void
  * @retval void
  */
void Judge_InitConfig(void)
{
	UART_DMA_RX_INIT(&judgement_huart, judgement_uart_rx, sizeof(judgement_uart_rx));
}

/*�������ܣ���DMA�ڻ�������������ݰ��ղ���ϵͳ��ȡ������Ҫ�󴫵ݸ���������
**�����������������ݴ�С��Ҫ�浽��������׵�ַ��
**��������ֵ���ɹ�����0��ʧ�ܷ���-1��
*/
static int8_t judge_transmit(uint16_t read_size, uint8_t* plate)
{
	if (leftbuffer_size < read_size)
		return -1;
	else
	{
		if ((read_hand+read_size) < (ring_buffer+buffer_size))
		{
			memcpy(plate, read_hand, read_size);
			read_hand = read_hand+read_size;
		}
		else if((read_hand+read_size) == (ring_buffer+buffer_size))
		{
			memcpy(plate, read_hand, read_size);
			read_hand = ring_buffer;
		}
		else
		{
			uint8_t left_size;
			left_size = ring_buffer+buffer_size-read_hand;
			memcpy(plate, read_hand, left_size);
			read_hand = ring_buffer;
			memcpy(plate+left_size, read_hand, read_size-left_size);
			read_hand = ring_buffer+read_size-left_size;
		}
		leftbuffer_size = leftbuffer_size-read_size;
		return 0;
	}
}

/*�������ܣ���little endian�洢��uint32_t��4��uint8_t��ת��Ϊfloat
**����������uint8_t������׵�ַ
**��������ֵ��float��ֵ
*/
static float u32_to_float(uint8_t* chReceive){
float fReceive;
*((char *)(&fReceive)) = chReceive[0];
*((char *)(&fReceive) + 1) = chReceive[1];
*((char *)(&fReceive) + 2) = chReceive[2];
*((char *)(&fReceive) + 3) = chReceive[3];
return fReceive;
}

/*�������ܣ���������2018�����ϵͳ����������ϵͳ�յ���һ֡���ݣ��鵽�ṹ��
**����������һ֡���ݵ��׵�ַ
**��������ֵ����
*/

static void judge_data2018(uint8_t* frame)
{
    uint16_t cmdID = 256*frame[6]+frame[5];
    uint8_t* data=&frame[7];
    switch(cmdID){

		//����������״̬��10HzƵ�����ڽ���
		case 0x0001:
					_Judge.StateTick = Tick;
					_Judge.stageRemianTime = (uint16_t)(data[0]|data[1]<<8); //��ǰ�׶�ʣ��ʱ��
					_Judge.gameProgress = data[2];							 //��ǰ�����׶�
					_Judge.robotLevel = data[3];							 //�����˵�ǰ�ȼ�
					_Judge.remainHP = (uint16_t)(data[4]|data[5]<<8);		 //�����˵�ǰѪ��
					_Judge.maxHP = (uint16_t)(data[6]|data[7]<<8);   		 //��������Ѫ��
					break;

		//�������˺����ݣ��ܵ��˺�ʱ����
        case 0x0002:
					_Judge.hurt.armorType = (uint8_t)(data[0]&0x0F);		 //���仯����Ϊװ���˺�ʱ��ʶװ��ID
					_Judge.hurt.hurtType = (uint8_t)(data[0]>>4);			 //Ѫ���仯���ͣ�0x0װ���˺���0x1ģ�����
					break;

		//ʵʱ������ݣ����䵯��ʱ����
        case 0x0003:
					_Judge.ShootTick = Tick;
					_Judge.bulletType = data[0];							 //�������ͣ�1��17mm��2:42mm
					_Judge.bulletFreq = data[1];							 //������Ƶ  (��ÿ��)
					_Judge.bulletSpeed = u32_to_float(&data[2]);			 //��������  (m/s)
					break;

		//ʵʱ���ʺ��������ݣ�50HzƵ�ʽ���
        case 0x0004:
					_Judge.PowerHeatTick = Tick;
					_Judge.chassisVolt = u32_to_float(&data[0]);			 //���������ѹ(V)
					_Judge.chassisCurrent = u32_to_float(&data[4]);		     //�����������(A)
					_Judge.chassisPower = u32_to_float(&data[8]);			 //�����������(W)
					_Judge.chassisPowerBuffer = u32_to_float(&data[12]);	 //���̹��ʻ���(W)
					_Judge.shooterHeat0 = (uint16_t)(data[16]|data[17]<<8);  //17mmǹ������
					_Judge.shooterHeat1 = (uint16_t)(data[18]|data[19]<<8);  //42mmǹ������
					break;

		//���ؽ�������
        case 0x0005:
					_Judge.cardType = data[0];							     //������
					_Judge.cardIdx = data[1];								 //�������ţ����������ֲ�ͬ����
					break;

		//�������
        case 0x0006:
					_Judge.winner = data[0];								 //0ƽ�� 1�췽ʤ 2����ʤ
					break;

		//Buff��ȡ����
        case 0x0007:
					_Judge.buffType = data[0];								 //Buff����
					_Judge.buffAddition = data[1];							 //�ӳɰٷֱ�(����10����ӳ�10%)
					break;

		//������λ�ó�����Ϣ
		case 0x0008:
					_Judge.x = u32_to_float(&data[0]);						 //λ��X����ֵ(m)
					_Judge.y = u32_to_float(&data[4]);						 //λ��Y����ֵ(m)
					_Judge.z = u32_to_float(&data[8]);						 //λ��Z����ֵ(m)
					_Judge.yaw = u32_to_float(&data[12]);					 //ǹ�ڳ���Ƕ�ֵ(��)
					break;

		//�������Զ�������
		case 0x0100:
					
					break;
		default:
		
				break;
    }
}

/*�������ܣ������ò���ϵͳ��ȡ����ʱ�����Զ��ӻ���������ȡһ֡���ݡ�
**�����������ޡ�
**��������ֵ������ȡ��һ֡��ȷ������ʱ������0.����ȡ�����Ǵ���ģ��򷵻�-1��
*/
static int i = 0;
uint8_t seq[100] = { 0 };
uint8_t frame[100];
int8_t judge_read(void)
{
	uint8_t* plate=frame;
  if(judge_transmit(5,plate) == -1)
	{
		return -2;
	}
	while(frame[0]!=0xA5)
	{
		uint8_t* bu;
		memcpy(frame,frame+1,4);
		bu = frame+4;
		if(judge_transmit(1,bu) == -1)
			return -2;
	}
	if(Verify_CRC8_Check_Sum(frame, 5))
		//�ж�CRC8У��λ�Ƿ���ȷ
	{
		uint16_t datalength;
		datalength=frame[1]+256*frame[2];
		if(judge_transmit(datalength+4,frame+5) == -1)
		{
			return -2;
		}
		if(Verify_CRC16_Check_Sum(frame, (datalength+9)))
		//�ж�CRC16У��λ�Ƿ���ȷ
		{
			judge_data2018(frame);
			seq[i++] = frame[3];
			if(i == 100)
				i = 0;
			return 0;
		}
		else
			return -1;
	}
	else return -1;
}
uint16_t datalength_;
void USART6_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;

	if((__HAL_UART_GET_FLAG(&judgement_huart,UART_FLAG_IDLE)!=RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&judgement_huart);

		__HAL_DMA_DISABLE(judgement_huart.hdmarx);
		DMA_CLEAR_FLAG_ALL(judgement_huart.hdmarx);

        //��DMA���յ�������д�뻺����
		num = __HAL_DMA_GET_COUNTER(judgement_huart.hdmarx);

		if((BinSemaphoreJudge != NULL))
		{
		//��ȡdata����
		datalength_ = (uint16_t)(frame[1] | frame[2]<<8);
		if((write_hand+100-num) < (ring_buffer+buffer_size))
		{
			memcpy(write_hand,judgement_uart_rx,100-num);
			write_hand=write_hand+100-num;
		}
		else if((write_hand+100-num) == (ring_buffer+buffer_size))
		{
			memcpy(write_hand,judgement_uart_rx,100-num);
			write_hand = ring_buffer;
		}
		else
		{
			uint8_t left_size;
			left_size = ring_buffer+buffer_size-write_hand;
			memcpy(write_hand, judgement_uart_rx, left_size);
			write_hand = ring_buffer;
			memcpy(write_hand, judgement_uart_rx+left_size, 100-num-left_size);
			write_hand = ring_buffer+100-num-left_size;
		}
		leftbuffer_size=leftbuffer_size+100-num;

		//�ͷŶ�ֵ�ź���
		xSemaphoreGiveFromISR(BinSemaphoreJudge, &xHigherPriorityTaskWoken);
		//�����Ҫ�Ļ�����һ�������л�
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		__HAL_DMA_SET_COUNTER(judgement_huart.hdmarx,100);
		__HAL_DMA_ENABLE(judgement_huart.hdmarx);
	}
}

