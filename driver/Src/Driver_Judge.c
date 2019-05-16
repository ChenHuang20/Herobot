
#include "Handle.h"

#include "BSP_TIM.h"
#include "BSP_DMA.h"
#include "BSP_UART.h"
#include "Driver_Judge.h"
#include "Driver_CRC.h"

#include "Driver_Monitor.h"

#include <string.h>

#define buffer_size 1000
#define judgement_huart huart6
//裁判系统数据缓存
uint8_t ring_buffer[buffer_size] = { 0 };
uint8_t* write_hand = ring_buffer;
uint8_t* read_hand = ring_buffer;
uint16_t  leftbuffer_size = 0;

//DMA缓存
uint8_t judgement_uart_rx[100] = { 0 };
uint16_t num = 0;

//裁判系统串口
extern UART_HandleTypeDef judgement_huart;

FLAOT_UNION DataA;
FLAOT_UNION DataB;
FLAOT_UNION DataC;

//2018裁判系统结构体
Judge_t _Judge = { 0 };
JudgeSend_t _JudgeSend = { 0 };
uint8_t JudgeSendBuff[21];

/**
  * @brief  裁判系统初始化
  * @param  void
  * @retval void
  */
void Judge_InitConfig(void)
{
	_JudgeSend.SOF=0xA5;
	_JudgeSend.DataLength = 13;
	_JudgeSend.CmdID=0x0100;
	//使能串口DMA接收并开启串口空闲中断
	UART_DMA_RX_INIT(&judgement_huart, judgement_uart_rx, sizeof(judgement_uart_rx));
	//使能串口DMA发送并开启DMA发送中断
    UART_DMA_TX_INIT(&judgement_huart,22);
}

/*函数功能：将DMA在缓存区所存的数据按照裁判系统读取函数的要求传递给读函数。
**函数参数：所需数据大小，要存到的数组的首地址。
**函数返回值：成功返回0，失败返回-1。
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

/*函数功能：将little endian存储的uint32_t（4个uint8_t）转化为float
**函数参数：uint8_t数组的首地址
**函数返回值：float的值
*/
static float u32_to_float(uint8_t* chReceive){
float fReceive;
*((char *)(&fReceive)) = chReceive[0];
*((char *)(&fReceive) + 1) = chReceive[1];
*((char *)(&fReceive) + 2) = chReceive[2];
*((char *)(&fReceive) + 3) = chReceive[3];
return fReceive;
}

/*函数功能：（适用于2018年裁判系统）分析裁判系统收到的一帧数据，归到结构体
**函数参数：一帧数据的首地址
**函数返回值：无
*/

static void judge_data2018(uint8_t* frame)
{
    uint16_t cmdID = 256*frame[6]+frame[5];
    uint8_t* data=&frame[7];
    switch(cmdID){

		//比赛机器人状态，10Hz频率周期接收
		case 0x0001:
					_Judge.StateTick = Tick;
					_Judge.stageRemianTime = (uint16_t)(data[0]|data[1]<<8); //当前阶段剩余时间
					_Judge.gameProgress = data[2];							 //当前比赛阶段
					_Judge.robotLevel = data[3];							 //机器人当前等级
					_Judge.remainHP = (uint16_t)(data[4]|data[5]<<8);		 //机器人当前血量
					_Judge.maxHP = (uint16_t)(data[6]|data[7]<<8);   		 //机器人满血量
					break;

		//机器人伤害数据，受到伤害时接收
        case 0x0002:
					_Judge.hurt.armorType = (uint8_t)(data[0]&0x0F);		 //若变化类型为装甲伤害时标识装甲ID
					_Judge.hurt.hurtType = (uint8_t)(data[0]>>4);			 //血量变化类型，0x0装甲伤害，0x1模块掉线
					break;

		//实时射击数据，发射弹丸时接收
        case 0x0003:
					_Judge.ShootTick = Tick;
					_Judge.bulletType = data[0];							 //弹丸类型，1：17mm，2:42mm
                    if(_Judge.bulletType == 1) {
                    _Judge.bulletFreq[0] = data[1];							 //17mm弹丸射频  (发每秒)
					_Judge.bulletSpeed[0] = u32_to_float(&data[2]);			 //17mm弹丸射速  (m/s)

                    }
                    else if(_Judge.bulletType == 2) {
					_Judge.bulletFreq[1] = data[1];							 //42mm弹丸射频  (发每秒)
					_Judge.bulletSpeed[1] = u32_to_float(&data[2]);			 //42mm弹丸射速  (m/s)
                    }
					break;

		//实时功率和热量数据，50Hz频率接收
        case 0x0004:
					_Judge.PowerHeatTick = Tick;
					_Judge.chassisVolt = u32_to_float(&data[0]);			 //底盘输出电压(V)
					_Judge.chassisCurrent = u32_to_float(&data[4]);		     //底盘输出电流(A)
					_Judge.chassisPower = u32_to_float(&data[8]);			 //底盘输出功率(W)
					_Judge.chassisPowerBuffer = u32_to_float(&data[12]);	 //底盘功率缓冲(W)
					_Judge.shooterHeat0 = (uint16_t)(data[16]|data[17]<<8);  //17mm枪口热量
					_Judge.shooterHeat1 = (uint16_t)(data[18]|data[19]<<8);  //42mm枪口热量
					break;

		//场地交互数据
        case 0x0005:
					_Judge.cardType = data[0];							     //卡类型
					_Judge.cardIdx = data[1];								 //卡索引号，可用于区分不同区域
					break;

		//比赛结果
        case 0x0006:
					_Judge.winner = data[0];								 //0平局 1红方胜 2蓝方胜
					break;

		//Buff获取数据
        case 0x0007:
					_Judge.buffType = data[0];								 //Buff类型
					break;

		//机器人位置朝向信息
		case 0x0008:
					_Judge.x = u32_to_float(&data[0]);						 //位置X坐标值(m)
					_Judge.y = u32_to_float(&data[4]);						 //位置Y坐标值(m)
					_Judge.z = u32_to_float(&data[8]);						 //位置Z坐标值(m)
					_Judge.yaw = u32_to_float(&data[12]);					 //枪口朝向角度值(度)
					break;

		//参赛队自定义数据
		case 0x0100:
					_Judge.ShowData1 = u32_to_float(&data[0]);
					_Judge.ShowData2 = u32_to_float(&data[4]);
					_Judge.ShowData3 = u32_to_float(&data[8]);
					_Judge.mask = data[12];
					break;
		default:
		
				break;
    }
}

/*函数功能：当调用裁判系统读取函数时，会自动从缓存区向后读取一帧数据。
**函数参数：无。
**函数返回值：当读取到一帧正确的数据时，返回0.若读取数据是错误的，则返回-1。
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
		//判断CRC8校验位是否正确
	{
		uint16_t datalength;
		datalength=frame[1]+256*frame[2];
		if(judge_transmit(datalength+4,frame+5) == -1)
		{
			return -2;
		}
		if(Verify_CRC16_Check_Sum(frame, (datalength+9)))
		//判断CRC16校验位是否正确
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

void Judge_SendData(void)
{
	if(_JudgeSend.SendAllow==0)
	{
		_JudgeSend.SendAllow=1;
		_JudgeSend.Seq++;

		_JudgeSend.DataA=_Judge.chassisPower;
		_JudgeSend.DataB=_Judge.chassisPowerBuffer;
		_JudgeSend.DataC=_Judge.chassisVolt;	

		JudgeSendBuff[0]=_JudgeSend.SOF;
		JudgeSendBuff[1]=_JudgeSend.DataLength;
		JudgeSendBuff[2]=_JudgeSend.DataLength>>8;
		JudgeSendBuff[3]=_JudgeSend.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);
		JudgeSendBuff[5]=_JudgeSend.CmdID;
		JudgeSendBuff[6]=_JudgeSend.CmdID>>8;

		DataA.value=_JudgeSend.DataA;
		JudgeSendBuff[7]=DataA.float_byte.low_byte;
		JudgeSendBuff[8]=DataA.float_byte.mlow_byte;
		JudgeSendBuff[9]=DataA.float_byte.mhigh_byte;
		JudgeSendBuff[10]=DataA.float_byte.high_byte;

		DataB.value=_JudgeSend.DataB;
		JudgeSendBuff[11]=DataB.float_byte.low_byte;
		JudgeSendBuff[12]=DataB.float_byte.mlow_byte;
		JudgeSendBuff[13]=DataB.float_byte.mhigh_byte;
		JudgeSendBuff[14]=DataB.float_byte.high_byte;

		DataC.value=_JudgeSend.DataC;
		JudgeSendBuff[15]=DataC.float_byte.low_byte;
		JudgeSendBuff[16]=DataC.float_byte.mlow_byte;
		JudgeSendBuff[17]=DataC.float_byte.mhigh_byte;
		JudgeSendBuff[18]=DataC.float_byte.high_byte;

		JudgeSendBuff[19]=_Judge.mask;

//        if(_shooting.friction == 1)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x20;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xDF;

//        if(_chassis.mode == 1)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x10;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xEF;

//        if(_chassis.mode == 2)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x08;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xF7;

//        if(_status.chassis != 0)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x04;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xFB;

//        if(_status.gimbal != 0)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x02;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xFD;

//        if(_gimbal.auto_flag == 1)
//            JudgeSendBuff[19] = JudgeSendBuff[19]|0x01;
//        else
//            JudgeSendBuff[19] = JudgeSendBuff[19]&0xFE;

		Append_CRC16_Check_Sum(JudgeSendBuff,22);

		Uart_Transmit_DMA(&judgement_huart,(uint8_t *)JudgeSendBuff, 22);
	}
}

void DMA2_Stream1_IRQHandler(void)
{
	DMA_CLEAR_FLAG_ALL(judgement_huart.hdmatx);
	__HAL_DMA_DISABLE(judgement_huart.hdmatx);
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

		_monitor.Judgement++;
        //将DMA接收到的数据写入缓存区
		num = __HAL_DMA_GET_COUNTER(judgement_huart.hdmarx);

		if((BinSemaphoreJudge != NULL))
		{
		//读取data长度
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

		//释放二值信号量
		xSemaphoreGiveFromISR(BinSemaphoreJudge, &xHigherPriorityTaskWoken);
		//如果需要的话进行一次任务切换
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		__HAL_DMA_SET_COUNTER(judgement_huart.hdmarx,100);
		__HAL_DMA_ENABLE(judgement_huart.hdmarx);
	}
}

