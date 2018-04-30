
#include "Driver_CanSend.h"

#include "BSP_CAN.h"

/**
  * @brief  ����CAN���ͺ���
  * @param  CAN���
  * @param	����1����   3508
  * @param	����2����	3508
  * @param	����3����	3508
  * @param	����4����	3508
  * @retval CAN����״̬
  */
HAL_StatusTypeDef send_cm_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	hcan->pTxMsg->StdId   = 0x200;
	hcan->pTxMsg->IDE     = CAN_ID_STD;
	hcan->pTxMsg->RTR     = CAN_RTR_DATA;
	hcan->pTxMsg->DLC     = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;
	return HAL_CAN_Transmit(hcan, 1000);
}

/**
  * @brief  ��̨CAN���ͺ���
  * @param  CAN���
  * @param	pitch����   6623
  * @param	yaw����		6623
  * @retval CAN����״̬
  */
HAL_StatusTypeDef send_gimbal_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2)
{
	hcan->pTxMsg->StdId   = 0x1ff;
	hcan->pTxMsg->IDE     = CAN_ID_STD;
	hcan->pTxMsg->RTR     = CAN_RTR_DATA;
	hcan->pTxMsg->DLC     = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = 0;
	hcan->pTxMsg->Data[5] = 0;
	hcan->pTxMsg->Data[6] = 0;
	hcan->pTxMsg->Data[7] = 0;
	return HAL_CAN_Transmit(hcan, 1000);
}

/**
  * @brief  ����CAN���ͺ���
  * @param  CAN���
  * @param	��Ħ����1����  3510
  * @param	��Ħ����2����  3510
  * @param	С����������� 2310
  * @param	�󲦵�������� 3508
  * @retval CAN����״̬
  */
HAL_StatusTypeDef send_shoot_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	hcan->pTxMsg->StdId   = 0x200;
	hcan->pTxMsg->IDE     = CAN_ID_STD;
	hcan->pTxMsg->RTR     = CAN_RTR_DATA;
	hcan->pTxMsg->DLC     = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;
	return HAL_CAN_Transmit(hcan, 1000);
}

enum state_e {
    STATE_NULL = 0,
    STATE_UPDATED,
    STATE_BUSY
};

struct {
    int16_t data[10];
    enum state_e state;
} _can1 = {
    .state = STATE_NULL
};

struct {
    int16_t data[4];
    enum state_e state;
} _can2 = {
    .state = STATE_NULL
};


void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);

//Ħ���ַ���ת�� �����������ת�ٺͻ�е�Ƕ�
        switch (hcan2.pRxMsg->StdId) {
					case 0x201://FICMOTOR_LEFT
										_can2.data[0] = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
										break;

					case 0x202://FICMOTOR_RIGHT
										_can2.data[1] = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
										break;

					case 0x203://STIRMOTOR
										_can2.data[2] = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
					                    _can2.data[3] = (int16_t)(hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1]);
                                        break;
			    default:                break;
        }
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}











/**********************************************�����ӹ̼�CAN2����*****************************************************************/
/**
  * @brief  ��λָ��
  * @param  ȡֵ��Χ 0-7
  * @param	ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
  * @retval CAN����״̬
  */
HAL_StatusTypeDef CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan2.pTxMsg->Data[0] = 0x55;
    hcan2.pTxMsg->Data[1] = 0x55;
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    

    return HAL_CAN_Transmit(&hcan2, 1000);
}

/**
  * @brief  ģʽѡ��ָ��
  * @param  ȡֵ��Χ 0-7
  * @param	ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
  * @param  ģʽ��
			OpenLoop_Mode                       0x01
			Current_Mode                        0x02
			Velocity_Mode                       0x03
			Position_Mode                       0x04
			Velocity_Position_Mode              0x05
			Current_Velocity_Mode               0x06
			Current_Position_Mode               0x07
			Current_Velocity_Position_Mode      0x08
  * @retval CAN����״̬
  */
HAL_StatusTypeDef CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan2.pTxMsg->Data[0] = Mode;
    hcan2.pTxMsg->Data[1] = 0x55;
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan2.pTxMsg->Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan2.pTxMsg->Data[3] = (unsigned char)(Temp_Current&0xff);
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                   �ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
    hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan2.pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan2.pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                   λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan2.pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan2.pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan2.pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                  �ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan2.pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan2.pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan2.pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan2.pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan2.pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan2.pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}


/****************************************************************************************
                                  �����ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan2.pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan2.pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}


/****************************************************************************************
                                  ����λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan2.pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan2.pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan2.pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}


/****************************************************************************************
                                  �����ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    hcan2.pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan2.pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan2.pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan2.pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan2.pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan2.pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan2.pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan2.pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                      ����ָ��
Temp_Time��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Ctl1_Ctl2��ȡֵ��Χ��0 or 1 ������Ϊ0 or 1������Ϊ��0��Ϊ�ر�������λ��⹦��
�ر���ʾ��Ctl1��Ctl2�Ĺ��ܽ�������102 301������汾��������Ctl1_Ctl2 = 0 ����
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x00A;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    if((Ctl1_Ctl2 != 0x00)&&(Ctl1_Ctl2 != 0x01))
    {
        Ctl1_Ctl2 = 0x00;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan2.pTxMsg->Data[0] = Temp_Time;
    hcan2.pTxMsg->Data[1] = Ctl1_Ctl2;
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

/****************************************************************************************
                                      ���߼��
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //��׼֡
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan2.pTxMsg->DLC = 0x08;          //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan2.pTxMsg->Data[0] = 0x55;
    hcan2.pTxMsg->Data[1] = 0x55;
    hcan2.pTxMsg->Data[2] = 0x55;
    hcan2.pTxMsg->Data[3] = 0x55;
    hcan2.pTxMsg->Data[4] = 0x55;
    hcan2.pTxMsg->Data[5] = 0x55;
    hcan2.pTxMsg->Data[6] = 0x55;
    hcan2.pTxMsg->Data[7] = 0x55;
    
    return HAL_CAN_Transmit(&hcan2, 1000);
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//���������ݵĺ�����Ĭ��Ϊ4����������������0�飬���Ϊ1��2��3��4
/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
//void CAN1_RX0_IRQHandler(void)
//{
//    CanRxMsg rx_message;
//    
//    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
//	{
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//        
//        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
//        {
//            if(rx_message.StdId == 0x1B)
//            {
//                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x2B)
//            {
//                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x3B)
//            {
//                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x4B)
//            {
//                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
//            else if(rx_message.StdId == 0x1C)
//            {
//                Real_Ctl1_Value[0] = rx_message.Data[0];
//                Real_Ctl2_Value[0] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x2C)
//            {
//                Real_Ctl1_Value[1] = rx_message.Data[0];
//                Real_Ctl2_Value[1] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x3C)
//            {
//                Real_Ctl1_Value[2] = rx_message.Data[0];
//                Real_Ctl2_Value[2] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x4C)
//            {
//                Real_Ctl1_Value[3] = rx_message.Data[0];
//                Real_Ctl2_Value[3] = rx_message.Data[1];
//            }

//        }
//                
//    }
//}
void CanSend_InitConfig(void)
{
	//group: 0   number: 10
	CAN_RoboModule_DRV_Reset(0,10);
	//group: 0   number: 10   mode: Velocity_Mode
	CAN_RoboModule_DRV_Mode_Choice(0,10,0x03);
}

