#include "Driver_Stir.h"
#include "Driver_Chassis.h"
#include "Driver_CanSend.h"
#include "Driver_Friction.h"
#include "Driver_Gimbal.h"
#include "Driver_SynchroBelt.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"
/**
  * @brief  底盘CAN发送函数
  * @param  CAN句柄
  * @param	轮子1电流   3508
  * @param	轮子2电流	3508
  * @param	轮子3电流	3508
  * @param	轮子4电流	3508
  * @retval CAN发送状态
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
  * @brief  云台CAN发送函数
  * @param  CAN句柄
  * @param	pitch电流   6623
  * @param	yaw电流		6623
  * @retval CAN发送状态
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
  * @brief  发射CAN发送函数
  * @param  CAN句柄
  * @param	大摩擦轮1电流  3510
  * @param	大摩擦轮2电流  3510
  * @param	小拨弹电机电流 2310
  * @param	大拨弹电机电流 3508
  * @retval CAN发送状态
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

void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

	switch(hcan1.pRxMsg->StdId)
	{
		case 0x201:
			//左前轮
			_ChassisParam.Motor[0].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[0].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x202:
			//右前轮
			_ChassisParam.Motor[1].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[1].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x203:
			//左后轮
			_ChassisParam.Motor[2].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[2].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x204:
			//右后轮
			_ChassisParam.Motor[3].RealSpeed = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
			_ChassisParam.Motor[3].RealCurrent = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
			break;
		case 0x205:
            //Yaw
            _GimbalParam[YAW].RealEncodeAngle = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);
            break;
        case 0x206:
            //Pitch
            _GimbalParam[PIT].RealEncodeAngle = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);
            break;
		default :
			break;
	}
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

}
uint16_t io = 0;

void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
io++;
//摩擦轮反馈转速 拨弹电机反馈转速和机械角度
	switch (hcan2.pRxMsg->StdId) {
		case 0x201://大摩擦轮1速度
			_Fric42Param.RealSpeed[0] = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
			break;

		case 0x202://大摩擦轮2速度
			_Fric42Param.RealSpeed[1]  = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
			break;

		case 0x203://小拨弹机械角度和速度
			_Stir17Param.RealSpeed = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
		    _Stir17Param.CurrentSpeed = _Stir17Param.RealSpeed / 6500.0f;

			if(_Stir17Param.msg_cnt++ <= 10)
			{
			_Stir17Param.EncoderPosition = (int16_t)(hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1]);
			_Stir17Param.RealPosition = (float)_Stir17Param.EncoderPosition / 8192.0f;
			_Stir17Param.offset_angle = _Stir17Param.RealPosition;
			}

			_Stir17Param.PrePositon = _Stir17Param.RealPosition;

            _Stir17Param.EncoderPosition = (int16_t)(hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1]);
			_Stir17Param.RealPosition = (float)_Stir17Param.EncoderPosition / 8192.0f;


            if((_Stir17Param.RealPosition - _Stir17Param.PrePositon) < -0.45f)           //过零点处理
                _Stir17Param.Round_cnt++;
            else if((_Stir17Param.RealPosition - _Stir17Param.PrePositon) > 0.45f)
                _Stir17Param.Round_cnt--;

            _Stir17Param.CurrentPosition = _Stir17Param.Round_cnt + _Stir17Param.RealPosition;

            break;

		case 0x204://大拨弹
			_Stir42Param.RealSpeed = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
			_Stir42Param.CurrentSpeed = _Stir42Param.RealSpeed * RPM_TO_V / 19.0f;

            _Stir42Param.RealSpeed = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
		    _Stir42Param.CurrentSpeed = _Stir42Param.RealSpeed / 6500.0f;

			if(_Stir42Param.msg_cnt++ <= 10)
			{
			_Stir42Param.EncoderPosition = (int16_t)(hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1]);
			_Stir42Param.RealPosition = (float)_Stir42Param.EncoderPosition / 8192.0f;
//			_Stir42Param.offset_angle = _Stir42Param.RealPosition;
			}

			_Stir42Param.PrePositon = _Stir42Param.RealPosition;
            _Stir42Param.LastPosition = _Stir42Param.CurrentPosition;

            _Stir42Param.EncoderPosition = (int16_t)(hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1]);
			_Stir42Param.RealPosition = (float)_Stir42Param.EncoderPosition / 8192.0f;

            if((_Stir42Param.RealPosition - _Stir42Param.PrePositon) < -0.45f)           //过零点处理
                _Stir42Param.Round_cnt++;
            else if((_Stir42Param.RealPosition - _Stir42Param.PrePositon) > 0.45f)
                _Stir42Param.Round_cnt--;

            _Stir42Param.CurrentPosition = _Stir42Param.Round_cnt + _Stir42Param.RealPosition;

            if(_Stir42Param.CurrentPosition - _Stir42Param.LastPosition <= 0.0035f)
            {
                _Stir42Param.Blocked_tick++;
                if(_Stir42Param.Blocked_tick > 1000) { //(400ms)
                    _Stir42Param.Stop_flag = 1;
                }
            }
            else
            {
                
                _Stir42Param.Blocked_tick = 0;
                _Stir42Param.Stop_flag = 0;
            }

//            _Stir42Param.PreBlockPositon = _Stir42Param.CurrentPosition;
			break;
        case 0x01B://机械臂
            _ArmParam.RealCurrent  = (hcan2.pRxMsg->Data[0]<<8)|hcan2.pRxMsg->Data[1];
            _ArmParam.RealVelocity = (hcan2.pRxMsg->Data[2]<<8)|hcan2.pRxMsg->Data[3];
            _ArmParam.RealPosition = (hcan2.pRxMsg->Data[4]<<24)| (hcan2.pRxMsg->Data[5]<<16)| (hcan2.pRxMsg->Data[6]<<8)| hcan2.pRxMsg->Data[7];
            break;
//		case 0x02B://同步带
//            _SynchroBeltParam.RealCurrent  = (hcan2.pRxMsg->Data[0]<<8)|hcan2.pRxMsg->Data[1];
//            _SynchroBeltParam.RealVelocity = (hcan2.pRxMsg->Data[2]<<8)|hcan2.pRxMsg->Data[3];
//            _SynchroBeltParam.RealPosition = (hcan2.pRxMsg->Data[4]<<24)| (hcan2.pRxMsg->Data[5]<<16)| (hcan2.pRxMsg->Data[6]<<8)| hcan2.pRxMsg->Data[7];
//		    break;		
		default:                break;
		
        }
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}




/**********************************************蓝盒子固件CAN2驱动*****************************************************************/
/**
  * @brief  复位指令
  * @param  取值范围 0-7
  * @param	取值范围 0-15，其中Number==0时，为广播发送
  * @retval CAN发送状态
  */
HAL_StatusTypeDef CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
  * @brief  模式选择指令
  * @param  取值范围 0-7
  * @param	取值范围 0-15，其中Number==0时，为广播发送
  * @param  模式：
			OpenLoop_Mode                       0x01
			Current_Mode                        0x02
			Velocity_Mode                       0x03
			Position_Mode                       0x04
			Velocity_Position_Mode              0x05
			Current_Velocity_Mode               0x06
			Current_Position_Mode               0x07
			Current_Velocity_Position_Mode      0x08
  * @retval CAN发送状态
  */
HAL_StatusTypeDef CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
    hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

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
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    
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
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                      配置指令
Temp_Time的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Ctl1_Ctl2的取值范围：0 or 1 ，当不为0 or 1，则认为是0，为关闭左右限位检测功能
特别提示：Ctl1，Ctl2的功能仅存在于102 301，其余版本驱动器，Ctl1_Ctl2 = 0 即可
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x00A;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
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
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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
                                      在线检测
*****************************************************************************************/
HAL_StatusTypeDef CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    
    hcan2.pTxMsg->IDE     = CAN_ID_STD;    //标准帧
     hcan2.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan2.pTxMsg->DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return HAL_ERROR;
    }
    
    hcan2.pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
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

void CanSend_InitConfig(void)
{
//                                     //刚开始要有足够的延时，确保驱动器已经初始化完成
//	while (CAN_RoboModule_DRV_Reset(0,0) != HAL_OK);                     //对0组所有的驱动器进行复位
//                                      //发送复位指令后的延时必须要有，等待驱动器再次初始化完成
//    while (CAN_RoboModule_DRV_Config(0,1,100,0)!= HAL_OK);               //1号驱动器配置为100ms传回一次电流速度位置数据
//                                     //此处延时为了不让传回数据时候4个不一起传
////    while (CAN_RoboModule_DRV_Config(0,2,100,0)!= HAL_OK);                //2号驱动器配置为100ms传回一次电流速度位置数据

//    while (CAN_RoboModule_DRV_Mode_Choice(0,0,Current_Velocity_Position_Mode)!= HAL_OK);  //0组的所有驱动器 都进入电流-速度-位置模式
//    
//    usleep(500);                                      //发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
    
    usleep(500);                                        //刚开始要有足够的延时，确保驱动器已经初始化完成
	CAN_RoboModule_DRV_Reset(0,0);                     //对0组所有的驱动器进行复位
    usleep(500);                                        //发送复位指令后的延时必须要有，等待驱动器再次初始化完成
    CAN_RoboModule_DRV_Config(0,1,100,0);               //1号驱动器配置为100ms传回一次电流速度位置数据
    usleep(200);                                        //此处延时为了不让传回数据时候4个不一起传
    CAN_RoboModule_DRV_Config(0,2,100,0);                //2号驱动器配置为100ms传回一次电流速度位置数据

//   CAN_RoboModule_DRV_Mode_Choice(0,0,Current_Velocity_Position_Mode);  //0组的所有驱动器 都进入电流-速度-位置模式
   CAN_RoboModule_DRV_Mode_Choice(0,0,Position_Mode);  //0组的所有驱动器 都进入电流-速度-位置模式
    
    usleep(500);                                      //发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
}

