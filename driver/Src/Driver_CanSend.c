
#include "Driver_CanSend.h"

#include "BSP_CAN.h"

/**
  * @brief  底盘CAN发送函数
  * @param  CAN句柄
  * @param	轮子1电流
  * @param	轮子2电流
  * @param	轮子3电流
  * @param	轮子4电流
  * @retval CAN发送状态
  */
HAL_StatusTypeDef send_cm_current(CAN_HandleTypeDef *hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	HAL_StatusTypeDef kk = HAL_TIMEOUT;
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
	kk = HAL_CAN_Transmit(hcan, 1000);
	return kk;
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






//void CAN1_RX0_IRQHandler(void)
//{
//  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

//  /* USER CODE END CAN1_RX0_IRQn 0 */
//  HAL_CAN_IRQHandler(&hcan1);
//  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

//        //0x201~0x204反馈速度   0x205~0x206反馈机械角度
//        switch (hcan1.pRxMsg->StdId) {
//      case 0x201:
//                _can1.data[0] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
//                _can1.data[6] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);

////                if(_motor_trigger.msg_cnt++ <= 10)
////                {
////                _motor_trigger.angle = (uint16_t)(hcan1.pRxMsg->Data[0] << 8 | hcan1.pRxMsg->Data[1]);
////                _motor_trigger.offset_angle = _motor_trigger.angle;
////                }
////                else //+-4096
////                {
////                _motor_trigger.speed_rpm  = (int16_t)(hcan1.pRxMsg->Data[2] << 8 | hcan1.pRxMsg->Data[3]);
////                _tri_rates.trigger = _motor_trigger.speed_rpm / 60 / 96;//(rad/s)
////                _motor_trigger.given_current = (int16_t)(hcan1.pRxMsg->Data[4] << 8 | hcan1.pRxMsg->Data[5]) / -5;
////                _motor_trigger.hall = hcan1.pRxMsg->Data[6];
////                _motor_trigger.last_angle = _motor_trigger.angle;
////                _motor_trigger.angle = (uint16_t)(hcan1.pRxMsg->Data[0] << 8 | hcan1.pRxMsg->Data[1]);

////                if (_motor_trigger.angle - _motor_trigger.last_angle > 4096)
////                _motor_trigger.round_cnt--;

////                else if (_motor_trigger.angle - _motor_trigger.last_angle < -4096)
////                _motor_trigger.round_cnt++;

////                _motor_trigger.total_ecd = _motor_trigger.round_cnt * 8192 + _motor_trigger.angle - _motor_trigger.offset_angle;
////                _motor_trigger.total_angle = _motor_trigger.total_ecd * 360 / 8192;

//                

//                break;

//			case 0x202:
//                _can1.data[1] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
//                _can1.data[7] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
//                break;

//			case 0x203:
//                _can1.data[2] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
//                _can1.data[8] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
//                break;

//			case 0x204:
//                _can1.data[3] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
//                _can1.data[9] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
//                break;

//			case 0x205:
//                _can1.data[4] = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);
//                break;

//			case 0x206:
//                _can1.data[5] = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);

//			default:
//                break;
//        
//    }

//    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
//  /* USER CODE END CAN1_RX0_IRQn 1 */
//}


void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);

//摩擦轮反馈转速 拨弹电机反馈转速和机械角度
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

