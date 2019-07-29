
#include "Handle.h"

#include "BSP_CAN.h"

#include "Driver_CanSend.h"
#include "Driver_Chassis.h"
#include "Driver_Gimbal.h"
#include "Driver_Stir.h"
#include "Driver_Friction.h"

#include "Task_CanSend.h"

#include "Algorithm_Pid.h"

/**
  * @brief  CAN1�������񣨵���3508��4 + ��̨6623��2��
  * @param  unused
  * @retval void
  */
void Task_Can1Send(void *Parameters)
{
	BaseType_t SemaphoreChassis = pdFALSE;
	BaseType_t SemaphoreGimbal = pdFALSE;
	_StackSurplus.Can1Send = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		HAL_StatusTypeDef EventCanChassis = HAL_TIMEOUT;
		HAL_StatusTypeDef EventCanGimbal = HAL_TIMEOUT;
		_Tick.Can1Send++;
taskENTER_CRITICAL();
		//��ȡ���������ź���
		if(BinSemaphoreChassis!=NULL)
		{
			SemaphoreChassis = xSemaphoreTake(BinSemaphoreChassis,2);//�ȴ�2ms��ʱʱ��

			if(SemaphoreChassis == pdTRUE)
			{
				EventCanChassis = send_cm_current(&hcan1,						   //CAN���
												  _ChassisParam.ChassisCurrent[0], //����1����  3508
												  _ChassisParam.ChassisCurrent[1], //����2����	3508
												  _ChassisParam.ChassisCurrent[2], //����3����	3508
												  _ChassisParam.ChassisCurrent[3]);//����4����	3508
			}
		}
		//���ü�ص���CAN�¼�λBIT0
		if(EventGroupHandler!=NULL)
		{
			if(EventCanChassis == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_0);
			}
		}
		//��ȡ��̨�����ź���
		if(BinSemaphoreGimbal!=NULL)
		{
			SemaphoreGimbal = xSemaphoreTake(BinSemaphoreGimbal,2);//�ȴ�2ms��ʱʱ��

			if(SemaphoreGimbal == pdTRUE)
			{
				EventCanGimbal = send_gimbal_current(&hcan1,
//				//CAN���
				                                     _GimbalParam[YAW].SendCurrent, //yaw����	 6623
													 _GimbalParam[PIT].SendCurrent);//pitch����   6623
				EventCanGimbal = send_gimbal_current(&hcan1,
				//CAN���
				                                     (int16_t)(_pid.output_gimbal[2] * 2000.0f), //yaw����	 6623
													 (int16_t)(_pid.output_gimbal[1] * 2000.0f));//pitch����   6623
			}
		}
		//���ü����̨CAN�¼�λBIT1
		if(EventGroupHandler!=NULL)
		{
			if(EventCanGimbal == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_1);
			}
		}
		//��ȡʣ��ջ��С
		_StackSurplus.Can1Send = uxTaskGetStackHighWaterMark(NULL);
taskEXIT_CRITICAL();
	vTaskDelay(4);
    }
}


/**
  * @brief  CAN2�������񣨴󲦵����3508��1��С�������2310��1  �����ӡ�2  ��СĦ����3510��2��
  * @param  unused
  * @retval void
  */
void Task_Can2Send(void *Parameters)
{
	BaseType_t SemaphoreShoot = pdFALSE;
	BaseType_t SemaphoreTakeBullet = pdFALSE;
	_StackSurplus.Can2Send = uxTaskGetStackHighWaterMark(NULL);

    while(1)
    {
		HAL_StatusTypeDef EventCanShoot = HAL_TIMEOUT;
		HAL_StatusTypeDef EventCanTakeBullet = HAL_TIMEOUT;
		_Tick.Can2Send++;
		//��ȡ��������ź���
		if(BinSemaphoreShoot!=NULL)
		{
			SemaphoreShoot = xSemaphoreTake(BinSemaphoreShoot,2);////�ȴ�2ms��ʱʱ��

			if(SemaphoreShoot == pdTRUE)
			{
				EventCanShoot = send_shoot_current(&hcan2,//CAN���
													_Fric42Param.TargetCurrent[0],   //��Ħ����1����  3510
													_Fric42Param.TargetCurrent[1],   //��Ħ����2����  3510
													_Stir17Param.TargetCurrent,    //С����������� 2310
													_Stir42Param.TargetCurrent);   //�󲦵�������� 3508
			}
		}
		//���ü�����CAN�¼�λBIT2
		if(EventGroupHandler!=NULL)
		{
			if(EventCanShoot == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_2);
			}
		}
		//��ȡȡ�������ź���
		if(BinSemaphoreTakeBullet!=NULL)
		{
			SemaphoreTakeBullet = xSemaphoreTake(BinSemaphoreTakeBullet,2);//�ȴ�2ms��ʱʱ��

			if(SemaphoreTakeBullet == pdTRUE)
			{
				//�����ٶȻ�
//				EventCanTakeBullet = CAN_RoboModule_DRV_Velocity_Mode(0,	//group
//																	  10,	//number
//																	  5000, //�����ѹ
//																	  1400);//�ٶ�(RPM)
			}
		}
		//���ü��ȡ��CAN�¼�λBIT3
		if(EventGroupHandler!=NULL)
		{
			if(EventCanTakeBullet == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_3);
			}
		}
		//��ȡʣ��ջ��С
		_StackSurplus.Can2Send = uxTaskGetStackHighWaterMark(NULL);

	vTaskDelay(4);
    }
}
