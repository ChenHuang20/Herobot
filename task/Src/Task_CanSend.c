
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
  * @brief  CAN1发送任务（底盘3508×4 + 云台6623×2）
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
		//获取底盘任务信号量
		if(BinSemaphoreChassis!=NULL)
		{
			SemaphoreChassis = xSemaphoreTake(BinSemaphoreChassis,2);//等待2ms超时时间

			if(SemaphoreChassis == pdTRUE)
			{
				EventCanChassis = send_cm_current(&hcan1,						   //CAN句柄
												  _ChassisParam.ChassisCurrent[0], //轮子1电流  3508
												  _ChassisParam.ChassisCurrent[1], //轮子2电流	3508
												  _ChassisParam.ChassisCurrent[2], //轮子3电流	3508
												  _ChassisParam.ChassisCurrent[3]);//轮子4电流	3508
			}
		}
		//设置监控底盘CAN事件位BIT0
		if(EventGroupHandler!=NULL)
		{
			if(EventCanChassis == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_0);
			}
		}
		//获取云台任务信号量
		if(BinSemaphoreGimbal!=NULL)
		{
			SemaphoreGimbal = xSemaphoreTake(BinSemaphoreGimbal,2);//等待2ms超时时间

			if(SemaphoreGimbal == pdTRUE)
			{
				EventCanGimbal = send_gimbal_current(&hcan1,
//				//CAN句柄
				                                     _GimbalParam[YAW].SendCurrent, //yaw电流	 6623
													 _GimbalParam[PIT].SendCurrent);//pitch电流   6623
				EventCanGimbal = send_gimbal_current(&hcan1,
				//CAN句柄
				                                     (int16_t)(_pid.output_gimbal[2] * 2000.0f), //yaw电流	 6623
													 (int16_t)(_pid.output_gimbal[1] * 2000.0f));//pitch电流   6623
			}
		}
		//设置监控云台CAN事件位BIT1
		if(EventGroupHandler!=NULL)
		{
			if(EventCanGimbal == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_1);
			}
		}
		//获取剩余栈大小
		_StackSurplus.Can1Send = uxTaskGetStackHighWaterMark(NULL);
taskEXIT_CRITICAL();
	vTaskDelay(4);
    }
}


/**
  * @brief  CAN2发送任务（大拨弹电机3508×1、小拨弹电机2310×1  蓝盒子×2  大、小摩擦轮3510×2）
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
		//获取射击任务信号量
		if(BinSemaphoreShoot!=NULL)
		{
			SemaphoreShoot = xSemaphoreTake(BinSemaphoreShoot,2);////等待2ms超时时间

			if(SemaphoreShoot == pdTRUE)
			{
				EventCanShoot = send_shoot_current(&hcan2,//CAN句柄
													_Fric42Param.TargetCurrent[0],   //大摩擦轮1电流  3510
													_Fric42Param.TargetCurrent[1],   //大摩擦轮2电流  3510
													_Stir17Param.TargetCurrent,    //小拨弹电机电流 2310
													_Stir42Param.TargetCurrent);   //大拨弹电机电流 3508
			}
		}
		//设置监控射击CAN事件位BIT2
		if(EventGroupHandler!=NULL)
		{
			if(EventCanShoot == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_2);
			}
		}
		//获取取弹任务信号量
		if(BinSemaphoreTakeBullet!=NULL)
		{
			SemaphoreTakeBullet = xSemaphoreTake(BinSemaphoreTakeBullet,2);//等待2ms超时时间

			if(SemaphoreTakeBullet == pdTRUE)
			{
				//蓝盒速度环
//				EventCanTakeBullet = CAN_RoboModule_DRV_Velocity_Mode(0,	//group
//																	  10,	//number
//																	  5000, //输出电压
//																	  1400);//速度(RPM)
			}
		}
		//设置监控取弹CAN事件位BIT3
		if(EventGroupHandler!=NULL)
		{
			if(EventCanTakeBullet == HAL_OK)
			{
				xEventGroupSetBits(EventGroupHandler,EVENTBIT_3);
			}
		}
		//获取剩余栈大小
		_StackSurplus.Can2Send = uxTaskGetStackHighWaterMark(NULL);

	vTaskDelay(4);
    }
}
