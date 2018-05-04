
#include "Handle.h"

#include "BSP_CAN.h"
#include "BSP_TIM.h"
#include "BSP_DMA.h"
#include "BSP_SPI.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"

#include "Driver_DBUS.h"
#include "Driver_Debug.h"
#include "Driver_Judge.h"
#include "Driver_Gimbal.h"
#include "Driver_CanSend.h"
#include "Driver_Chassis.h"
#include "Driver_ICM20600.h"
#include "Driver_PowerLimit.h"
#include "Driver_ModeSwitch.h"

#include "Task_SysInitConfig.h"
#include "Task_ModeSwitch.h"
#include "Task_PowerLimit.h"
#include "Task_TakeBullet.h"
#include "Task_CanSend.h"
#include "Task_Monitor.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Judge.h"
#include "Task_Debug.h"
#include "Task_Shoot.h"
#include "Task_DBUS.h"
#include "Task_IMU.h"

/**
  * @brief  系统初始化任务
  * @param  unused
  * @retval void
  */
void Task_SysInitConfig(void *Parameters)
{
/********************************************硬件初始化**********************************************/

	HAL_Init();
	BSP_NVIC_InitConfig();
    BSP_GPIO_InitConfig();
	BSP_TIM_InitConfig();
	BSP_DMA_InitConfig();
    BSP_CAN_InitConfig();
    BSP_UART_InitConfig();
	BSP_SPI_InitConfig();

/******************************************控制模块初始化********************************************/

	PowerLimit_InitConfig();
	ModeSwitch_InitConfig();
	ICM20600_InitConfig();
	Chassis_InitConfig();
	CanSend_InitConfig();
	Gimbal_InitConfig();
	Judge_InitConfig();
	Debug_InitConfig();
	DBUS_InitConfig();

/********************************************进入临界区**********************************************/

	taskENTER_CRITICAL();

/******************************************创建二值信号量********************************************/

	BinSemaphoreDBUS = xSemaphoreCreateBinary();
	while( BinSemaphoreDBUS == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}
	BinSemaphoreJudge = xSemaphoreCreateBinary();
	while( BinSemaphoreJudge == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}
	BinSemaphoreChassis = xSemaphoreCreateBinary();
	while( BinSemaphoreChassis == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}
	BinSemaphoreGimbal = xSemaphoreCreateBinary();
	while( BinSemaphoreGimbal == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}
	BinSemaphoreShoot = xSemaphoreCreateBinary();
	while( BinSemaphoreShoot == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}
	BinSemaphoreTakeBullet = xSemaphoreCreateBinary();
	while( BinSemaphoreTakeBullet == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}

/******************************************创建事件标志组********************************************/

	EventGroupHandler = xEventGroupCreate();
	while( EventGroupHandler == NULL )
	{
	/* 没有足够堆时，可能会失败 */
	}

/********************************************创建任务************************************************/

	//DBUS任务
	xTaskCreate(Task_DBUS, "Task_DBUS", 128, NULL, 8, HandleDBUS );

    //模式选择任务
    xTaskCreate(Task_ModeSwitch, "Task_ModeSwitch", 64, NULL, 7, HandleModeSwitch );

	//裁判系统任务
	xTaskCreate(Task_Judge, "Task_Judge", 256, NULL, 7, NULL );

	//功率限制任务
	xTaskCreate(Task_PowerLimit, "Task_PowerLimit", 256, NULL, 7, NULL );

    //底盘任务
	xTaskCreate(Task_Chassis, "Task_Chassis", 128, NULL, 6, HandleChassis );

    //云台任务
	xTaskCreate(Task_Gimbal, "Task_Gimbal", 128, NULL, 6, HandleGimbal );

    //射击任务
//	xTaskCreate(Task_Shoot, "Task_Shoot", 256, NULL, 6, HandleShoot );

    //取弹任务
//	xTaskCreate(Task_TakeBullet, "Task_TakeBullet", 64, NULL, 6, HandleTakeBullet );

    //Can1发送任务
	xTaskCreate(Task_Can1Send, "Task_Can1Send", 256, NULL, 5, HandleCan1Send );

	//Can2发送任务
//	xTaskCreate(Task_Can2Send, "Task_Can2Send", 256, NULL, 5, HandleCan2Send );

    //监视任务
    xTaskCreate(Task_Monitor, "Task_Monitor", 64, NULL, 4, HandleMonitor );

	//上位机调试任务   (仅在调试时创建)
//	xTaskCreate(Task_Debug, "Task_Debug", 256, NULL, 3, NULL );

	//IMU任务  (板子没有IMU中断则创建)
	xTaskCreate(Task_IMU, "Task_IMU", 128, NULL, 2, HandleIMU );


/****************************************阻塞系统初始化任务******************************************/

    vTaskDelay(500);

/********************************************退出临界区**********************************************/

	taskEXIT_CRITICAL();

/****************************************删除系统初始化任务******************************************/

    vTaskDelete(NULL);
}

