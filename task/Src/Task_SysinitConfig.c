
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
  * @brief  ϵͳ��ʼ������
  * @param  unused
  * @retval void
  */
void Task_SysInitConfig(void *Parameters)
{
/********************************************Ӳ����ʼ��**********************************************/

	HAL_Init();
	BSP_NVIC_InitConfig();
    BSP_GPIO_InitConfig();
	BSP_TIM_InitConfig();
	BSP_DMA_InitConfig();
    BSP_CAN_InitConfig();
    BSP_UART_InitConfig();
	BSP_SPI_InitConfig();

/******************************************����ģ���ʼ��********************************************/

	PowerLimit_InitConfig();
	ModeSwitch_InitConfig();
	ICM20600_InitConfig();
	Chassis_InitConfig();
	CanSend_InitConfig();
	Gimbal_InitConfig();
	Judge_InitConfig();
	Debug_InitConfig();
	DBUS_InitConfig();

/********************************************�����ٽ���**********************************************/

	taskENTER_CRITICAL();

/******************************************������ֵ�ź���********************************************/

	BinSemaphoreDBUS = xSemaphoreCreateBinary();
	while( BinSemaphoreDBUS == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}
	BinSemaphoreJudge = xSemaphoreCreateBinary();
	while( BinSemaphoreJudge == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}
	BinSemaphoreChassis = xSemaphoreCreateBinary();
	while( BinSemaphoreChassis == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}
	BinSemaphoreGimbal = xSemaphoreCreateBinary();
	while( BinSemaphoreGimbal == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}
	BinSemaphoreShoot = xSemaphoreCreateBinary();
	while( BinSemaphoreShoot == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}
	BinSemaphoreTakeBullet = xSemaphoreCreateBinary();
	while( BinSemaphoreTakeBullet == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}

/******************************************�����¼���־��********************************************/

	EventGroupHandler = xEventGroupCreate();
	while( EventGroupHandler == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}

/********************************************��������************************************************/

	//DBUS����
	xTaskCreate(Task_DBUS, "Task_DBUS", 128, NULL, 8, HandleDBUS );

    //ģʽѡ������
    xTaskCreate(Task_ModeSwitch, "Task_ModeSwitch", 64, NULL, 7, HandleModeSwitch );

	//����ϵͳ����
	xTaskCreate(Task_Judge, "Task_Judge", 256, NULL, 7, NULL );

	//������������
	xTaskCreate(Task_PowerLimit, "Task_PowerLimit", 256, NULL, 7, NULL );

    //��������
	xTaskCreate(Task_Chassis, "Task_Chassis", 128, NULL, 6, HandleChassis );

    //��̨����
	xTaskCreate(Task_Gimbal, "Task_Gimbal", 128, NULL, 6, HandleGimbal );

    //�������
//	xTaskCreate(Task_Shoot, "Task_Shoot", 256, NULL, 6, HandleShoot );

    //ȡ������
//	xTaskCreate(Task_TakeBullet, "Task_TakeBullet", 64, NULL, 6, HandleTakeBullet );

    //Can1��������
	xTaskCreate(Task_Can1Send, "Task_Can1Send", 256, NULL, 5, HandleCan1Send );

	//Can2��������
//	xTaskCreate(Task_Can2Send, "Task_Can2Send", 256, NULL, 5, HandleCan2Send );

    //��������
    xTaskCreate(Task_Monitor, "Task_Monitor", 64, NULL, 4, HandleMonitor );

	//��λ����������   (���ڵ���ʱ����)
//	xTaskCreate(Task_Debug, "Task_Debug", 256, NULL, 3, NULL );

	//IMU����  (����û��IMU�ж��򴴽�)
	xTaskCreate(Task_IMU, "Task_IMU", 128, NULL, 2, HandleIMU );


/****************************************����ϵͳ��ʼ������******************************************/

    vTaskDelay(500);

/********************************************�˳��ٽ���**********************************************/

	taskEXIT_CRITICAL();

/****************************************ɾ��ϵͳ��ʼ������******************************************/

    vTaskDelete(NULL);
}

