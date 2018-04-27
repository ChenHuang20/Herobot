
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
#include "Driver_Chassis.h"
#include "Driver_ICM20600.h"
#include "Driver_PowerLimit.h"


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

	Chassis_InitConfig();
	PowerLimit_InitConfig();
	DBUS_InitConfig();
	Judge_InitConfig();
//	Debug_InitConfig();

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

/******************************************�����¼���־��********************************************/

	EventGroupHandler = xEventGroupCreate();
	while( EventGroupHandler == NULL )
	{
	/* û���㹻��ʱ�����ܻ�ʧ�� */
	}

/********************************************��������************************************************/

	//DBUS����
	xTaskCreate(Task_DBUS, "Task_DBUS", 128, NULL, 9, HandleDBUS );

	//IMU����
	xTaskCreate(Task_IMU, "Task_IMU", 128, NULL, 1, HandleIMU );

    //��������
    xTaskCreate(Task_Monitor, "Task_Monitor", 64, NULL, 1, HandleMonitor );

    //ģʽѡ������
//    xTaskCreate(Task_ModeSwitch, "Task_ModeSwitch", 64, NULL, 1, HandleModeSwitch );

    //��������
	xTaskCreate(Task_Chassis, "Task_Chassis", 128, NULL, 3, HandleChassis );

    //��̨����
//	xTaskCreate(Task_Gimbal, "Task_Gimbal", 64, NULL, 2, HandleGimbal );

    //�������
//	xTaskCreate(Task_Shoot, "Task_Shoot", 64, NULL, 2, HandleShoot );

    //Can��������
	xTaskCreate(Task_CanSend, "Task_CanSend", 512, NULL, 4, HandleCanSend );

	//����ϵͳ����
	xTaskCreate(Task_Judge, "Task_Judge", 256, NULL, 8, NULL );

	//������������
	xTaskCreate(Task_PowerLimit, "Task_PowerLimit", 256, NULL, 7, NULL );

    //ȡ������
//	xTaskCreate(Task_TakeBullet, "Task_TakeBullet", 64, NULL, 2, HandleTakeBullet );

	//��λ����������
//	xTaskCreate(Task_Debug, "Task_Debug", 256, NULL, 1, NULL );



/****************************************����ϵͳ��ʼ������******************************************/

    vTaskDelay(500);

/********************************************�˳��ٽ���**********************************************/

	taskEXIT_CRITICAL();

/****************************************ɾ��ϵͳ��ʼ������******************************************/

    vTaskDelete(NULL);
}

