
#include "Handle.h"

TaskHandle_t * HandleModeSwitch;
TaskHandle_t * HandleTakeBullet;
TaskHandle_t * HandleCan1Send;
TaskHandle_t * HandleCan2Send;
TaskHandle_t * HandleMonitor;
TaskHandle_t * HandleChassis;
TaskHandle_t * HandleGimbal;
TaskHandle_t * HandleShoot;
TaskHandle_t * HandleDBUS;
TaskHandle_t * HandleIMU;

//��ֵ�ź������
SemaphoreHandle_t BinSemaphoreDBUS;
SemaphoreHandle_t BinSemaphoreShoot;
SemaphoreHandle_t BinSemaphoreJudge;
SemaphoreHandle_t BinSemaphoreGimbal;
SemaphoreHandle_t BinSemaphoreChassis;
SemaphoreHandle_t BinSemaphoreTakeBullet;

//�¼���־����
EventGroupHandle_t EventGroupHandler;

Tick_t _Tick = { 0 };

StackSurplus_t _StackSurplus = { 0 };

