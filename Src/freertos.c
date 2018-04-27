
#include "Handle.h"

TaskHandle_t * HandleModeSwitch;
TaskHandle_t * HandleTakeBullet;
TaskHandle_t * HandleCanSend;
TaskHandle_t * HandleMonitor;
TaskHandle_t * HandleChassis;
TaskHandle_t * HandleGimbal;
TaskHandle_t * HandleShoot;
TaskHandle_t * HandleDBUS;
TaskHandle_t * HandleIMU;

//二值信号量句柄
SemaphoreHandle_t BinSemaphoreDBUS;
SemaphoreHandle_t BinSemaphoreJudge;
SemaphoreHandle_t BinSemaphoreChassis;
SemaphoreHandle_t BinSemaphoreGimbal;

//事件标志组句柄
EventGroupHandle_t EventGroupHandler;

Tick_t _Tick = { 0 };

StackSurplus_t _StackSurplus = { 0 };

