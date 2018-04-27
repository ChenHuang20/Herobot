#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ON  1
#define OFF 0

#define EVENTBIT_0     (1 << 0)
#define EVENTBIT_1     (1 << 1)
#define EVENTBIT_2     (1 << 2)
#define EVENTBIT_3     (1 << 3)
#define EVENTBIT_ALL   (EVENTBIT_0 | EVENTBIT_1 | EVENTBIT_2 | EVENTBIT_3 )

typedef struct {
	UBaseType_t PowerLimit;
	UBaseType_t ModeSwitch;
	UBaseType_t TakeBullet;
	UBaseType_t CanSend;
	UBaseType_t Monitor;
	UBaseType_t Chassis;
	UBaseType_t Gimbal;
	UBaseType_t Judge;
	UBaseType_t Debug;
	UBaseType_t Shoot;
	UBaseType_t DBUS;
	UBaseType_t IMU;
} Tick_t;

typedef struct {
	UBaseType_t PowerLimit;
	UBaseType_t ModeSwitch;
	UBaseType_t TakeBullet;
	UBaseType_t CanSend;
	UBaseType_t Monitor;
	UBaseType_t Chassis;
	UBaseType_t Gimbal;
	UBaseType_t Debug;
	UBaseType_t Judge;
	UBaseType_t Shoot;
	UBaseType_t DBUS;
	UBaseType_t IMU;
} StackSurplus_t;

extern Tick_t _Tick;

extern StackSurplus_t _StackSurplus;

extern TaskHandle_t * HandleModeSwitch;
extern TaskHandle_t * HandleTakeBullet;
extern TaskHandle_t * HandleCanSend;
extern TaskHandle_t * HandleMonitor;
extern TaskHandle_t * HandleChassis;
extern TaskHandle_t * HandleGimbal;
extern TaskHandle_t * HandleShoot;
extern TaskHandle_t * HandleDBUS;
extern TaskHandle_t * HandleIMU;

extern SemaphoreHandle_t BinSemaphoreDBUS;
extern SemaphoreHandle_t BinSemaphoreJudge;
extern SemaphoreHandle_t BinSemaphoreChassis;
extern SemaphoreHandle_t BinSemaphoreGimbal;

extern EventGroupHandle_t EventGroupHandler;

#ifdef __cplusplus
}
#endif
