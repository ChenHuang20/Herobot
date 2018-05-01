#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


//���������
typedef struct
{
    float TargetSpeed;               //���Ŀ���ٶ�
    int16_t RealSpeed;               //���ʵ���ٶ�
    uint16_t LimitCurrent;           //���Ƶ���
    uint16_t RealCurrent;            //ʵ�ʵ���
    uint16_t NeedCurrent;            //�������
}OneMotorParam_t;

//���̵�������ṹ��
typedef struct
{
    OneMotorParam_t Motor[4];
    float TargetVX;
    float TargetVY;
    float TargetOmega;
    float TargetABSAngle;
	float MaxWheelSpeed;
	int16_t ChassisCurrent[4];
	int16_t ChassisMaxSumCurrent;
	float kp[5];
	float ki[5];
	float kd[5];
}ChassisParam_t;


extern OneMotorParam_t _OneMotorParam;
extern ChassisParam_t _ChassisParam;

void Current_Distribution(int16_t *current, uint16_t total_current);
void Chassis_SpeedSet(float XSpeed, float YSpeed, float OmegaSpeed);
void Chassis_InitConfig(void);

#ifdef __cplusplus
}
#endif
