#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//��̨��������ṹ��
typedef struct
{
    uint16_t RealEncoderAngle; //ʵ�ʽǶȱ�����ֵ
	float RealSpeed;         //ʵ���ٶ�
    float RealABSAngle;        //ʵ�ʽǶȣ�����ֵ��
    float TargetAngle;         //Ŀ��Ƕ�
	float TargetRates;		   //Ŀ����ٶ�
	float TargetCurrent;	   //Ŀ�����
    uint16_t FrameCounter;     //֡�ʼ�����
    uint16_t FrameRate;        //֡��

	float kp[2];
	float ki[2];
	float kd[2];
}GimbalParam_t;

extern GimbalParam_t PitchParam;
extern GimbalParam_t YawParam;

void Gimbal_InitConfig(void);

#ifdef __cplusplus
}
#endif
