#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    float Real_Power[3]; //2���� 1�ϴ� 0���ϴ�
	float RemainPower[3];
	float Set_Power;
	float MaxSpeed;  //����ٶ�ֵ
	uint8_t Cut;        //�Ƿ��жϵ������
	float Zoom;    //���ű��ʷ�ֹ�������
	float ZoomTemp;//�����ݴ�ֵ
	uint8_t ZoomCount;//����Forѭ������
	uint8_t Flag;//��־λ����������Ƿ��˳�������
	uint8_t K_stall;//�����洢�ս������������ٶ�
	int16_t pidTemp;//�Զ�������ʱ�������洢PID���ֵ
	float RealSpeed;
	float v;
} PowerLimit_t;

typedef struct
{
	int16_t Bias[5];//��ֵ=�����ٶ�-ʵ���ٶ�
	int16_t MaxBias;  //����ֵ
	uint8_t MotorID;//����ֵ�����ID
	float Zoom[5];    //���ű��ʷ�ֹ�������
	float ZoomTemp;//�����ݴ�ֵ
	uint8_t ZoomCount;//����Forѭ������
	float Increment[5];//�������
} Increment_t;

void CMControlLoop(void);
void Moving_Trial(float *MotoSpeed);
void Power_Deal(void);

void PowerLimitLoop(void);
int32_t Motor_Increment_Deal(uint8_t Motor_ID,int32_t Set_Val,float Increment);
void Motor_Increment_Self_Deal(float* MotoSpeed);
void PowerLimit_InitConfig(void);

extern PowerLimit_t PowerLimit;
extern Increment_t Increment;
extern int32_t Real_Speed[7];


#ifdef __cplusplus
}
#endif
