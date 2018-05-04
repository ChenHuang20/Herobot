#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


	
//�󲦵����3508�����ṹ��
typedef struct
{
    float TargetSpeed;               //���Ŀ���ٶ�
    int16_t RealSpeed;               //���ʵ���ٶ�
	int16_t TargetCurrent;             //����������
	int16_t MaxStirSpeed;
	float kp;
	float ki;
	float kd;
}Stir42Param_t;

//С�������2310�����ṹ��
typedef struct
{
    float TargetSpeed;               //���Ŀ���ٶ�
    float RealSpeed;               //���ʵ���ٶ�
	float PrePositon;
	float TargetPosition;
	float RealPosition;
	int32_t Round_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;
	int32_t Blocked_tick;			//��תʱ��
	uint8_t Reverse_task;           //��תģʽ��־λ
	uint16_t Reverse_tick;			//��ת���תʱ��
	uint8_t Shoot_task;				//ִ�з��������־λ
	uint8_t Shoot_num;				//���������ӵ���
	uint16_t Shoot_count;            //��¼�ѷ����ӵ��� ��0~�����
	uint16_t Shoot_lastcount;        //��¼��һ���ӵ���
	uint16_t Shoot_time;			//�����������ʱ��
	uint16_t Shoot_tick;			//��¼������ʱ��
	float kp_p;
	float ki_p;
	float kd_p;
	
	float kp_v;
	float ki_v;
	float kd_v;

}Stir17Param_t;

extern Stir42Param_t _Stir42Param;
extern Stir17Param_t _Stir17Param;
void Stir_InitConfig(void);

#ifdef __cplusplus
}
#endif
