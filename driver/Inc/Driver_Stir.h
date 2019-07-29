#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


	
//�󲦵����3508�����ṹ��
typedef struct
{
    float TargetSpeed;             //���Ŀ���ٶ�
    float RealSpeed;               //���ʵ���ٶ�rpm
    float CurrentSpeed;
	float PreBlockPositon;
    float PrePositon;
//	float TargetPosition;
	float RealPosition;
    float CurrentPosition;
    float LastPosition;
    uint16_t EncoderPosition;
//	float offset_angle;
	int32_t Round_cnt;
	uint32_t msg_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;
    
    uint8_t Sensor;             
	uint8_t Sensor_last;  	         
	uint8_t Bullet_count;
	uint8_t Stop_flag;        
	uint16_t Bullet_tick;         
	uint16_t Shoot_count;
	uint16_t Blocked_tick;

//	float kp_p;
//	float ki_p;
//	float kd_p;
	
	float kp_v;
	float ki_v;
	float kd_v;

}Stir42Param_t;

//С�������2310�����ṹ��
typedef struct
{
    float TargetSpeed;             //���Ŀ���ٶ�
    float RealSpeed;               //���ʵ���ٶ�rpm
    float CurrentSpeed;
	float PrePositon;
	float TargetPosition;
	float RealPosition;
    float CurrentPosition;

    uint16_t EncoderPosition;
	float offset_angle;
	int32_t Round_cnt;
	uint32_t msg_cnt;
	int16_t TargetCurrent;            
	int16_t MaxStirSpeed;

	int32_t Blocked_tick;			//��תʱ��
	uint8_t Reverse_task;           //��תģʽ��־λ
	uint16_t Reverse_tick;			//��ת���תʱ��
	uint8_t Shoot_task;				//ִ�з��������־λ
	uint8_t Shoot_num;				//���������ӵ���
	uint16_t Shoot_count;            //��¼�ѷ����ӵ��� 
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
void Stir42_ON(void);
void Stir42_OFF(void);
void Stir17_ON(void);
void Stir17_OFF(void);

#ifdef __cplusplus
}
#endif
