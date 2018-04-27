#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef __packed struct {
	//����������״̬
	uint32_t   StateTick;
	uint16_t   stageRemianTime;
    uint8_t    gameProgress;
    uint8_t    robotLevel;
    uint16_t   remainHP;
    uint16_t   maxHP;

	//�������˺�����
	__packed struct {
    uint8_t   armorType  : 4;
    uint8_t   hurtType   : 4;
	}hurt;

	//ʵʱ�������
	uint32_t ShootTick;
	uint8_t  bulletType;
    uint8_t  bulletFreq;
    float    bulletSpeed;

	//ʵʱ���ʺ���������
	uint32_t  PowerHeatTick;
	float     chassisVolt;
    float     chassisCurrent;
    float     chassisPower;
    float     chassisPowerBuffer;
    uint16_t  shooterHeat0;
    uint16_t  shooterHeat1;

	//���ؽ�������
	uint8_t   cardType;
    uint8_t   cardIdx;

	//�������
    uint8_t  winner;

	//Buff��ȡ����
    uint8_t   buffType;
    uint8_t   buffAddition;

	//������λ�ó�����Ϣ
    float x;
	float y;
	float z;
	float yaw;

	//�������Զ�������
    float ShowData1;
	float ShowData2;
	float ShowData3;
	uint8_t mask;
}Judge_t;

extern Judge_t _Judge;

extern int8_t judge_read(void);
extern void Judge_InitConfig(void);

#ifdef __cplusplus
}
#endif
