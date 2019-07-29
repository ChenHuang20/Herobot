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
    uint8_t  bulletFreq[2];
    float    bulletSpeed[2];

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
    uint16_t   buffType;

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

typedef struct
{
	uint8_t SendAllow;
	uint8_t SOF;                               //��ͷ
	uint16_t DataLength;                       //���ݳ���
	uint8_t Seq;                               //�����
	uint8_t CRC8;                              //֡ͷУ��
	uint16_t CmdID;                            //������ID
	float DataA;                               //�Զ�������ABC
	float DataB;                           
	float DataC;
	uint8_t BaseStatus;
	uint8_t IslandStatus;
	uint8_t BuffStatus;
	uint16_t CRC16;                            //ȫ֡У��
}JudgeSend_t;

typedef union     //float��������
{  
    struct   
    {  
        unsigned char low_byte;  
        unsigned char mlow_byte;  
        unsigned char mhigh_byte;  
        unsigned char high_byte;  
     }float_byte;  
            
     float  value;  
}FLAOT_UNION;

extern JudgeSend_t _JudgeSend;
extern Judge_t _Judge;

extern int8_t judge_read(void);
extern void Judge_InitConfig(void);
extern void Judge_SendData(void);

#ifdef __cplusplus
}
#endif
