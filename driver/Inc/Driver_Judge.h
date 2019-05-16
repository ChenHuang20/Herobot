#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef __packed struct {
	//比赛机器人状态
	uint32_t   StateTick;
	uint16_t   stageRemianTime;
    uint8_t    gameProgress;
    uint8_t    robotLevel;
    uint16_t   remainHP;
    uint16_t   maxHP;

	//机器人伤害数据
	__packed struct {
    uint8_t   armorType  : 4;
    uint8_t   hurtType   : 4;
	}hurt;

	//实时射击数据
	uint32_t ShootTick;
	uint8_t  bulletType;
    uint8_t  bulletFreq[2];
    float    bulletSpeed[2];

	//实时功率和热量数据
	uint32_t  PowerHeatTick;
	float     chassisVolt;
    float     chassisCurrent;
    float     chassisPower;
    float     chassisPowerBuffer;
    uint16_t  shooterHeat0;
    uint16_t  shooterHeat1;

	//场地交互数据
	uint8_t   cardType;
    uint8_t   cardIdx;

	//比赛结果
    uint8_t  winner;

	//Buff获取数据
    uint16_t   buffType;

	//机器人位置朝向信息
    float x;
	float y;
	float z;
	float yaw;

	//参赛队自定义数据
    float ShowData1;
	float ShowData2;
	float ShowData3;
	uint8_t mask;
}Judge_t;

typedef struct
{
	uint8_t SendAllow;
	uint8_t SOF;                               //包头
	uint16_t DataLength;                       //数据长度
	uint8_t Seq;                               //包序号
	uint8_t CRC8;                              //帧头校验
	uint16_t CmdID;                            //命令码ID
	float DataA;                               //自定义数据ABC
	float DataB;                           
	float DataC;
	uint8_t BaseStatus;
	uint8_t IslandStatus;
	uint8_t BuffStatus;
	uint16_t CRC16;                            //全帧校验
}JudgeSend_t;

typedef union     //float数据重组
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
