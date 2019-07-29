#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t CM1;
	uint16_t CM2;
	uint16_t CM3;
	uint16_t CM4;
	uint16_t GM_yaw;
	uint16_t GM_pitch;
	uint16_t SM;
	uint16_t FM1;
	uint16_t FM2;
	uint16_t Dbus;
	uint16_t Judgement;
	uint16_t Icm20600;

} Monitor_t;

typedef struct {
	uint8_t CM1_inited:1;
	uint8_t CM2_inited:1;
	uint8_t CM3_inited:1;
	uint8_t CM4_inited:1;
	uint8_t GM_yaw_inited:1;
	uint8_t GM_pitch_inited:1;
	uint8_t SM_inited:1;
	uint8_t FM1_inited:1;
	uint8_t FM2_inited:1;
	uint8_t dbus_inited:1;
	uint8_t judgement_inited:1;
	uint8_t icm20600_inited:1;
	uint8_t all_inited:1;

	uint8_t chassis_pre;
	uint8_t chassis;										//0�����ص��ߣ�1��������2�������ص���
	uint8_t gimbal_pre;
	uint8_t gimbal;											//0�����ص��ߣ�1������
	uint8_t shooting_pre;
	uint8_t shooting;										//0�����ص��ߣ�1������
	uint8_t judgement_pre;
	uint8_t judgement;									//0�����ߣ�1����������
	uint8_t dbus_pre;
	uint8_t dbus;												//0�����ߣ�1���������ӣ�2�����������ӣ����ݴ�λ��

} Device_Status_t;

extern Device_Status_t _status;
extern Monitor_t _monitor;

#ifdef __cplusplus
}
#endif
