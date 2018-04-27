#ifndef ___CRC_H__
#define ___CRC_H__

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) ;
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) ;
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) ;
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) ;
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) ;


#ifdef __cplusplus
}
#endif

#endif


