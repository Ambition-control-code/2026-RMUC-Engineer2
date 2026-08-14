#ifndef __CRC_H__
#define __CRC_H__

#include "stm32f4xx.h"                  // Device header

uint8_t get_CRC8_check_sum(unsigned char *pch_message,unsigned int dw_length,unsigned char ucCRC8);
uint32_t verify_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length);
void append_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length);
uint16_t get_CRC16_check_sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC);
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

