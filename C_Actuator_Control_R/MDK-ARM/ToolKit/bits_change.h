#ifndef __BITS_CHANGE_H__
#define __BITS_CHANGE_H__

#include "stm32f4xx.h"                  // Device header

int16_t bit8TObit16(uint8_t change_info[2]);
uint32_t bit8TObit32(uint8_t change_info[4]);
uint32_t bit8TObit64(uint8_t change_info[8]);
float bit8TOfloat32(uint8_t change_info[4]);
uint8_t bit16TObit8(uint8_t Index_need,int16_t bit16);
uint8_t bit32TObit8(uint8_t Index_need,uint32_t bit32);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

#endif


