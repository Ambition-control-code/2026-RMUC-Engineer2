#ifndef __VOFA_H__
#define __VOFA_H__

#include "stm32f4xx.h"               // Device header

#define VOFA_MAX_DATA 10            //最大发送数据个数，10个够用了，想要加的话也可以加大

void VOFA_Print(uint8_t number,...);

#endif


