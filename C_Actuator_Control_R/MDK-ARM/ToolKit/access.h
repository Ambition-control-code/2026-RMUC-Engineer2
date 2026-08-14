#ifndef __ACCESS_H
#define __ACCESS_H

#include "stm32f4xx.h"               // Device header

typedef enum
{
	Ready_mode = 0,
	storage_mode = 1,//存?的模式
	extract_mode = 2,//提取的模式
}access_mode_t;	//存取的模式?志位

void TFR_conrrol_Front(void);
void TFR_conrrol_Behind(void);
void Ready_work_task(void);




#endif


