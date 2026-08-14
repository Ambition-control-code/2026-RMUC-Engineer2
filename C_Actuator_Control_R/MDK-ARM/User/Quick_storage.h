#ifndef __QUICK_STORAGE_H__
#define __QUICK_STORAGE_H__

#include "stm32f4xx.h"                // Device header

extern uint8_t Big_number_storage_state;

typedef struct 
{
    uint8_t Quick_storage_sate;
	uint32_t data1;
	uint16_t data2;
	uint8_t  data3;
}Chassis_data_t;
extern Chassis_data_t Chassis_data;

void Quick_storage_task(void);
	
#endif


