#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "stm32f4xx.h"                  // Device header

extern uint8_t Referee_flag;
extern UART_HandleTypeDef huart6;

void BSP_USART_Init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size);

#endif


