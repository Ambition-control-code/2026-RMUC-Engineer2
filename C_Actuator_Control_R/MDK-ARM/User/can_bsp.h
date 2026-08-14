#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "can.h"

#define Motor_Yaw 0x01
#define Motor_BigPitch 0x02
#define Motor_BigRoll 0x03
#define Motor_SmollPitch 0x04
#define Motor_SmollRoll 0x05
#define Motor_EndPitch 0x06
#define Motor_EndRoll 0x07
#define Motor_Claw 0x08

#define Control_Motor_hcan (&hcan1)
#define TRX_hcan (&hcan2)


void can_bsp_init(void);
uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *data, uint32_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern CAN_RxHeaderTypeDef R1Header;
extern CAN_RxHeaderTypeDef R2Header;

#endif


