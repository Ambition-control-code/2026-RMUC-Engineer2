#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "Motor_control.h"
#include "Quick_storage.h"
#include "can_bsp.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能（初始化）
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_CAN_Start(&hcan1);   
	HAL_CAN_Start(&hcan2);   
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/***************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	发送数据
* @rember：     电流范围-20A到+20A映射的数据为-16384到+16384
**************************************************************************/

uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *data, uint32_t len)
{
	static CAN_TxHeaderTypeDef	tx_header;
	tx_header.StdId = ID;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
		}
	}
  return 0;
}
/******
**************************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的CAN中断回调函数
* @rember：     是读取CAN信息的重要函数，切不可随意抛弃，切记！切记！！！
**************************************************************************
******/
extern uint32_t Chassis_life;
extern uint32_t yaw,B_pitch,S_pitch,B_roll,End_pitch,End_roll,Claw_state,Smoll_Roll;

CAN_RxHeaderTypeDef	R1Header;
CAN_RxHeaderTypeDef	R2Header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&R1Header,RX_data);
		switch(R1Header.StdId)
		{
			case 0x000:	 
			{	
				switch(RX_data[0])
				{
					case 0x11: DM_Normal_GetData(&DM_YAW_back);					yaw = 0;							break;//获取大Pitch达妙电机反馈数据
					case 0x12: DM_Normal_GetData(&Big_Pitch_back);			B_pitch = 0;					break;//获取小Pitch达妙电机反馈数据
					case 0x13: DM_Normal_GetData(&Big_Roll_back);			  B_roll = 0;					  break;
					case 0x14: DM_Normal_GetData(&Smoll_Pitch_back);		S_pitch = 0;					break;//获取YAW轴达妙电机反馈数据
					case 0x15: DM_Normal_GetData(&DM_ROLL_back);				Smoll_Roll = 0;				break;//获取小臂ROLL轴达妙电机反馈数据
					case 0x16: DM_Normal_GetData(&DM_endPitch_back);		End_pitch = 0;				break;//获取末端Pitch轴达妙电机反馈数据
					case 0x17: DM_Normal_GetData(&DM_endRoll_back);			End_roll = 0;					break;//获取末端Roll轴达妙电机反馈数据
					case 0x18: DM_Normal_GetData(&DM_Claw_back);        Claw_state = 0;     	break;//获取夹爪达妙电机反馈数据
				}
			}	break;
	  }
	}
	if(hcan->Instance==CAN2)
	{
		Chassis_life = 0;
		HAL_CAN_GetRxMessage(&hcan2,CAN_FILTER_FIFO0,&R2Header,RX_data);
		switch(R2Header.StdId)
		{
			case 0x000:	 
			{	
				switch(RX_data[0])
				{
					case 0x11: DM_Normal_GetData(&DM_riseMotor_back);break;
				}
			}	break;
	  }
	}
}


