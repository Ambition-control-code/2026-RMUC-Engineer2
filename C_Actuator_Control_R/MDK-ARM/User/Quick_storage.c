#include "Quick_storage.h"
#include "PID.h"
#include "IMU.h"
#include "remote.h"
#include "arm_math.h"
#include "Motor_control.h"
#include "can.h"
#include "Get_Referee_Data.h"
#include "string.h"
#include "filter.h"

/***@role:	挂起和恢复任务的句柄,队列的句柄***/
extern osThreadId dJi_motor_armHandle;
extern osThreadId mainTaskHandle;
extern osThreadId deal_IMU_dataHandle;
extern osThreadId Data_HandleHandle;

extern osMessageQueueId_t storageHandle;
extern osMessageQueueId_t MYor_remoteHandle;
extern osMessageQueueId_t change_mode_flagHandle;
//=============================================

/***@role:	一键存储能量单元的函数***/
uint8_t join_number = 0;
uint8_t Big_number_storage_state = 0;
uint32_t wait_time = 0;
void Quick_storage_task(void)
{
	if(Big_number_storage_state == 5)
	{
		switch(join_number)
		{
			case 0:
			{
				if(fabs(DM_YAW_back.raw_pos - 0.0f) <= 1.0f)	    join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x04,0.0f,0.5f);
			}	break;
			case 1:
			{
				if(fabs(Big_Pitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x01,0.0f,0.5f);
			}	break;
			case 2:
			{
				if(fabs(Smoll_Pitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x02,0.0f,0.5f);
			}	break;
			case 3:
			{
				if(fabs(DM_ROLL_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x03,0.0f,5.0f);
			}	break;
			case 4:
			{
				if(fabs(DM_endPitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x05,0.0f,5.0f);
			}	break;
			case 5:
			{
				if(fabs(DM_endRoll_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x06,0.0f,5.0f);
			}	break;
			case 6:
			{
//				DM_MIT_Control(&hcan1,0x07,0.0f,0.0f,0.0f,0.0f,(-Claw_torque));
				join_number ++;
			}	break;
			case 7:
			{
//				if((custom_data.Custom_Data.Quick_storage_flag - last_storage_flag1) == 1)
//				{
//					last_storage_flag1 = custom_data.Custom_Data.Quick_storage_flag;
//					Big_number_storage_state = 0;
//				}
				if((New_VT03_data.fn_1 - last_storage_flag2) == 1)
				{
					last_storage_flag2 = New_VT03_data.fn_1;
					Big_number_storage_state = 0;
				}
				join_number = 0;
				remote.last_sr = remote.sr;
			}	break;
		}
	}
	else
	{
		switch(join_number)
		{
			case 0:
			{
				if(fabs(DM_YAW_back.raw_pos - 0.0f) <= 1.0f)	  join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x04,0.0f,0.5f);
			}	break;
			case 1:
			{
				if(fabs(Big_Pitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x01,0.0f,0.5f);
			}	break;
			case 2:
			{
				if(fabs(Smoll_Pitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x02,0.0f,0.5f);
			}	break;
			case 3:
			{
				if(fabs(DM_ROLL_back.raw_pos - 0.0f) <= 1.0f)		join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x03,0.0f,5.0f);
			}	break;
			case 4:
			{
				if(fabs(DM_endPitch_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x05,0.0f,5.0f);
			}	break;
			case 5:
			{
				if(fabs(DM_endRoll_back.raw_pos - 0.0f) <= 1.0f)	join_number ++;
				else	DM_Pos_Speed_Control(&hcan1,0x06,0.0f,5.0f);
			}	break;
			case 6:
			{
				if(wait_time == 0)	wait_time = HAL_GetTick();
//				DM_MIT_Control(&hcan1,0x07,0.0f,0.0f,0.0f,0.0f,(-Claw_torque));
				if((HAL_GetTick() - wait_time) > 3000)	
				{
					join_number ++;
					wait_time = 0;
				}
			}	break;
			case 7:
			{
//				if((custom_data.Custom_Data.Quick_storage_flag - last_storage_flag1) == 1)
//				{
//					last_storage_flag1 = custom_data.Custom_Data.Quick_storage_flag;
//					Big_number_storage_state = 0;
//				}
				if((New_VT03_data.fn_1 - last_storage_flag2) == 1)
				{
					last_storage_flag2 = New_VT03_data.fn_1;
					Big_number_storage_state = 0;
				}
				join_number = 0;
				remote.last_sr = remote.sr;
			}	break;
		}
	}
}


