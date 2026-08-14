#include "Get_Referee_Data.h"
#include "CRC.h"
#include "bits_change.h"
#include "stdbool.h"
#include "Controller_Data.h"
#include "string.h"

uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
Referee_System_Info_TypeDef Referee_System_Info;
static void Referee_System_Info_Update(uint8_t *Buff,Referee_System_Info_TypeDef *Referee_System_Info);
custom_data_union custom_data;
extern uint8_t remote_MyControl_flag;

/***@role 更新裁判系统数据的函数，将裁判系统的数据包解析，提纯出有用的数据信息***/
void Referee_System_Frame_Update(uint8_t *Buff)
{
	Referee_System_Info.Index = 0;
	Referee_System_Info.DataLength = 0;
	
	/*Check the header frame 检查数据帧头*/
  while(Buff[Referee_System_Info.Index] == 0xA5)
	{
		/*CRC8 verification CRC8数据校验*/
    if(verify_CRC8_check_sum(&Buff[Referee_System_Info.Index],FrameHeader_Length) == true)
    {
			/*Update data length 更新数据长度*/
      Referee_System_Info.DataLength = 
			                           (uint16_t)(Buff[Referee_System_Info.Index+2]<<8 | Buff[Referee_System_Info.Index+1]) 
			                           + FrameHeader_Length + CMDID_Length + CRC16_Length;
      
		  /*CRC16 verification CRC16数据校验*/
			if(verify_CRC16_check_sum(&Buff[Referee_System_Info.Index],Referee_System_Info.DataLength) == true)
      {
				/*Update the referee system data 更新裁判系统数据*/
        Referee_System_Info_Update(Buff,&Referee_System_Info);
			}
		}
		else{break;}
		/*Continue updating	继续更新*/
		Referee_System_Info.Index += Referee_System_Info.DataLength;
	}
}

/**
  * @brief  Update the referee system data 
  * @param  *Buff: pointer to a array that contains the information of the received message
  * @param  *Referee_System_Info pointer to a array that contains the information of Referee_System
  * @role   用于解析RoboMaster裁判系统串口通信协议的帧数据处理函数
  * @retval None
  */
uint32_t Zhou = 0;
KeyBoard_Value_Data_t KeyBoard_Value_Data;
static void Referee_System_Info_Update(uint8_t *Buff,Referee_System_Info_TypeDef *Referee_System_Info)
{
  switch (bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length]))
  {
#ifdef GAME_STATUS_ID 
    case GAME_STATUS_ID:
				Referee_System_Info->game_status.game_type         =  Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length] & 0x0F  ;
				Referee_System_Info->game_status.game_progress     = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length] & 0xF0) >> 4  ;
				Referee_System_Info->game_status.stage_remain_time = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1]);
				Referee_System_Info->game_status.SyncTimeStamp     = bit8TObit64(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif

#ifdef GAME_RESULT_ID
		case GAME_RESULT_ID:
		    Referee_System_Info->game_result.winner = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
		break;
#endif

#ifdef GAME_ROBOT_HP_ID
    case GAME_ROBOT_HP_ID:
				Referee_System_Info->game_robot_HP.red_1_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
				Referee_System_Info->game_robot_HP.red_2_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
				Referee_System_Info->game_robot_HP.red_3_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
				Referee_System_Info->game_robot_HP.red_4_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 6]);
				Referee_System_Info->game_robot_HP.red_7_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10]);
				Referee_System_Info->game_robot_HP.red_outpost_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12]);
				Referee_System_Info->game_robot_HP.red_base_HP    = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 14]);

				Referee_System_Info->game_robot_HP.blue_1_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 16]);
				Referee_System_Info->game_robot_HP.blue_2_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 18]);
				Referee_System_Info->game_robot_HP.blue_3_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 20]);
				Referee_System_Info->game_robot_HP.blue_4_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 22]);
				Referee_System_Info->game_robot_HP.blue_7_robot_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 26]);
				Referee_System_Info->game_robot_HP.blue_outpost_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 28]);
				Referee_System_Info->game_robot_HP.blue_base_HP    = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 30]);
    break;
#endif

#ifdef EVENET_DATA_ID
    case EVENET_DATA_ID:
        Referee_System_Info->event_data.event_data = bit8TObit32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
    break;
#endif

#ifdef REFEREE_WARNING_ID
    case REFEREE_WARNING_ID:
				Referee_System_Info->referee_warning.level = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
				Referee_System_Info->referee_warning.offending_robot_id = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+1];
				Referee_System_Info->referee_warning.count = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+2];
	  break;
#endif

#ifdef DART_INFO_ID
    case DART_INFO_ID:
        Referee_System_Info->dart_info.dart_remaining_time = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
		    Referee_System_Info->dart_info.dart_info = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+1]);
		break;
#endif

#ifdef ROBOT_STATUS_ID
    case ROBOT_STATUS_ID:
		    Referee_System_Info->robot_status.robot_id = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
		    Referee_System_Info->robot_status.robot_level= Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
		    Referee_System_Info->robot_status.current_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
		    Referee_System_Info->robot_status.maximum_HP = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
		    Referee_System_Info->robot_status.shooter_barrel_cooling_value= bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 6]);
		    Referee_System_Info->robot_status.shooter_barrel_heat_limit   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
		    Referee_System_Info->robot_status.chassis_power_limit         = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10]);
        Referee_System_Info->robot_status.mains_power_gimbal_output = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12] & 0x01);
        Referee_System_Info->robot_status.mains_power_chassis_output= (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12] & 0x02) >> 1;
        Referee_System_Info->robot_status.mains_power_shooter_output= (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12] & 0x04) >> 2;
    break;
#endif

#ifdef POWER_HEAT_DATA_ID
    case POWER_HEAT_DATA_ID:
				Referee_System_Info->power_heat_data.buffer_energy = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
				Referee_System_Info->power_heat_data.shooter_17mm_1_barrel_heat = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10]);
				Referee_System_Info->power_heat_data.shooter_17mm_2_barrel_heat = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12]);
				Referee_System_Info->power_heat_data.shooter_42mm_barrel_heat   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 14]);
    break;
#endif

#ifdef ROBOT_POS_ID
    case ROBOT_POS_ID:
				Referee_System_Info->robot_pos.x     = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
				Referee_System_Info->robot_pos.y     = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
				Referee_System_Info->robot_pos.angle = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
    break;
#endif

#ifdef BUFF_ID
    case BUFF_ID:
				Referee_System_Info->buff.recovery_buff = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
				Referee_System_Info->buff.cooling_buff  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
				Referee_System_Info->buff.defence_buff  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2];
				Referee_System_Info->buff.vulnerability_buff  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 3];
				Referee_System_Info->buff.attack_buff = bit8TObit16 (&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
				Referee_System_Info->buff.remaining_energy = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 5];
    break;
#endif

#ifdef HURT_DATA_ID
    case HURT_DATA_ID:
      Referee_System_Info->hurt_data.armor_id = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length] & 0x0F ;
      Referee_System_Info->hurt_data.HP_deduction_reason = (Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length] & 0xF0) >>4;
    break;
#endif

#ifdef SHOOT_DATA_ID
    case SHOOT_DATA_ID:
      Referee_System_Info->shoot_data.bullet_type  = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
      Referee_System_Info->shoot_data.shooter_number = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
      Referee_System_Info->shoot_data.launching_frequency = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2];
      Referee_System_Info->shoot_data.initial_speed = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 3]);
    break;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
    case PROJECTILE_ALLOWANCE_ID:
      Referee_System_Info->projectile_allowance.projectile_allowance_17mm = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
      Referee_System_Info->projectile_allowance.projectile_allowance_42mm = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
      Referee_System_Info->projectile_allowance.remaining_gold_coin       = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef RFID_STATUS_ID
    case RFID_STATUS_ID:
      Referee_System_Info->rfid_status.rfid_status = bit8TObit32(&Buff[Referee_System_Info->Index+FrameHeader_Length+CMDID_Length]);
    break;
#endif

#ifdef DART_CLIENT_CMD_ID
    case DART_CLIENT_CMD_ID:
      Referee_System_Info->dart_client_cmd.dart_launch_opening_status = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
      Referee_System_Info->dart_client_cmd.reserved = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1];
      Referee_System_Info->dart_client_cmd.target_change_time = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
      Referee_System_Info->dart_client_cmd.latest_launch_cmd_time = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
    break;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    case GROUND_ROBOT_POSITION_ID:
      Referee_System_Info->ground_robot_position.hero_x       = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
      Referee_System_Info->ground_robot_position.hero_y       = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
      Referee_System_Info->ground_robot_position.engineer_x   = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
      Referee_System_Info->ground_robot_position.engineer_y   = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12]);
      Referee_System_Info->ground_robot_position.standard_3_x = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 16]);
      Referee_System_Info->ground_robot_position.standard_3_y = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 20]);
      Referee_System_Info->ground_robot_position.standard_4_x = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 24]);
      Referee_System_Info->ground_robot_position.standard_4_y = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 28]);
    break;
#endif

#ifdef RADAR_MARAKING_DATA_ID
	 case RADAR_MARAKING_DATA_ID :
     Referee_System_Info->radar_mark_data.mark_progress = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
   break;
#endif
	
#ifdef SENTRY_INFO_ID
	 case SENTRY_INFO_ID:
		 Referee_System_Info->sentry_info.sentry_info   = bit8TObit32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
	   Referee_System_Info->sentry_info.sentry_info_2 = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
	 break;
#endif
	 
#ifdef RADAR_INFO_ID
	 case RADAR_INFO_ID:
		 Referee_System_Info->radar_info.radar_info = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
	 break;
#endif	 

#ifdef ROBOT_INTERACTION_DATA_ID
   case ROBOT_INTERACTION_DATA_ID:
		  Referee_System_Info->robot_interaction_data.data_cmd_id = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
	    Referee_System_Info->robot_interaction_data.sender_id   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
	    Referee_System_Info->robot_interaction_data.receiver_id = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
	    for(uint8_t i = 0; i< 112; i++){
			Referee_System_Info->robot_interaction_data.user_data[i]=	Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 5 + i];
			}
	 break;
#endif	 

#ifdef MAP_COMMAND_ID
 	 case	MAP_COMMAND_ID:
  		Referee_System_Info->map_command.target_position_x = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]); 
			Referee_System_Info->map_command.target_position_y = bit8TOfloat32(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]); 
      Referee_System_Info->map_command.cmd_keyboard      = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8];
	    Referee_System_Info->map_command.target_robot_id   = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 9];
	    Referee_System_Info->map_command.cmd_source        = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10];
	 break;
#endif	 

#ifdef MAP_ROBOT_DATA_ID
	 case MAP_ROBOT_DATA_ID:
	    Referee_System_Info->map_robot_data.hero_position_x       = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
	    Referee_System_Info->map_robot_data.hero_position_y       = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
	    Referee_System_Info->map_robot_data.engineer_position_x   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4]);
	   	Referee_System_Info->map_robot_data.engineer_position_y   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 6]);
      Referee_System_Info->map_robot_data.infantry_3_position_x = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 8]);
	    Referee_System_Info->map_robot_data.infantry_3_position_y = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 10]);
	    Referee_System_Info->map_robot_data.infantry_4_position_x = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 12]);
	   	Referee_System_Info->map_robot_data.infantry_4_position_y = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 14]);
	    Referee_System_Info->map_robot_data.infantry_5_position_x = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 16]);
	   	Referee_System_Info->map_robot_data.infantry_5_position_y = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 18]);
	    Referee_System_Info->map_robot_data.sentry_position_x     = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 20]);
	   	Referee_System_Info->map_robot_data.sentry_position_y     = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 22]);
	 break;
#endif	 
	 
#ifdef MAP_DATA_ID
	 case MAP_DATA_ID:
      Referee_System_Info->map_data.intention = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];
      Referee_System_Info->map_data.start_position_x = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 1]);
      Referee_System_Info->map_data.start_position_y = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 3]);
      for(uint8_t i = 0;i < 49; i++){
			Referee_System_Info->map_data.delta_x[i] = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 5 + i];
      }
			for(uint8_t i = 0;i < 49; i++){
      Referee_System_Info->map_data.delta_y[i] = Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 54 + i];      
		  }
			Referee_System_Info->map_data.sender_id = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 103]);
	 break;
#endif

#ifdef CUSTOM_INFO_ID
	 case CUSTOM_INFO_ID:
      Referee_System_Info->custom_info.sender_id   = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length]);
			Referee_System_Info->custom_info.receiver_id = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 2]);
			for(uint8_t i = 0;i < 30; i++){
			Referee_System_Info->custom_info.user_data[i]= Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + 4 + i];
      }
	 break;
#endif 	
			
#ifdef custom_controller_ID
	 case custom_controller_ID:
			Zhou = 0;													//调试所用的标志位，持续自增表示数据传输没有问题
			for(uint8_t i = 0;i < 30; i++){
			Referee_System_Info->custom_robot_data.data[i]=Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length + i];
      }
			
			for(uint8_t i=0;i<30;i++)
			{
				custom_data.RX_custom_data[i] = Referee_System_Info->custom_robot_data.data[i];
			}
			
//			if(remote_MyControl_flag == 0)	Control_angle_filter();
	 break;
#endif

#ifdef Keymouse_Control_ID
   case Keymouse_Control_ID:
		 Referee_System_Info->KeyMouse_control.Mouse_X=bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+2]);
		 Referee_System_Info->KeyMouse_control.Mouse_Y=bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+4]);
	   Referee_System_Info->KeyMouse_control.Mouse_Z=bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+6]);
	   Referee_System_Info->KeyMouse_control.Left_button_down=Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+7];
		 Referee_System_Info->KeyMouse_control.Right_button_down=Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+8];
	   Referee_System_Info->KeyMouse_control.keyboard_value_t=Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+9];
		 Referee_System_Info->KeyMouse_control.reserved=Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length+11];
	   //键盘上的按键数据使用共用体，省着我一个一个自己拼了，直接调用KeyBoard_Value_Data这个共用体里面的数据就好。
	   KeyBoard_Value_Data.keyboard_value_data=Referee_System_Info->KeyMouse_control.keyboard_value_t;
	 break;
#endif
	 default:	break;
  }
}


