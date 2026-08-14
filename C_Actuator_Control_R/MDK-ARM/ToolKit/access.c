#include "access.h"
#include "Motor_control.h"
#include "can_bsp.h"
#include "math.h"
#include "remote.h"

extern float Claw_mid_angle;
extern uint8_t change_claw_state;
extern uint8_t Front_control,Behind_control;

// ¤@?¦s¨ú?‰Z¶q
uint8_t TFR_count0 = 0;
uint8_t count1 = 0;
uint8_t F_access_flag = 0,B_access_flag = 0;
extern uint8_t Front_flag,Behind_flag;
extern float Claw_Speed;
float target_Big_Pitch = 0.0f,target_Smoll_Pitch = 0.0f,
        target_End_Pitch = 0.0f,target_yaw = 0.0f, target_bigRoll = 0.0f,
					target_smollRoll = 0.0f,target_endRoll = 0.0f;

uint8_t check_claw = 0,check_flag = 0;;

access_mode_t access_mode = Ready_mode;	//¦s¨úªº¼Ò¦¡?§Ó¦ì

void TFR_conrrol_Front(void)
{
	switch (access_mode)
    {
        case Ready_mode:
        {
            Ready_work_task();
            if(TFR_count0 == 8)
            {
                if(F_access_flag)    access_mode = extract_mode;
                else    access_mode = storage_mode;

                TFR_count0 = 0;
            }
        }   break;
        case storage_mode:
        {
            switch(TFR_count0)
            {
                case 0:
                    // ¤jPitch?¦ì
                    target_Big_Pitch = 93.3f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 1;
                break;
                case 1:
                    // ¤pPitch?¦ì
                    target_Smoll_Pitch = -20.16f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 2;
                break;
                case 2:
                    // EndPitch?¦ì
                    target_End_Pitch = -71.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 3;
                case 3:
                    // Yaw?¦ì
                    target_yaw = -146.56f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);
                    TFR_count0 = 4;
                break;
                case 4:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
					    fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
						    fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f &&
                                    fabs(DM_YAW_back.engle - target_yaw) < 3.0f)
                        TFR_count0 = 5;
                    else TFR_count0 = 0;
                break;
								case 5:
										target_End_Pitch = -71.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
								
										if(fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
										{
											Front_control = 1;
                      osDelay(2500);
											TFR_count0 = 6;
										}
										else	TFR_count0 = 5;
                   
                //ªQ¤ö¤lµM¦Z¥ý¦Z°hÉó±ñÁu
                case 6:
                    
										DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,(Claw_mid_angle-1150.0f),13.0f);
								
                    if(fabs(DM_Claw_back.engle - (Claw_mid_angle-1150.0f)) < 15.0f)
										{
											TFR_count0 = 7;
											New_VT03_data.trigger = !New_VT03_data.trigger;
										}
                    else    TFR_count0 = 6;
                break;
                case 7:
										target_End_Pitch = -49.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 5.0f);
										
										if(fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
											TFR_count0 = 8;
										else
											TFR_count0 = 7;
                break;
                case 8:
                    target_Smoll_Pitch = -4.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
										
										if(fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f)
											TFR_count0 = 9;
										else
											TFR_count0 = 8;
                break;
                case 9:
                    target_Big_Pitch = 74.428f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 10;
                break;
                case 10:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
					    fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
						    fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
                    {
                        Front_control = 0;
                        TFR_count0 = 11;
                    }
                    else    TFR_count0 = 7;
                break;
                case 11:
                    target_yaw = 0.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);
                    
                    if(fabs(DM_YAW_back.engle - target_yaw) < 3.0f)
                    {
                        Front_control = 0;
                        TFR_count0 = 0;
					    TFR_count0 = 0;		// ­«¸m??Éó
					    Front_flag = 0;
					    Behind_flag = 0;
					    target_Big_Pitch = 0.0f;
					    target_Smoll_Pitch = 0.0f;
					    target_End_Pitch = 0.0f;
					    target_yaw = 0.0f;
                        F_access_flag = 1;
                        access_mode = Ready_mode;
											New_VT03_data.New_Z = 0;
                    }
                    else    TFR_count0 = 11;
                break;
            }
        }   break;
        case extract_mode:
        {
            switch(TFR_count0)
            {
                case 0:
										if(check_flag == 0)
										{
											if(fabs(DM_Claw_back.engle - Claw_mid_angle)<100.0f)	check_claw = 0;
											else	check_claw = 1;
											
											check_flag = 1;
										}
                    Front_control = 1;
                    target_Big_Pitch = 81.428f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 1;
                break;
                case 1:
                    target_Smoll_Pitch = -8.975f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 2;
                break;
                case 2:
                    target_End_Pitch = -67.482f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 3;
                break;
                case 3:
										DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,(Claw_mid_angle-1150.0f),13.0f);
									
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f &&
															fabs(DM_Claw_back.engle - (Claw_mid_angle-1150.0f))<15.0f)
                    TFR_count0 = 4;
                    else    TFR_count0 = 0;
                break;
                case 4:
                    target_yaw = -146.56f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);

                    if(fabs(DM_YAW_back.engle - target_yaw) < 5.0f) TFR_count0 = 5;
                    else    TFR_count0 = 4;
                break;
                //«e¦ù?¨ú
                case 5:
                    target_Big_Pitch = 104.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 6;
                break;
                case 6:
                    target_Smoll_Pitch = -16.775f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 7;
                break;
                case 7:
                    target_End_Pitch = -88.31f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 8;
                break;
                case 8:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
                    TFR_count0 = 9;
                    else    TFR_count0 = 5;
                break;
                case 9:
                    DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,Claw_mid_angle,13.0f);

                    if(fabs(DM_Claw_back.engle - Claw_mid_angle) < 15.0f)
                    {
                        Front_control = 0;
                        osDelay(3500);
											
												if(check_claw)	New_VT03_data.trigger = !New_VT03_data.trigger;
											
                        TFR_count0 = 10;
                    }
                    else    TFR_count0 = 9;
                break;
                case 10:
                    target_End_Pitch = -80.31f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 11;
                break;
								case 11:
										target_Smoll_Pitch = -21.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 12;
                break;
								case 12:
										target_Big_Pitch = 92.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
								
										if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
										TFR_count0 = 13;
										else	TFR_count0 = 10;
								break;
                case 13:
                    target_yaw = 0.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);

                    if(fabs(DM_YAW_back.engle - target_yaw) < 5.0f)
                    {
                        TFR_count0 = 0;
					    TFR_count0 = 0;		// ­«¸m??Éó
					    Front_flag = 0;
					    Behind_flag = 0;
					    target_Big_Pitch = 0.0f;
					    target_Smoll_Pitch = 0.0f;
					    target_End_Pitch = 0.0f;
					    target_yaw = 0.0f;
                        F_access_flag = 0;
                        access_mode = Ready_mode;
											New_VT03_data.New_Z = 0;
											check_flag = 0;
                    }
                    else    TFR_count0 = 13;
                break;
            }
        }   break;
        default:    break;
    }
}

void TFR_conrrol_Behind(void)
{
	switch (access_mode)
    {
        case Ready_mode:
        {
            Ready_work_task();
            if(TFR_count0 == 8)
            {
                if(B_access_flag)    access_mode = extract_mode;
                else    access_mode = storage_mode;

                TFR_count0 = 0;
            }
        }   break;
        case storage_mode:
        {
            switch(TFR_count0)
            {
                case 0:
										// EndPitch?¦ì
                    target_End_Pitch = -79.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 1.5f);
                    TFR_count0 = 1;
                break;
                case 1:
                    // ¤pPitch?¦ì
                    target_Smoll_Pitch = -32.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 2;
                break;
                case 2:
                    // ¤jPitch?¦ì
                    target_Big_Pitch = 107.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 0.5f);
                    TFR_count0 = 3;
                case 3:
                    // Yaw?¦ì
                    target_yaw = -157.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);
                    TFR_count0 = 4;
                break;
                case 4:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
					    fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
						    fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f &&
                                    fabs(DM_YAW_back.engle - target_yaw) < 3.0f)
                        TFR_count0 = 5;
                    else TFR_count0 = 0;
                break;
					case 5:
					target_End_Pitch = -79.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 0.5f);
								
										if(fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
										{
											Behind_control = 1;
                                            osDelay(2500);
											TFR_count0 = 6;
										}
										else	TFR_count0 = 5;
                   
                //ªQ¤ö¤lµM¦Z¥ý¦Z°hÉó±ñÁu
                case 6:
                    
					DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,(Claw_mid_angle-1150.0f),13.0f);
								
                    if(fabs(DM_Claw_back.engle - (Claw_mid_angle-1150.0f)) < 15.0f)
					{
						TFR_count0 = 7;
            			New_VT03_data.trigger = !New_VT03_data.trigger;
					}
                    else    TFR_count0 = 6;
                break;
                case 7:
										target_End_Pitch = -70.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.5f);
										if(fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)	TFR_count0 = 8;
                    else	TFR_count0 = 7;
                break;
                case 8:
                    target_Smoll_Pitch = -14.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
								
										if(fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f)	TFR_count0 = 9;
                    else	TFR_count0 = 8;
                break;
                case 9:
                    target_Big_Pitch = 86.428f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 10;
                break;
                case 10:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
					    fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
						    fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
                    {
                        Behind_control = 0;
                        TFR_count0 = 11;
                    }
                    else    TFR_count0 = 7;
                break;
                case 11:
                    target_yaw = 0.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);
                    
                    if(fabs(DM_YAW_back.engle - target_yaw) < 3.0f)
                    {
                        Behind_control = 0;
                        TFR_count0 = 0;
					    TFR_count0 = 0;		// ­«¸m??Éó
					    Front_flag = 0;
					    Behind_flag = 0;
					    target_Big_Pitch = 0.0f;
					    target_Smoll_Pitch = 0.0f;
					    target_End_Pitch = 0.0f;
					    target_yaw = 0.0f;
                        B_access_flag = 1;
                        access_mode = Ready_mode;
											New_VT03_data.New_C = 0;
                    }
                    else    TFR_count0 = 11;
                break;
            }
        }   break;
        case extract_mode:
        {
            switch(TFR_count0)
            {
                case 0:
										if(check_flag == 0)
										{
											if(fabs(DM_Claw_back.engle - Claw_mid_angle)<100.0f)	check_claw = 0;
											else	check_claw = 1;
											
											check_flag = 1;
										}
                    Behind_control = 1;
                    target_Big_Pitch = 91.128f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 1;
                break;
                case 1:
                    target_Smoll_Pitch = -20.815f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 2;
                break;
                case 2:
                    target_End_Pitch = -75.482f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 3;
                break;
                case 3:
										DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,(Claw_mid_angle-1150.0f),13.0f);
									
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f &&
															fabs(DM_Claw_back.engle - (Claw_mid_angle-1150.0f))<15.0f)
                    TFR_count0 = 4;
                    else    TFR_count0 = 0;
                break;
                case 4:
                    target_yaw = -157.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);

                    if(fabs(DM_YAW_back.engle - target_yaw) < 5.0f) TFR_count0 = 5;
                    else    TFR_count0 = 4;
                break;
                //«e¦ù?¨ú
                case 5:
										target_End_Pitch = -68.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 6;
                break;
                case 6:
                    target_Smoll_Pitch = -34.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 7;
                break;
                case 7:
                    target_Big_Pitch = 108.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
                    TFR_count0 = 8;
                break;
                case 8:
                    if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
                    TFR_count0 = 9;
                    else    TFR_count0 = 5;
                break;
                case 9:
                    DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,Claw_mid_angle,13.0f);

                    if(fabs(DM_Claw_back.engle - Claw_mid_angle) < 15.0f)
                    {
                        Behind_control = 0;
                        osDelay(3500);
											
												if(check_claw)	New_VT03_data.trigger = !New_VT03_data.trigger;
											
                        TFR_count0 = 10;
                    }
                    else    TFR_count0 = 9;
                break;
                case 10:
                    target_End_Pitch = -71.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 3.0f);
                    TFR_count0 = 11;
                break;
								case 11:
										target_Smoll_Pitch = -42.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
                    TFR_count0 = 12;
                break;
								case 12:
										target_Big_Pitch = 92.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 1.0f);
								
										if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
                        fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
                            fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f)
										TFR_count0 = 13;
										else	TFR_count0 = 10;
								break;
                case 13:
                    target_yaw = 0.0f;
                    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.5f);

                    if(fabs(DM_YAW_back.engle - target_yaw) < 5.0f)
                    {
                        TFR_count0 = 0;
					    TFR_count0 = 0;		// ­«¸m??Éó
					    Front_flag = 0;
					    Behind_flag = 0;
					    target_Big_Pitch = 0.0f;
					    target_Smoll_Pitch = 0.0f;
					    target_End_Pitch = 0.0f;
					    target_yaw = 0.0f;
                        B_access_flag = 0;
                        access_mode = Ready_mode;
											New_VT03_data.New_C = 0;
											check_flag = 0;
                    }
                    else    TFR_count0 = 13;
                break;
            }
        }   break;
        default:    break;
    }
}


void Ready_work_task(void)
{
    switch(TFR_count0)
    {
        case 0:
            // ¤jPitch?¦ì
            target_Big_Pitch = 70.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigPitch, target_Big_Pitch, 0.5f);
            TFR_count0 = 1;
        break;
        case 1:
            // ¤pPitch?¦ì
            target_Smoll_Pitch = -70.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollPitch, target_Smoll_Pitch, 35.0f);
            TFR_count0 = 2;
        break;
        case 2:
            // EndPitch?¦ì
            target_End_Pitch = 0.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndPitch, target_End_Pitch, 1.0f);
            TFR_count0 = 3;
        break;
        case 3:
            // Yaw?¦ì
            target_yaw = 0.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, target_yaw, 1.0f);
            TFR_count0 = 4;
        break;
        case 4:
            //¤pRoll?¦ì
            target_smollRoll = 0.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_SmollRoll, target_smollRoll, 3.0f);
            TFR_count0 = 5;
        break;
        case 5:
            //¤jRoll?¦ì
            target_bigRoll = 0.0f;
            DM_Pos_Speed_Control(Control_Motor_hcan, Motor_BigRoll, target_bigRoll, 35.0f);
            TFR_count0 = 6;
        break;
				case 6:
						target_endRoll = 0.0f;
						DM_Pos_Speed_Control(Control_Motor_hcan, Motor_EndRoll, target_endRoll, 3.0f);
						TFR_count0 = 7;
        case 7:
            if(fabs(Big_Pitch_back.engle - target_Big_Pitch) < 3.0f && 
					fabs(Smoll_Pitch_back.engle - target_Smoll_Pitch) < 3.0f && 
						fabs(DM_endPitch_back.engle - target_End_Pitch) < 3.0f &&
                            fabs(DM_ROLL_back.engle - target_smollRoll) < 3.0f &&
                                fabs(DM_YAW_back.engle - target_yaw) < 2.0f &&
                                    fabs(Big_Roll_back.engle - target_bigRoll) < 3.0f &&
																			fabs(DM_endRoll_back.engle - target_endRoll) < 3.0f)
			TFR_count0 = 8;
            else TFR_count0 = 0;
        break;
    }
}



