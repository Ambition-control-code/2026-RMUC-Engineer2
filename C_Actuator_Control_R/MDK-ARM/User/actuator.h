#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

#include "stm32f4xx.h"                  	// Device header

#define Big_Arm_Long 40.0f                //大臂臂长（CM）
#define Smoll_Arm_Long 38.0f						 //小臂臂长（CM）
#define Yaw_Mid_Angle 0.0f								 //Yaw轴6020电机的中值编码器值

extern float Smoll_Pitch,Big_Pitch;
extern float Set_Arm_yaw,Set_Arm_roll;
extern float Set_end_Pit,Set_end_roll;
extern float Set_Big_Roll;
extern uint8_t DM_Init_count;

typedef struct
{
	double theta1;  // 大臂与水平面的夹角
	double Last_theta1;
	double theta2;  // 小臂与大臂的夹角
	double Last_theta2;
	double Set_YAW;
	double Set_ROLL;
	float end_roll;
	float end_pitch;
}ARM_Theta_t;
extern ARM_Theta_t ARM_Theta;


//void Think_Target_Pos(ARM_Theta_t* arm_theta_t,double End_X,double End_Y,double set_yaw,double set_roll);
void Arm_Task(void);
void OverLoad_detection(void);
void ARM_Init(void);
#endif


