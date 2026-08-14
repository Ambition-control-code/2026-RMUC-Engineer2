#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

#include "stm32f4xx.h"               // Device header

// 角度结构体 (单位：度)
typedef struct
{
  float d1; // 大臂角度 Degree
  float d2; // 小臂角度 Degree
  float d3; // 手腕角度 Degree
	
	float target_d1;
	float target_d2;
	float target_d3;
}RobotState_Deg;
extern RobotState_Deg robot_deg;

int8_t moveForward_Degree(RobotState_Deg* deg,float dx_m);
int8_t moveUpward_Degree(RobotState_Deg* deg,float dy_m);

#endif


