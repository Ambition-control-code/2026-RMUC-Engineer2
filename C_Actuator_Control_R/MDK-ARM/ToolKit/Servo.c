#include "Servo.h"
#include "tim.h"

void Set_Servo_Angle(float angle1,float angle2)//上场使用的是270度的舵机
{
	// 限幅保护，防止代码异常导致舵机内部齿轮扫齿
	if(angle1 < 0.0f) angle1 = 0.0f;
	if(angle1 > 180.0f) angle1 = 180.0f;
	
	if(angle2 < 0.0f) angle2 = 0.0f;
	if(angle2 > 180.0f) angle2 = 180.0f;
	
	// 计算脉宽：0度对应500us，180度对应2500us
	// 因为前面配置的定时器1 Tick就是1us，所以算出来的脉宽直接就是寄存器的值
	uint32_t pulse1 = (uint32_t)(500.0f + (angle1 / 180.0f) * 2000.0f);
	uint32_t pulse2 = (uint32_t)(500.0f + (angle2 / 180.0f) * 2000.0f);
	
	// 修改 TIM1 通道 1 的占空比 (如果用的是其他通道，请修改 TIM_CHANNEL_1)
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pulse2);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pulse1);
}


