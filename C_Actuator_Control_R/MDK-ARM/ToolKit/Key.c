#include "Key.h"
#include "stm32f4xx_hal.h"

/**
 *@role:	一个检测按键状态的小函数
**/
ButtonState_t Key_Pause = {0};
ButtonState_t Key_trigger = {0};
ButtonState_t Key_fn1 = {0};
ButtonState_t Key_fn2 = {0};
ButtonState_t Key_R = {0};
ButtonState_t Key_C = {0};
ButtonState_t Key_Z = {0};
ButtonState_t Key_F = {0};
ButtonState_t Key_mouse_left = {0};

ButtonState_t Key_Front_flag = {0};
ButtonState_t Key_Behind_flag = {0};

ButtonState_t Key_Claw_flag = {0};

ButtonState_t Key_fn1_flag = {0};
ButtonState_t Key_fn2_flag = {0};

uint8_t Button_Check_Event(uint8_t current_state, ButtonState_t *btn)
{
	if(current_state != btn->last_state) 
	{
		uint32_t now = HAL_GetTick();
		uint8_t event = 0;

		if(current_state == 1)	btn->press_tick = now;
		else 
		{
		if(now - btn->press_tick > 15)	event = 1; // 记录发生了一次有效点击
		}
		btn->last_state = current_state;
		return event; // 返回 1 表示要翻转，返回 0 表示不变
	}
	return 0;
}


