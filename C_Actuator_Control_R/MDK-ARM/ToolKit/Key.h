#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"               // Device header

typedef struct {
    uint8_t last_state;     // 上次物理状态
    uint32_t press_tick;    // 按下时刻的时间戳
} ButtonState_t;

extern ButtonState_t Key_Pause;
extern ButtonState_t Key_trigger;
extern ButtonState_t Key_fn1;
extern ButtonState_t Key_fn2;
extern ButtonState_t Key_R;
extern ButtonState_t Key_C;
extern ButtonState_t Key_Z;
extern ButtonState_t Key_F;
extern ButtonState_t Key_mouse_left;

extern ButtonState_t Key_Front_flag;
extern ButtonState_t Key_Behind_flag;

extern ButtonState_t Key_Claw_flag;

extern ButtonState_t Key_fn1_flag;
extern ButtonState_t Key_fn2_flag;

uint8_t Button_Check_Event(uint8_t current_state, ButtonState_t *btn);

#endif


