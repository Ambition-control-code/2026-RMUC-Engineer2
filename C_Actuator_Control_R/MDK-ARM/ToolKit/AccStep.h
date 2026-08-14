#ifndef __ACCSTEP_H__
#define __ACCSTEP_H__

#include "stm32f4xx.h"               // Device header

#define ACC_STEP_NUM 1000

// 加速步数限制
#define MAX_STEP_FULL 334   // 全速最大索引 (AccStep[334] = 497)
#define MAX_STEP_HALF 178   // 半速最大索引 (在数组中约等于248的值对应的位置，可自行微调)

// 双击时间窗口 (根据你的任务执行周期调整)
// 假设该函数每 10ms 调用一次，30 代表 300ms 内连按两次视为双击
#define DOUBLE_CLICK_TICKS 500 

// 按键控制结构体
typedef struct {
    int16_t t_val;          // 当前加速步数
    uint8_t last_state;     // 上一次按键状态 (0:松开, 1:按下)
    uint16_t dc_timer;      // 双击检测计时器
    uint8_t is_full_speed;  // 当前模式 (0:半速, 1:全速)
    uint16_t out_speed;     // 最终输出的速度值
} KeySpeedCtrl_t;

// 实例化6个按键的控制结构体
extern KeySpeedCtrl_t W_ctrl;
extern KeySpeedCtrl_t A_ctrl;
extern KeySpeedCtrl_t S_ctrl;
extern KeySpeedCtrl_t D_ctrl;
extern KeySpeedCtrl_t E_ctrl;
extern KeySpeedCtrl_t Q_ctrl;


void Process_Key_Speed(uint8_t key_pressed, KeySpeedCtrl_t *ctrl);

#endif


