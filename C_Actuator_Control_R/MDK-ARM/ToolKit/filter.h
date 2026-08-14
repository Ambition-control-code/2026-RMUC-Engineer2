#ifndef __FILTER_H__
#define __FILTER_H__

#include "stm32f4xx.h"                  // Device header

//滑动窗口滤波的结构体
typedef struct
{
	uint8_t len;   			//窗口长度
	uint8_t index; 			//当前写入位置的索引，也就是当前指针，使用时赋零就好
	float buf[50];			//缓冲区，最大窗口数是50
}WindowFilter_Parameter;
extern WindowFilter_Parameter WindowFilter_L_current;
extern WindowFilter_Parameter WindowFilter_R_current;
extern WindowFilter_Parameter WindowFilter_pitch;

extern WindowFilter_Parameter WindowFilter_BigPitch;
extern WindowFilter_Parameter WindowFilter_Yaw;
extern WindowFilter_Parameter WindowFilter_SmollPitch;
extern WindowFilter_Parameter WindowFilter_BigRoll;
extern WindowFilter_Parameter WindowFilter_EndPitch;
extern WindowFilter_Parameter WindowFilter_EndRoll;
extern WindowFilter_Parameter WindowFilter_SmollRoll;

// 卡尔曼滤波结构体
typedef struct 
{
  float q;        		// 过程噪声协方差
  float r;        		// 测量噪声协方差
  float x;        		// 系统状态估计值
  float p;        		// 估计误差协方差
  float k;        		// 卡尔曼增益，也就是output
}KalmanFilter_Parameter;

//一阶低通滤波的结构体
typedef struct
{
	float Input;				//当前输入值
	float Output;				//当前输出值
	float Last_Output;	//上一次的输出值
}LowpassFilter_Parameter;
extern LowpassFilter_Parameter Filter_Claw_angle;

//微分追踪器的结构体
typedef __packed struct
{
    float Input;			//当前的输入量

    float h0;					// 滤波因子（决定平滑度，越大越平滑但延迟越大）
    float r;					// 快速因子，决定跟踪速度

    float x;					// 跟踪输出（滤波后的位置信号）
    float dx;					// 一阶微分（速度估计）
    float ddx;				// 二阶微分（加速度估计）

    float last_dx;		// 上一时刻的速度
    float last_ddx;		// 上一时刻的加速度

    uint32_t DWT_CNT;	// 时间戳，可用于精确计时
    float dt;					// 实际时间间隔（应由DWT_CNT计算得出）
} TD_t;

float WindowFilter(WindowFilter_Parameter *Window_Struct,float inPut);
void kalman_init(KalmanFilter_Parameter* Kalman_Struct, float q, float r, float init_x, float init_p);
float kalman_update(KalmanFilter_Parameter* Kalman_Struct, float input);
float Lowpass_Filter(LowpassFilter_Parameter* Lowpass_Struct,float input,float alpha,float dead);
float Slope_Buffer(float Set_Angle,float real_Angle,float Turn_Speed);

#endif


