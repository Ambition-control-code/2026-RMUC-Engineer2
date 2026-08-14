#ifndef __REMOTE_H__
#define __REMOTE_H__

/*包含头文件****begain！！！*/
#include "stm32f4xx.h"                  // Device header
#include "struct_typedef.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"  // 包含标准整数类型定义
#include "stdbool.h"
#include "math.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "can_bsp.h"
#include "can_driver.h"
#include "stdarg.h"
/*包含头文件****end！！！****/

/*宏定义********begain!*/
#define DMA_RX_BUF_SIZE 18		//DMA接收缓冲区大小
#define keypad_K 16.5f
#define remote_K 12.5
#define limit 8192

#define Claw_open_speed 1.0f
#define Claw_close_speed -1.0f
#define Claw_stop_speed 0.0f

/*宏定义********end!****/

/***@struct： 结构体****/
typedef struct
{
	int16_t R_x;                     //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	                   //右手y    上1684 下364 中1024
	int16_t L_x;                     //左手x
	int16_t L_y;                     //右手y
	uint8_t sl;		                   //拨动左  上1 中3 下2（左拨杆）
	uint8_t sr;                      //右拨杆
	uint8_t sl_change_flag;			     //拨动左  上1 中3 下2     //拨动值异常检测标志
	uint8_t sr_change_flag;				   //拨动值异常检测标志
	uint8_t mr_change_flag;					 //拨动值异常检测标志
	uint8_t ml_change_flag;					 //拨动值异常检测标志
	int16_t mouse_x;	               //鼠标x轴
	int16_t mouse_y;                 //鼠标y轴
	int16_t mouse_z;                 //鼠标滚轮
	uint8_t mouse_l;	               //鼠标左键
	uint8_t mouse_r;                 //鼠标右键
	uint16_t key;		                 //键盘
	u8 W;                            //W键
	u8 S;                            //S键
	u8 A;                            //A键
	u8 D;                            //D键
	u8 Shift;                        //Shift键
	u8 Ctrl;                         //Ctrl键
	u8 Q;                            //Q键
	u8 E;                            //E键
	u8 R;                            //R键
	u8 F;                            //F键
	u8 G;                            //G键
	u8 Z;                            //Z键
	u8 X;                            //X键
	u8 C;                            //C键
	u8 V;                            //V键
	u8 B;                            //B键
	uint8_t last_sr;
	uint8_t last_sl;
	uint8_t last_mouse_l;
	uint8_t last_mouse_r;
	uint8_t last_R;
	int16_t trigger;                //遥控侧边拨轮
	uint8_t trigger_flag_down;			//遥控器标志位
	uint8_t trigger_flag_up;				//遥控器标志位
}Remote_all;
extern Remote_all remote;

typedef __packed struct
{
  uint8_t sof_1;								//帧头1
  uint8_t sof_2;								//帧头2
  uint64_t ch_0:11;							//右摇杆水平方向的位置【最小值364，中间值1024，最大值1684】
  uint64_t ch_1:11;							//右摇杆竖直方向的位置【最小值364，中间值1024，最大值1684】
  uint64_t ch_2:11;							//左摇杆竖直方向的位置【最小值364，中间值1024，最大值1684】
  uint64_t ch_3:11;							//左摇杆水平方向的位置【最小值364，中间值1024，最大值1684】
  uint64_t mode_sw:2;						//挡位切换开关位置【C:0	N:1	S:2】
  uint64_t pause:1;							//暂停按键是否按下【未按下是0，按下是1】
  uint64_t fn_1:1;							//左侧自定义按键是否按下【按下是1，未按下是0】
  uint64_t fn_2:1;							//右侧自定义按键是否按下【按下是1，未按下是0】
  uint64_t wheel:11;						//左上角拨轮【最小值364，中间值1024，最大值1684】
  uint64_t trigger:1;						//右上角扳机键【按下是1，未按下是0】

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left:2;
  uint8_t mouse_right:2;
  uint8_t mouse_middle:2;
  uint8_t without:2;
	uint16_t W:1;
	uint16_t S:1;
	uint16_t A:1;
	uint16_t D:1;
	uint16_t SHIFT:1;
	uint16_t CTRL:1;
	uint16_t Q:1;
	uint16_t E:1;
	uint16_t R:1;
	uint16_t F:1;
	uint16_t G:1;
	uint16_t Z:1;
	uint16_t X:1;
	uint16_t C:1;
	uint16_t V:1;
	uint16_t B:1;
  uint16_t crc16;
}VT13_data;									//新遥控器VT03的数据结构体

typedef union
{
	VT13_data use_data;
	uint8_t  temp_data[21];
}VT13_USE;
extern VT13_USE VT13_use;

typedef struct
{
	int16_t R_rocker_x;
	int16_t R_rocker_y;
	int16_t L_rocker_x;
	int16_t L_rocker_y;
	int16_t wheel;
	uint8_t mode_sw;
	uint8_t pause;
	uint8_t fn_1;
	uint8_t fn_2;
	uint8_t trigger;
	uint8_t New_R;
	uint8_t New_C;
	uint8_t New_Z;
	uint8_t New_V;
	uint8_t New_F;
	uint8_t New_X;
	uint8_t shift;
	uint8_t mouse_left;
	//以上是遥控器按键的处理，要处理键鼠的按键再后期自行增加
}New_VT03_data_T;
extern New_VT03_data_T New_VT03_data;
/***********************/

/***@role： 别处定义的变量*******/
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern u8 dbus_recieve_1[36],dbus_recieve_2[36];
extern float deal_W,deal_A,deal_S,deal_D,deal_E,deal_Q,deal_R,deal_F,deal_mouse_r,deal_mouse_l;
extern float deal_control_L,deal_control_R;
extern uint8_t remote_MyControl_flag;
extern uint8_t Chassis_or_Arm_or_End;
extern uint8_t last_storage_flag1,last_storage_flag2;
/****************************/

/***@role： 函数的声明**********/
void MY_DMA_Init(void);
u8 if_change(u8 flag,u8 last_flag);
void deal_dbus(u8 *RXBuff);
void remote_fifo(void);
void New_Remote_fifo(void);
void Chassis_Date_sent(int16_t x,int16_t y,int16_t turn,uint8_t Stop,uint8_t erro);
void Flag_Inspect_task(void);
void Remote_To_Chassis(void);
void USART3_IRQHandler(void);
void Handle_VT03_Data(New_VT03_data_T* new_VT03_data_t);
void remote_save_control(void);
void Turn_Date_sent(float turn);
/****************************/

#endif


