#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#define RATE_BUF_SIZE 6

//-----------达妙电机的内容-------------------//
#define P_MIN -12.5	  //位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值
//------达妙电机三个模式，模式ID。------------//
#define DM_MIT_MODE 	0x000			//MIT模式
#define DM_POS_MODE		0x100			//位置速度模式
#define DM_SPEED_MODE	0x200			//速度模式
//------本末电机五个模式，模式ID。------------//
#define BM_U_MODE			0x00			//电压环控制模式，这个也就是开环
#define BM_I_MODE		  0x20			//电流环控制模式
#define BM_v_MODE		  0x30			//速度环控制模式
#define BM_P_MODE		  0x40			//位置环控制模式
//--------------------------------------------//
/*包含头文件****begain！！！*/
#include "stm32f4xx.h"                   // Device header
/*包含头文件******end！！！**/
//----------------------------------------

//---------------------------------------//
extern int8_t motor_id;
extern uint8_t RX_data[8];
//---------------------------------------//

//---------------------------------------------------------------------------------
/******begain！！！*********结构体***********/

typedef struct{
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;				 //上一次的编码器原始值
	int32_t ecd_value;             //经过处理后连续的编码器值
	int32_t diff;							     //两次编码器之间的差值
	int32_t temp_count;            //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						   //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t rate_buf[6];	         //buf，for filter（滤波用的）
	int32_t round_cnt;						 //圈数
	int32_t filter_rate;					 //速度（计算出来的，不要用，有误差，实时性不好）
	int32_t filter_rate_max;			 //最大速度
	float ecd_angle;						   //角度
	int16_t current;               //电流
	float re_speed;                //电调速度
	int8_t tempature;	             //温度
	int state;  								   //电机状态（如：错误，就绪等）
	float Real_Angle;							 //真实角度
}Motor_Feedback;								 //大疆电机使用这个结构体，达妙的一拖四模式也可以用这个结构体



typedef struct{
	int id;
	int state;								 
	float vel;										 //电机实际转速
	float tor;										 //电机扭矩 
	float Tmos;
	float Tcoil;
	float engle;									 //电机实际角度
	int32_t round_cnt;						 //圈数
	float last_raw_pos;				     //上一次的角度值
	float diff;							       //两次角度之间的差值
	float raw_pos;   					     //未经计圈数处理过的角度值
}DM_Normol_Feedback;
extern DM_Normol_Feedback Big_Pitch_back;
extern DM_Normol_Feedback Big_Roll_back;
extern DM_Normol_Feedback Smoll_Pitch_back;
extern DM_Normol_Feedback DM_YAW_back;
extern DM_Normol_Feedback DM_ROLL_back;
extern DM_Normol_Feedback DM_endPitch_back;
extern DM_Normol_Feedback DM_endRoll_back;
extern DM_Normol_Feedback DM_Claw_back;
extern DM_Normol_Feedback DM_riseMotor_back;

typedef struct{
	float v_speed;							 //电机的中心轴转速
	float I_electric;						 //电机实际反馈的电流值
	float last_angle;							 //上一次的电机角度值
	float Real_Angle;							 //真实角度【0~360】
	float ecd_angle;							 //处理后的角度【-∞~+∞】
	float diff;										 //电机前后两次角度的差值
	float U_voltage;						 //系统母线电压
	int32_t round_cnt;						 //电机圈数
	float torque;									//力矩
}BM_Feedback;
/******end！！！***********结构体***********************/

//---------------------------------------------------------------------------------

/***********函数声明****begain！！！********/
void Motor_control(CAN_HandleTypeDef *hcan,uint16_t Control_ID,int16_t Speed,uint8_t esc_id);
void Motor_enable(CAN_HandleTypeDef *hcan,uint16_t Control_ID);

void get_dJi_Motor_Data(volatile Motor_Feedback *feedback);

void get_DM_OTF_Data(volatile Motor_Feedback *feedback);
void DM_Motor_Mode_enable(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id);
void DM_MIT_Control(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);
void DM_Normal_GetData(DM_Normol_Feedback* DM_Normal_feedback);
void DM_Speed_Control(CAN_HandleTypeDef* hcan,uint16_t ID,float _vel);
void DM_Pos_Speed_Control(CAN_HandleTypeDef* hcan, uint16_t ID, float pos, float vel);
void DM_save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t mode_id);
void DM_Motor_Mode_disable(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id);
void DM_Motor_clean_erro(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id);

void BM_Motor_enable(CAN_HandleTypeDef* hcan,uint8_t ID);
void BM_Motor_disenable(CAN_HandleTypeDef* hcan,uint8_t ID);
void get_BM_Motor_data(BM_Feedback* BM_feedback);
void BM_Motor_ID_change(CAN_HandleTypeDef* hcan,uint8_t Now_ID,uint8_t target_ID);
void BM_Motor_mode_change(CAN_HandleTypeDef* hcan,uint8_t Now_ID,uint8_t BM_mode);
void BM_Motor_zero_save(CAN_HandleTypeDef* hcan,uint8_t Now_ID);
void BM_Motor_flash_save(CAN_HandleTypeDef* hcan);
/***********函数声明****end！！！**********/

#endif


