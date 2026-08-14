#ifndef __CONTROLLER_DATA_H__
#define __CONTROLLER_DATA_H__

#include "stm32f4xx.h"                  // Device header

#define FRAME_HEADER_LENGTH 5 		// 帧头数据长度
#define CMD_ID_LENGTH 2       		// 命令码ID数据长度
#define DATA_LENGTH 30        		// 数据段长度
#define FRAME_TAIL_LENGTH 2   		// 帧尾数据长度
#define HAL_MAX_DELAY      0xFFFFFFFFU

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define Arm_CMD_ID 0x0309			//向自定义控制器发送数据的命令码

#define MAX_retry_count 3

//-----------将裁判系统发来的30字节自定义控制器数据解析后存储下面结构体中，名字可以随着需求修改，这只是第一版----------------//
//	typedef struct {
//	float Control01_data;
//	float Control02_data;
//	float Control03_data;
//	float Control04_data;
//	float Control05_data;
//	float Control06_data;
//	float Control07_data;
//	float Control08_data;
//	float Control09_data;
//	float Control10_data;
//	float Control11_data;
//	float Control12_data;
//	float Control13_data;
//	float Control14_data;
//	float Control15_data;
//	}Controller_Data_T;
//	extern Controller_Data_T Controller_Data;
//》》》》》》》》》》》》》上面是早期处理自控数据的方法，精度不高，现改用共用体的方法来存储自控的数据《《《《《《《《《《《《《《《《《

//==================以下是共用体处理自控的数据，处理数据和自控端数据对上精度较高===============================//
#pragma pack(push, 1)
typedef union
{
	struct
	{
		float Smoll_Pitch_data;      	// 自控小臂Pitch轴数据
		float control_yaw;       		// 自控C板yaw轴数据
		float Big_Pitch_data;        	// 自控大疆3508屁股控制大Pitch数据
		float Roll_3508_data;   		// 自控大疆3508屁股控制roll数据
		float endPitch_data;			// 自控末端Pitch轴3508屁股的数据
		float endRoll_data;			// 自控末端Roll轴3508屁股的数据
		float BigRoll_data;			// 自控大臂Roll轴3508屁股的数据
		uint8_t Claw_flag:1;				// 自控夹取的微动标志位
		uint8_t Front_flag:1;			// 自控左面快捷存储的标志位
		uint8_t	Behind_flag:1;			// 自控右面快捷存储的标志位
		uint8_t Mid_flag:1;				// 自控中间标志位
		uint8_t data1:4;				// 自控预留数据位1
		uint8_t data2;					// 自控预留数据位2
	}Custom_Data;
	uint8_t RX_custom_data[30];
}custom_data_union;
#pragma pack(pop)
extern custom_data_union custom_data;
//==========================================================================

//==========================================================================
typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[30];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
}Controller_t;                    // 自定义控制器数据包，这是裁判系统要的所有数据，包括帧头帧尾校验等
extern Controller_t Arm_tx_data;

typedef union
{
	struct
	{
		float M1_Yaw;
		float M2_Big_Pitch;
		float M3_Smoll_Pitch;
		float M4_Big_Roll;
		float M5_End_Pitch;
		float M6_End_Roll;
		uint8_t yaw_joint1:1;
		uint8_t Big_pitch:1;
		uint8_t Smoll_pitch:1;
		uint8_t Big_roll:1;
		uint8_t end_pitch:1;
		uint8_t	end_roll:1;
		uint8_t data1:1;
		uint8_t	data2:1;
		uint8_t data3;
		uint8_t data4;
		uint8_t data5;
		uint8_t data6;
		uint8_t data7;
	}Arm_data_t;
	uint8_t TX_Arm_data[30];
}Arm_union_data_t;
extern Arm_union_data_t Arm_union_data;
//=====================================================================================================

//=====================================================================================================
typedef struct{
	uint8_t yaw;
	uint8_t Big_pitch;
	uint8_t Smoll_pitch;
	uint8_t Big_roll;
	uint8_t end_pitch;
	uint8_t	end_roll;
	uint8_t smoll_roll;
	uint8_t data0:1;
	uint8_t data1:1;
	uint8_t data2:1;
	uint8_t data3:1;
	uint8_t data4:1;
	uint8_t data5:1;
	uint8_t data6:1;
	uint8_t data7:1;
}Joint_collision_flag_t;
extern Joint_collision_flag_t Joint_collision_flag;
//=====================================================================================================

void Controller_Data_translate(void);
void DM10010L_Control_task(void);
void DM8009_xPitch_Control_task(void);
void DM4340_Control_task(void);
void DM10010L_xYaw_Control_task(void);
void End_effector_MYcontrol(void);
void DM4340_endPitch_Control(void);
void DM4310_endRoll_Control(void);
void DM4310_claw_Control(void);
void DM4340_BigRoll_Control(void);

void Control_angle_filter(void);

void MY_UART6_DMA_TX_Init(void);
void Data_to_Control(void);
void Collision_detection(Joint_collision_flag_t* flag_struct);
void ui_Date_sent(uint8_t yaw,uint8_t big_pitch,uint8_t smoll_pitch,uint8_t big_roll,
															uint8_t end_pitch,uint8_t end_roll,uint8_t smoll_roll);

#endif


