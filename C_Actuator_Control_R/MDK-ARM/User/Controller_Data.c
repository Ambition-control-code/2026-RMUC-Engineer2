//控制器的代码文件
#include "Controller_Data.h"
#include "PID.h"
#include "Motor_control.h"
#include "can.h"
#include "Get_Referee_Data.h"
#include "string.h"
#include "remote.h"
#include "filter.h"
//--------------------------------------------------------------------------------------------------------------

/***@role: 大Pitch关节电机10010L控制函数***/
//float xxxx = 0.5f;
void DM10010L_Control_task(void)
{
	if(Big_Pitch    >  100.0f)	Big_Pitch =  100.0f;
	if(Big_Pitch    <  0.5f)	Big_Pitch =  0.5f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_BigPitch,Big_Pitch,1.0f);
}

/***@role: 小Pitch关节电机4340控制函数*****/
void DM8009_xPitch_Control_task(void)
{
	if(Smoll_Pitch  <-135.0f)	Smoll_Pitch = -135.0f;
	if(Smoll_Pitch  >  0.0f)	Smoll_Pitch =  0.0f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_SmollPitch,Smoll_Pitch,35.0f);
}

/***@role: 大Roll关节电机4310控制函数******/
void DM4340_Control_task(void)
{
	if(Set_Arm_roll >  165.0f)	Set_Arm_roll =  165.0f;
	if(Set_Arm_roll < -90.0f )	Set_Arm_roll = -90.0f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_SmollRoll,Set_Arm_roll,5.0f);
}

/***@role: 小Yaw关节电机4340控制函数******/
uint8_t check_yawflag = 0;

float base_yaw_input = 0.0f;  // 记录模式切换瞬间，遥控器的原始输入值
float base_yaw_output = 0.0f; // 记录模式切换瞬间，电机原本的限幅输出值值

void DM10010L_xYaw_Control_task(void)
{
	
	float final_target = 0.0f;
  float current_raw_input = Set_Arm_yaw;
	
	 if (Big_Pitch_back.engle < 60.0f)
    {
        // --- 模式 A：严格限幅模式 [-10, 3] ---
        check_yawflag = 0; // 重置标志位
        
        // 计算限幅后的目标值
        final_target = current_raw_input;
        if (final_target > 3.0f)  final_target = 3.0f;
        if (final_target < -10.0f) final_target = -10.0f;
    }
    else
    {
        // --- 模式 B：宽泛范围模式 [-155, 30] ---
        if (check_yawflag == 0)
        {
            // 【关键】在进入模式 B 的那一瞬间，锁存当前的状态
            base_yaw_input = current_raw_input; // 记录此刻摇杆在哪
            
            // 计算切换瞬间的输出值（即模式 A 的最后一次有效输出）
            float last_clamped = current_raw_input;
            if (last_clamped > 3.0f)  last_clamped = 3.0f;
            if (last_clamped < -10.0f) last_clamped = -10.0f;
            base_yaw_output = last_clamped; 
            
            check_yawflag = 1;
        }

        // 基于“切换瞬间的输出值”进行偏移计算
        // 公式：当前目标 = 切换瞬间的输出值 + (当前摇杆值 - 切换瞬间的摇杆值)
        final_target = base_yaw_output + (current_raw_input - base_yaw_input);

        // 宽泛范围的二次限幅
        if (final_target > 30.0f)   final_target = 30.0f;
        if (final_target < -155.0f) final_target = -155.0f;
    }

    // 执行控制
    DM_Pos_Speed_Control(Control_Motor_hcan, Motor_Yaw, -final_target, 1.0f);
}

/***@role:	末端Pitch轴关节电机4340控制函数***/
void DM4340_endPitch_Control(void)
{
	if(Set_end_Pit  >  125.0f)	Set_end_Pit  =  125.0f;
	if(Set_end_Pit  < -115.0f)	Set_end_Pit  = -115.0f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_EndPitch,Set_end_Pit,5.0f);
}

/***@role:	末端Roll轴关节电机4310控制函数***/
void DM4310_endRoll_Control(void)
{
	if(Set_end_roll >  110.0f)	Set_end_roll =  110.0f;
	if(Set_end_roll < -130.0f)	Set_end_roll = -130.0f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_EndRoll,Set_end_roll,5.0f);
}

void DM4340_BigRoll_Control(void)
{
	if(Set_Big_Roll >  85.0f)	Set_Big_Roll =  85.0f;
	if(Set_Big_Roll < -75.0f)	Set_Big_Roll = -75.0f;
	
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_BigRoll,Set_Big_Roll,35.0f);
}

/***@role:	末端夹爪关节电机4310控制函数***/
extern osMessageQueueId_t Claw_flagHandle;
extern float Claw_mid_angle;
extern float Claw_Speed;
//正左转，负右转，右转向回收，左转是向出吐
void DM4310_claw_Control(void)
{
	DM_Pos_Speed_Control(Control_Motor_hcan,Motor_Claw,Claw_Speed,15.0f);
}



//自控数据滤波函数
void Control_angle_filter(void)
{
	Big_Pitch = WindowFilter(&WindowFilter_BigPitch,custom_data.Custom_Data.Big_Pitch_data);
	Smoll_Pitch = (-WindowFilter(&WindowFilter_SmollPitch,custom_data.Custom_Data.Smoll_Pitch_data));
	Set_Arm_roll = (-WindowFilter(&WindowFilter_SmollRoll,custom_data.Custom_Data.Roll_3508_data));
	Set_Arm_yaw = WindowFilter(&WindowFilter_Yaw,custom_data.Custom_Data.control_yaw);
	Set_end_Pit = (-WindowFilter(&WindowFilter_EndPitch,custom_data.Custom_Data.endPitch_data));
	Set_end_roll = (WindowFilter(&WindowFilter_EndRoll,custom_data.Custom_Data.endRoll_data));
	Set_Big_Roll = WindowFilter(&WindowFilter_BigRoll,custom_data.Custom_Data.BigRoll_data);
}



//=========================================以下是要发送给自控的任务代码==================================================

/***@role: UART6的DMA发送的重新初始化函数，用于DMA初次初始化出问题重新初始化串口时使用***/
extern DMA_HandleTypeDef hdma_usart6_tx;
void MY_UART6_DMA_TX_Init(void)
{
	__HAL_DMA_DISABLE(&hdma_usart6_tx);																											// 确保DMA处于禁用状态进行配置
	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)	{__HAL_DMA_DISABLE(&hdma_usart6_tx);}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6 | DMA_HISR_HTIF6 | DMA_HISR_TEIF6);// 清除所有DMA标志
	hdma_usart6_tx.Instance->PAR = (uint32_t)&(USART6->DR);  					 											// 发送数据寄存		// 配置DMA参数
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, 39u);                       											// 设置传输长度（39个字节）
  __HAL_DMA_ENABLE_IT(&hdma_usart6_tx, DMA_IT_TC);																				// 启用DMA传输完成中断（可选，用于通知发送完成）
  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);																					// 启用UART的DMA发送请求
}

/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
Controller_t Arm_tx_data = {0};
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
	static uint8_t seq = 0;
	// 帧头数据
	Arm_tx_data.frame_header.sof = 0xA5;                              			// 数据帧起始字节，固定值为 0xA5
	Arm_tx_data.frame_header.data_length = data_lenth;                			// 数据帧中数据段的长度
	Arm_tx_data.frame_header.seq = seq++;                             			// 包序号
	append_CRC8_check_sum((uint8_t *)(&Arm_tx_data.frame_header), 5); 			// 添加帧头 CRC8 校验位
	// 命令码ID
	Arm_tx_data.cmd_id = Arm_CMD_ID;																	      // 图传链路自定义控制器命令码
	// 数据段
	memcpy(Arm_tx_data.data, data, data_lenth);															// 有用的数据搬运
	// 帧尾CRC16，整包校验
	append_CRC16_check_sum((uint8_t *)(&Arm_tx_data), DATA_FRAME_LENGTH);		// CRC16帧尾校验
}

/***@role:	将机械臂关节角度回传给自控的函数***/
Arm_union_data_t Arm_union_data = {0};
uint8_t retry_count = 0;
void Data_to_Control(void)
{
	Arm_union_data.Arm_data_t.M1_Yaw = DM_YAW_back.engle;
	Arm_union_data.Arm_data_t.M2_Big_Pitch = Big_Pitch_back.engle;
	Arm_union_data.Arm_data_t.M3_Smoll_Pitch = Smoll_Pitch_back.engle;
	Arm_union_data.Arm_data_t.M4_Big_Roll = DM_ROLL_back.engle;
	Arm_union_data.Arm_data_t.M5_End_Pitch = 0.0f;
	Arm_union_data.Arm_data_t.M6_End_Roll = 0.0f;
	
	Arm_union_data.Arm_data_t.yaw_joint1 = Joint_collision_flag.yaw;
	Arm_union_data.Arm_data_t.Big_pitch = Joint_collision_flag.Big_pitch;
	Arm_union_data.Arm_data_t.Smoll_pitch = Joint_collision_flag.Smoll_pitch;
	Arm_union_data.Arm_data_t.Big_roll = Joint_collision_flag.Big_roll;
	Arm_union_data.Arm_data_t.end_pitch = Joint_collision_flag.end_pitch;
	Arm_union_data.Arm_data_t.end_roll = Joint_collision_flag.end_roll;
	
	Arm_union_data.Arm_data_t.data1 = 0;	//预留数据位1
	Arm_union_data.Arm_data_t.data2 = 0;	//预留数据位2
	Arm_union_data.Arm_data_t.data3 = 0;	//预留数据位3
	Arm_union_data.Arm_data_t.data4 = 0;	//预留数据位4
	Arm_union_data.Arm_data_t.data5 = 0;	//预留数据位5
	Arm_union_data.Arm_data_t.data6 = 0;	//预留数据位6
	Arm_union_data.Arm_data_t.data7 = 0;	//预留数据位7
	
	Data_Concatenation(Arm_union_data.TX_Arm_data,DATA_LENGTH);
	
	if (HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_TX) 
	{ 
		//DMA串口发送函数
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)(&Arm_tx_data), sizeof(Arm_tx_data));
		retry_count = 0;
	}
	else if((HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX) && retry_count < MAX_retry_count)	retry_count ++;
	//如果未进入上面的判断中，证明串口处于繁忙的状态，没有修改回就绪态，证明出现了BUG此时需要重新发送三次，看是否是偶然一次不能用了
	
	else if((HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX) && retry_count == MAX_retry_count)
	{
		//重新发送三次依然无用之后会进入这里重启串口，重新进行DMA的初始化
		UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();						//任务临界区保护	开启保护
		HAL_UART_AbortTransmit(&huart6);																									//中止DMA发送传输
		HAL_UART_DeInit(&huart6);																													//失能UART6
		HAL_UART_Init(&huart6);																														//使能UART6
		MY_UART6_DMA_TX_Init();																														//UART1的DMA初始化（这里是再次初始化）
		portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);             						//任务临界区保护	关闭保护
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)(&Arm_tx_data), sizeof(Arm_tx_data));		//DMA串口开始发送
		retry_count = 0;																																	//重复发送计数器置零
	}
}

/***@role:	碰撞检测的函数***/
Joint_collision_flag_t Joint_collision_flag = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
void Collision_detection(Joint_collision_flag_t* flag_struct)
{
	//=====================================
	if(fabs(DM_YAW_back.tor) > 1.5f)
	{
		flag_struct->yaw = 2;
		if(fabs(DM_YAW_back.tor) > 3.5f)
		{
			flag_struct->yaw = 3;
			if(fabs(DM_YAW_back.tor) > 6.0f)	flag_struct->yaw = 4;
		}
	}
	else	flag_struct->yaw = 1;
	
	//======================================
	if(fabs(Big_Pitch_back.tor) > 6.0f)
	{
		flag_struct->Big_pitch = 2;
		if(fabs(Big_Pitch_back.tor) > 8.5f)
		{
			flag_struct->Big_pitch = 3;
			if(fabs(Big_Pitch_back.tor) > 10.5f)	flag_struct->Big_pitch = 4;
		}
	}
	else	flag_struct->Big_pitch = 1;
	
	//=====================================
	if(fabs(Smoll_Pitch_back.tor) > 8.0)
	{
		flag_struct->Smoll_pitch = 2;
		if(fabs(Smoll_Pitch_back.tor) > 13.0f)
		{
			flag_struct->Smoll_pitch = 3;
			if(fabs(Smoll_Pitch_back.tor) > 17.0f)	flag_struct->Smoll_pitch = 4;
		}
	}
	else	flag_struct->Smoll_pitch = 1;
	
	//=========================================
	if(fabs(Big_Roll_back.tor) > 4.0f)
	{
		flag_struct->Big_roll = 2;
		if(fabs(Big_Roll_back.tor) > 6.0f)
		{
			flag_struct->Big_roll = 3;
			if(fabs(Big_Roll_back.tor) > 8.0f)	flag_struct->Big_roll = 4;
		}
	}
	else	flag_struct->Big_roll = 1;
	
	//==========================================
	if(fabs(DM_endPitch_back.tor) > 1.5f)
	{
		flag_struct->end_pitch = 2;
		if(fabs(DM_endPitch_back.tor) > 3.6f)
		{
			flag_struct->end_pitch = 3;
			if(fabs(DM_endPitch_back.tor) > 6.0f)	flag_struct->end_pitch = 4;
		}
	}
	else	flag_struct->end_pitch = 1;
	
	//==========================================
	if(fabs(DM_endRoll_back.tor) > 1.7f)
	{
		flag_struct->end_roll = 2;
		if(fabs(DM_endRoll_back.tor) > 3.5f)
		{
			flag_struct->end_roll = 3;
			if(fabs(DM_endRoll_back.tor) > 5.5f)	flag_struct->end_roll = 4;
		}
	}
	else	flag_struct->end_roll = 1;
	
	if(fabs(DM_ROLL_back.tor) > 2.5f)
	{
		flag_struct->smoll_roll = 2;
		if(fabs(DM_ROLL_back.tor) > 3.5f)
		{
			flag_struct->smoll_roll = 3;
			if(fabs(DM_ROLL_back.tor) > 5.0f)	flag_struct->smoll_roll = 4;
		}
	}
	else	flag_struct->smoll_roll = 1;
	
	ui_Date_sent(flag_struct->yaw,flag_struct->Big_pitch,flag_struct->Smoll_pitch,flag_struct->Big_roll,
																						flag_struct->end_pitch,flag_struct->end_roll,flag_struct->smoll_roll);
}

void ui_Date_sent(uint8_t yaw,uint8_t big_pitch,uint8_t smoll_pitch,uint8_t big_roll,
															uint8_t end_pitch,uint8_t end_roll,uint8_t smoll_roll)
{
	union
	{
		struct
		{
			uint8_t yaw;
			uint8_t big_pitch;
			uint8_t smoll_pitch;
			uint8_t big_roll;
			uint8_t end_pitch;
			uint8_t end_roll;
			uint8_t smoll_roll;
			uint8_t data0:1;
			uint8_t data1:1;
			uint8_t data2:1;
			uint8_t data3:1;
			uint8_t data4:1;
			uint8_t data5:1;
			uint8_t data6:1;
			uint8_t data7:1;
		}Specific_data_t;
		uint8_t DATA_TX[8];
	}ui_tx_data;

	ui_tx_data.Specific_data_t.yaw = yaw;
	ui_tx_data.Specific_data_t.big_pitch = big_pitch;
	ui_tx_data.Specific_data_t.smoll_pitch = smoll_pitch;
	ui_tx_data.Specific_data_t.big_roll = big_roll;
	ui_tx_data.Specific_data_t.end_pitch = end_pitch;
	ui_tx_data.Specific_data_t.end_roll = end_roll;
	ui_tx_data.Specific_data_t.smoll_roll = smoll_roll;
	ui_tx_data.Specific_data_t.data0 = Joint_collision_flag.data0;
	ui_tx_data.Specific_data_t.data1 = Joint_collision_flag.data1;
	ui_tx_data.Specific_data_t.data2 = Joint_collision_flag.data2;
	ui_tx_data.Specific_data_t.data3 = Joint_collision_flag.data3;
	ui_tx_data.Specific_data_t.data4 = Joint_collision_flag.data4;
	ui_tx_data.Specific_data_t.data5 = Joint_collision_flag.data5;
	ui_tx_data.Specific_data_t.data6 = Joint_collision_flag.data6;
	ui_tx_data.Specific_data_t.data7 = Joint_collision_flag.data7;
	
	canx_bsp_send_data(&hcan2,0x00b,ui_tx_data.DATA_TX,8);	//其中的ID是板子的ID值
}
