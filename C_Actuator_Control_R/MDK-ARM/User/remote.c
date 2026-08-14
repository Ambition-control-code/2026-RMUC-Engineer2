#include "remote.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "AccStep.h"
#include "Key.h"
#include "queue.h"


/*定义的变量***begain***/
u8 RXBuff[DMA_RX_BUF_SIZE];		//DMA接收缓冲区
u8 dbus_recieve_1[36] = {0},dbus_recieve_2[36] = {0};//双缓冲的DBUS接收数据（双缓冲接收机制）

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern osMessageQueueId_t Claw_flagHandle;			//队列句柄
extern osMessageQueueId_t MYor_remoteHandle;
//---------------------------------------------------------------------------------------------

/***@role： 遥控器串口UART3的DMA接收初始化***/
Remote_all remote;
void MY_DMA_Init(void)
{
	/***@important： 以下是DBUS接收重要的初始化工作，必不可少！！！*/
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);                     //启用DMA接收请求
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                       //开启空闲中断
	__HAL_DMA_DISABLE(&hdma_usart3_rx);                                //失能DMA,解除保护
	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)                   //循环检查，避免失能失败
	{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}
	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_LISR_TCIF1);             //清除DMA标志
	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);          //配置DMA地址寄存器（核心配置）
	//以下两行是双缓存区，当缓冲区1满时自动切换到缓冲区2，同时CPU可处理缓冲区1的数据，避免数据覆盖，实现连续接收
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(dbus_recieve_1);
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(dbus_recieve_2);
	__HAL_DMA_SET_COUNTER(&hdma_usart3_rx, 36u);                       //设置传输长度（36个字节）
	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);                //启用双缓冲模式
	__HAL_DMA_ENABLE(&hdma_usart3_rx);                                 //启用DMA
}

/***@role: 能高效处理高频、小数据包的通信需求，确保不丢失任何数据帧  ***/
/***@why : 1、无接收间隙 2、零拷贝处理 3、高实时性 4、防止数据覆盖 ***/
extern uint32_t life_state_old;
uint8_t white_remote_flag = 0;
//串口中断
void USART3_IRQHandler(void)
{
	if(USART3->SR & UART_FLAG_IDLE)            //接收到数据(检测到空闲中断)
  {
		life_state_old = 0;			//生命标志清零，保证能稳定运行
		white_remote_flag = 1;	//白色遥控器连接标志位，使用DT7就默认不使用自定义控制器了
		
		static uint16_t this_time_rx_len = 0;
    __HAL_UART_CLEAR_PEFLAG(&huart3);        //清除空闲中断标志
    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
			/* 使用内存缓冲区0的情况 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
	
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;
	
			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = 36;
	
			//set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
					
			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
	
			if(this_time_rx_len == 18)
			{
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_1, 18);                                               
				deal_dbus(dbus_recieve_1);
			}
			else
      {
				/**
				*@注意： 
				*UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				*portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
				*以上两行代码是用于中断内的保护，可以完美执行这两行代码之间的代码
				
				//taskENTER_CRITICAL();	
				//taskEXIT_CRITICAL();
				*这两行代码是用于任务中的，可以完美执行这两行的代码，防止被中断打断，被更高优先级的任务打断
				***/
				
				UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				//taskENTER_CRITICAL();	
				HAL_UART_DeInit(&huart3);
				HAL_UART_Init(&huart3);
				MY_DMA_Init();
				//taskEXIT_CRITICAL();
				portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
			}
    }
    else
    {
			/* 使用内存缓冲区1的情况 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

      //get receive data length, length = set_data_length - remain_length
      //获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

      //reset set_data_lenght
      //重新设定数据长度
      hdma_usart3_rx.Instance->NDTR = 36;

      //set memory buffer 0
      //设定缓冲区0
      DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			
      //enable DMA
      //使能DMA
      __HAL_DMA_ENABLE(&hdma_usart3_rx);

      if(this_time_rx_len == 18)
      {
				//处理接收到的数据
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_2, 18);                                               
				deal_dbus(dbus_recieve_2);
      }
			else
      {
				/**
				*@注意： 
				*UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				*portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
				*以上两行代码是用于中断内的保护，可以完美执行这两行代码之间的代码
				
				//taskENTER_CRITICAL();	
				//taskEXIT_CRITICAL();
				*这两行代码是用于任务中的，可以完美执行这两行的代码，防止被中断打断，被更高优先级的任务打断
				***/
				
				UBaseType_t uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
				//taskENTER_CRITICAL();	
				HAL_UART_DeInit(&huart3);
				HAL_UART_Init(&huart3);
				MY_DMA_Init();
				//taskEXIT_CRITICAL();
				portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);
			}
    }
  }
}

/***@role： 检测拨杆变化的函数****/
u8 if_change(u8 flag,u8 last_flag)
{
	if(flag!=last_flag)	return 1;
	else	return 0;
}


/********************************/
/***@role：DBUS通讯协议**********/
/********************************/
uint8_t button_change_flag1 = 0,button_change_flag2 = 0;
uint32_t button_time1=0,button_time2=0,last_button_time=0;
void deal_dbus(u8 *RXBuff)
{
	if(((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff))
	{
		//右摇杆X通道 (通道0)
		remote.R_x = ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)-1024; //转换为-1024~1023
	}
	if((((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff))
	{
		//右摇杆Y通道 (通道1)
		remote.R_y = (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)-1024;
	} //!< Channel 1
	if((((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff))
	{
		//左摇杆X通道（通道2）
		remote.L_x = (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)-1024;
	} //!< Channel 2
	if((((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff))
	{
		//左摇杆Y通道（通道3）
		remote.L_y = (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)-1024;
	} //!< Channel 3
	remote.sl = (((RXBuff[5] >> 4)& 0x000C) >> 2);    //!< Switch left// 左侧开关 (2位状态)
	remote.sr = (((RXBuff[5] >> 4)& 0x0003));         //!< Switch left// 右侧开关 (2位状态)
	remote.trigger = 1024 - (RXBuff[16]|(RXBuff[17] << 8));//遥控器侧面的拨轮
	
	//鼠标值解析
	remote.mouse_x = RXBuff[6] | (RXBuff[7] << 8);    //!< Mouse X axis
	remote.mouse_y = RXBuff[8] | (RXBuff[9] << 8);    //!< Mouse Y axis
	remote.mouse_z = RXBuff[10]| (RXBuff[11]<< 8);    //!< Mouse Z axis//滚轮
	remote.mouse_l = RXBuff[12];                      //!< Mouse Left Is Press ?
	remote.mouse_r = RXBuff[13];                      //!< Mouse Right Is Press ?
	//键盘按键解析（位映射）
	remote.key = RXBuff[14] | (RXBuff[15] << 8);      //!< KeyBoard value
	remote.W = (remote.key&0x01)>>0;
	remote.S = (remote.key&0x02)>>1;
	remote.A = (remote.key&0x04)>>2;
	remote.D = (remote.key&0x08)>>3;
	remote.Shift = (remote.key&0x10)>>4;
	remote.Ctrl  = (remote.key&0x20)>>5;
	remote.Q = (remote.key&0x40)>>6;
	remote.E = (remote.key&0x80)>>7;
	remote.R = (remote.key&0x100)>>8;
	remote.F = (remote.key&0x200)>>9;
	remote.G = (remote.key&0x400)>>10;
	remote.Z = (remote.key&0x800)>>11;
	remote.X = (remote.key&0x1000)>>12;
	remote.C = (remote.key&0x2000)>>13;
	remote.V = (remote.key&0x4000)>>14;
	remote.B = (remote.key&0x8000)>>15;
	//遥控器值抛出异常（数据有效性检验）
	remote.R_x = remote.R_x > 660 ? 0 : remote.R_x;   // 摇杆死区处理（超出±660置零）
	remote.R_x = remote.R_x < -660 ? 0 : remote.R_x;
	
	remote.R_y = remote.R_y > 660 ? 0 : remote.R_y;
	remote.R_y = remote.R_y < -660 ? 0 : remote.R_y;
	
	remote.L_x = remote.L_x > 660 ? 0 : remote.L_x;
	remote.L_x = remote.L_x < -660 ? 0 : remote.L_x;
	
	remote.L_y = remote.L_y > 660 ? 0 : remote.L_y;
	remote.L_y = remote.L_y < -660 ? 0 : remote.L_y;
	
	remote.mouse_x = remote.mouse_x > 1024 ? 0 : remote.mouse_x;
	remote.mouse_x = remote.mouse_x < -1024 ? 0 : remote.mouse_x;
	remote.mouse_y = remote.mouse_y > 1024 ? 0 : remote.mouse_y;
	remote.mouse_y = remote.mouse_y < -1024 ? 0 : remote.mouse_y;
	
	remote.sl_change_flag=if_change(remote.sl,remote.last_sl);
	remote.sr_change_flag=if_change(remote.sr,remote.last_sr);	
	remote.mr_change_flag=if_change(remote.mouse_r,remote.last_mouse_r);		
	remote.ml_change_flag=if_change(remote.mouse_l,remote.last_mouse_l);		
	
//	remote.last_sl=remote.sl;
//	remote.last_sr=remote.sr;
	remote.last_mouse_r=remote.mouse_r;
	remote.last_mouse_l=remote.mouse_l;
	
	//扳机键向下标志位，初始状态位是0，每摁一次加一
	if(remote.trigger < (-800) && button_change_flag1 == 0)							//按下
 	{
		if(HAL_GetTick() - last_button_time > 5)						
		{						
			button_time1 = HAL_GetTick();   																//按下时间
		}         
		button_change_flag1 = 1;																					//转换标志位
	}
	else if((remote.trigger > (-800) && remote.trigger < 0) && button_change_flag1 == 1)//松开
	{					
		if(HAL_GetTick() - last_button_time > 5)						
		{						
			button_time2 = HAL_GetTick();																		//松开时间
			if(button_time2 - button_time1 > 5)															//消抖
			{
				last_button_time = HAL_GetTick();
				remote.trigger_flag_up = 1 - remote.trigger_flag_up;
				button_change_flag1 = 0;																			//转换标志位
			}
		}
	}
	
	//扳机键向上标志位，初始状态位是0，每摁一次加一
	if(remote.trigger > 800 && button_change_flag2 == 0)							//按下
 	{
		if(HAL_GetTick() - last_button_time > 5)						
		{						
			button_time1 = HAL_GetTick();   																//按下时间
		}         
		button_change_flag2 = 1;																					//转换标志位
	}
	else if((remote.trigger > (-800) && remote.trigger < 0) && button_change_flag2 == 1)//松开
	{					
		if(HAL_GetTick() - last_button_time > 5)						
		{						
			button_time2 = HAL_GetTick();																		//松开时间
			if(button_time2 - button_time1 > 5)															//消抖
			{
				last_button_time = HAL_GetTick();
				remote.trigger_flag_down = 1 - remote.trigger_flag_down;
				button_change_flag2 = 0;																			//转换标志位
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------------
unsigned short AccStep[ACC_STEP_NUM] =      // 加速曲线数组
{
	0,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,8,8,8,8,9,9,9,9,10,10,10,
	11,11,11,12,12,12,13,13,13,14,14,15,15,16,16,16,17,17,18,18,19,20,20,21,21,22,23,23,
	24,25,25,26,27,28,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,51,
	52,54,55,57,58,60,61,63,64,66,68,70,72,73,75,77,79,81,83,85,88,90,92,94,97,99,101,104,
	106,109,111,114,117,119,122,125,128,131,133,136,139,142,146,149,152,155,158,162,165,168,
  171,175,178,182,185,189,192,196,199,203,207,210,214,218,221,225,229,233,236,240,244,248,
	251,255,259,262,266,270,274,277,281,285,288,292,296,299,303,307,310,314,317,321,324,327,
	331,334,337,341,344,347,350,353,357,360,363,366,368,371,374,377,380,382,385,388,390,393,
	395,398,400,403,405,407,410,412,414,416,418,420,422,424,426,428,430,431,433,435,437,438,
	440,441,443,444,446,447,449,450,451,453,454,455,456,458,459,460,461,462,463,464,465,466,
	467,468,469,470,471,471,472,473,474,474,475,476,477,477,478,478,479,480,480,481,481,482,
	482,483,483,484,484,485,485,486,486,486,487,487,488,488,488,489,489,489,490,490,490,490,
	491,491,491,492,492,492,492,492,493,493,493,493,494,494,494,494,494,494,495,495,495,495,
	495,495,495,496,496,496,496,496,496,496,496,497,497
};
//==========================================================================================================

/********************************************************************/
/***@role： 按键加速控制*********************************************/
/***@objective： 解决机械按键的瞬时响应问题，实现平滑的加速/减速效果*/
/********************************************************************/
float deal_W,deal_A,deal_S,deal_D,deal_E,deal_Q,deal_F;// 处理后的输出值																											// 处理后的输出值
int t_W,t_A,t_S,t_D,t_E,t_Q,t_R,t_F,t_mouse_r,t_mouse_l;// 时间计数器

void remote_fifo(void)
{
	// 按键状态检测
	if(remote.W!=0 )t_W++; else t_W--;//W键
	if(remote.A!=0) t_A++; else t_A--;//A键
	if(remote.S!=0) t_S++; else t_S--;//S键
	if(remote.D!=0) t_D++; else t_D--;//D键
	if(remote.E!=0) t_E++; else t_E--;//E键
	if(remote.Q!=0) t_Q++; else t_Q--;//Q键
	if(remote.F!=0) t_F++; else t_F--;//
	if(remote.mouse_r!=0) t_mouse_r++; else t_mouse_r--;//
	if(remote.mouse_l!=0) t_mouse_l++; else t_mouse_l--;//
	
	// 计数器限幅
	t_W = t_W > 334 ? 334 : t_W;  t_W = t_W < 0 ? 0 : t_W;	
	t_A = t_A > 334 ? 334 : t_A;  t_A = t_A < 0 ? 0 : t_A;	
	t_S = t_S > 334 ? 334 : t_S;  t_S = t_S < 0 ? 0 : t_S;
	t_D = t_D > 334 ? 334 : t_D;  t_D = t_D < 0 ? 0 : t_D;
	t_E = t_E > 334 ? 334 : t_E;  t_E = t_E < 0 ? 0 : t_E;
	t_Q = t_Q > 334 ? 334 : t_Q;  t_Q = t_Q < 0 ? 0 : t_Q;
	t_R = t_R > 334 ? 334 : t_R;  t_R = t_R < 0 ? 0 : t_R;
	t_F = t_F > 334 ? 334 : t_F;  t_F = t_F < 0 ? 0 : t_F;
	t_mouse_r = t_mouse_r > 334 ? 334 : t_mouse_r;  t_mouse_r = t_mouse_r < 0 ? 0 : t_mouse_r;
	t_mouse_l = t_mouse_l > 334 ? 334 : t_mouse_l;  t_mouse_l = t_mouse_l < 0 ? 0 : t_mouse_l;
	
	// 映射到加速曲线
	deal_W=AccStep[t_W];
	deal_A=AccStep[t_A];
	deal_S=AccStep[t_S];
	deal_D=AccStep[t_D];
	deal_E=AccStep[t_E];
	deal_Q=AccStep[t_Q];
	deal_F=AccStep[t_F];
}


KeySpeedCtrl_t W_ctrl;
KeySpeedCtrl_t A_ctrl;
KeySpeedCtrl_t S_ctrl;
KeySpeedCtrl_t D_ctrl;
KeySpeedCtrl_t Q_ctrl;
KeySpeedCtrl_t E_ctrl;


//新的按键处理函数
void New_Remote_fifo(void)
{
	// 处理每个按键的状态
  Process_Key_Speed(VT13_use.use_data.W != 0, &W_ctrl);
  Process_Key_Speed(VT13_use.use_data.A != 0, &A_ctrl);
  Process_Key_Speed(VT13_use.use_data.S != 0, &S_ctrl);
  Process_Key_Speed(VT13_use.use_data.D != 0, &D_ctrl);
  Process_Key_Speed(VT13_use.use_data.Q != 0, &Q_ctrl);
  Process_Key_Speed(VT13_use.use_data.E != 0, &E_ctrl);

	// 赋值给你的全局变量
  deal_W = W_ctrl.out_speed;
  deal_A = A_ctrl.out_speed;
  deal_S = S_ctrl.out_speed;
  deal_D = D_ctrl.out_speed;
  deal_Q = Q_ctrl.out_speed;
  deal_E = E_ctrl.out_speed;
}


//-----------------------------------------------------------------------------------------------------------------

//================================================================================================================
/***@roel: 用C板遥控的数据发送给底盘***/
uint8_t Init_flag = 0;
void Chassis_Date_sent(int16_t x,int16_t y,int16_t turn,uint8_t Stop,uint8_t erro) //底盘发送
{
	union
	{
		struct
		{
			int16_t x_t;
			int16_t y_t;
			int16_t turn_t;
			uint8_t Chassis_mode;
			uint8_t stop_t:1;
			uint8_t Z:1;	//预留数据位
			uint8_t C:1;
			uint8_t Init_flag:1;	//自稳标志位
			uint8_t erro:1;
			uint8_t V:1;
			uint8_t F:1;
			uint8_t SLR:1;
		}Specific_data_t;
		uint8_t DATA_TX[8];
	}C_to_C_tx_data;

	C_to_C_tx_data.Specific_data_t.x_t = x;
	C_to_C_tx_data.Specific_data_t.y_t = y;
	C_to_C_tx_data.Specific_data_t.turn_t = turn;
	C_to_C_tx_data.Specific_data_t.Chassis_mode = New_VT03_data.New_R;
	C_to_C_tx_data.Specific_data_t.stop_t = Stop;
	C_to_C_tx_data.Specific_data_t.erro = erro;
	C_to_C_tx_data.Specific_data_t.Init_flag = Init_flag;
	C_to_C_tx_data.Specific_data_t.Z = New_VT03_data.New_Z;
	C_to_C_tx_data.Specific_data_t.C = New_VT03_data.New_C;
	C_to_C_tx_data.Specific_data_t.V = New_VT03_data.New_V;
	C_to_C_tx_data.Specific_data_t.F = New_VT03_data.New_F;
	
	canx_bsp_send_data(&hcan2,0x00a,C_to_C_tx_data.DATA_TX,8);	//其中的ID是板子的ID值
}
//============================================================================

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role:	标志位检测函数***/
uint8_t last_storage_flag1 = 0,last_storage_flag2 = 0;//一键存储能量单元的标志位，实则没用上
uint8_t remote_MyControl_flag = 0;//自定义控制器遥控拨杆控制的标志位,底盘下电标志位
uint8_t Chassis_or_Arm_or_End = 0;//2为机械臂，1为末端，0为底盘
extern uint32_t life_state_new;
float Claw_Speed = 0.0f;
extern float Claw_mid_angle;
uint8_t change_claw_state = 0;
extern uint8_t Front_flag,Behind_flag;
uint8_t claw_flag = 0;

void Flag_Inspect_task(void)
{
	//=============是否使用自控标志位判断=====================================
	if(New_VT03_data.pause)	remote_MyControl_flag = 1;//遥控器强制锁定不用自定义控制器
	//遥控器放开权限，然后可以在自定义控制器那边切换自定义控制器模式和遥控键鼠模式
	else	remote_MyControl_flag = 0;
	//=========================================================================
	
	//=========================模式判断========================================
	if(remote.sl == 2 || remote.sl == 3 || New_VT03_data.mode_sw || New_VT03_data.mode_sw == 2)
	{
		if(remote.sl == 3 || New_VT03_data.mode_sw == 1)	Chassis_or_Arm_or_End = 2;			//2为使用机械臂
		if(remote.sl == 2 || New_VT03_data.mode_sw == 2)	Chassis_or_Arm_or_End = 1;			//1为使用末端
	}
	else	Chassis_or_Arm_or_End = 0;
	//=========================================================================
	
	//=============================夹爪的标志位==================================================
	static uint8_t last_claw_flag = 0;
	static uint8_t last_trigger = 0;
	static uint8_t last_mouse_left = 0;
	
	if(claw_flag != last_claw_flag) 
	{
    	change_claw_state = claw_flag;
    	New_VT03_data.trigger = change_claw_state;
		New_VT03_data.mouse_left = change_claw_state;
	}
	else if(New_VT03_data.trigger != last_trigger) 
	{
    	change_claw_state = New_VT03_data.trigger;
    	claw_flag = change_claw_state;
		New_VT03_data.mouse_left = change_claw_state;
	}
	else if(New_VT03_data.mouse_left != last_mouse_left)
	{
		change_claw_state = New_VT03_data.mouse_left;
		claw_flag = change_claw_state;
		New_VT03_data.trigger = change_claw_state;
	}

	if(change_claw_state)	Claw_Speed = Claw_mid_angle - 1150.0f;
	else	Claw_Speed = Claw_mid_angle;

	last_claw_flag = claw_flag;
	last_trigger = New_VT03_data.trigger;
	last_mouse_left = New_VT03_data.mouse_left;
	//============================================================================================
	
	//=============================腿部控制标志位处理============================================
	
	if(New_VT03_data.New_R||New_VT03_data.fn_1)	
		remote_MyControl_flag = 5;
	else if(Front_flag!=Behind_flag)	
		remote_MyControl_flag = 6;
	
	//队列函数用于切换机械臂姿态
	if(DM_Init_count == 15)
	{
		uint8_t remote_MyControl_flag_t = remote_MyControl_flag;
		osMessageQueuePut(MYor_remoteHandle,&remote_MyControl_flag_t,0,0);
	}
	//==========================================================================================
	
	//=============检测遥控器状态标志位判断，如果遥控器断了也就算是死了========
	if(life_state_new >= 1000 && life_state_old >= 1000)//生命状态标志自增到100标志着遥控器已经断连
	{
//		Big_Pitch = Big_Pitch_back.engle;
		Smoll_Pitch = Smoll_Pitch_back.engle;
		Set_Arm_roll = DM_ROLL_back.engle;
		Set_Arm_yaw = DM_YAW_back.engle;
		Set_end_Pit = DM_endPitch_back.engle;
		Set_end_roll = DM_endRoll_back.engle;
		
		Chassis_or_Arm_or_End = 2;
	}
	//=====================================================================================
}

/***@role: 处理识别遥控（或者键鼠）信息传给底盘***/
uint32_t Chassis_life = 0;
void Remote_To_Chassis(void)
{
	float remote_x, remote_y, X, Y, aa;          //云台的坐标系
	float W = 0,A = 0,S = 0,D = 0;
	
	New_Remote_fifo();                           // 调用按键加速处理函数
	// 将加速处理后的按键值乘以系数
	W = (int)(keypad_K * deal_W);                //前进
	A = (int)(keypad_K * deal_A);                //左移
	S = (int)(keypad_K * deal_S);                //后退
	D = (int)(keypad_K * deal_D);                //右移
	
	// 处理摇杆输入(乘以系数并取反)
	remote_x = (New_VT03_data.R_rocker_x + remote.R_x) * remote_K;           //右摇杆X轴
	remote_y = (New_VT03_data.R_rocker_y + remote.R_y) * remote_K;           //右摇杆Y轴
  /*下面四行代码需要根据实际情况进行修改！！！！！切记！切记！*/
	// 合成运动矢量 (键盘WASD + 右摇杆)
//	Y = W - S + remote_y;
//	X = -A + D + remote_x;
	Y = W - S + remote_y;                       //前后方向
	X = D - A + remote_x;                       //左右方向
//*上面四行代码需要根据实际情况进行修改！！！！！切记！切记！*//
	
	// 计算合成矢量的模长
	aa = sqrt(X * X + Y * Y);
	
	// 对速度矢量进行限幅 (防止超出最大值)
	X = aa <  limit ? X : X * (limit / aa);
	X = aa > -limit ? X : X * (limit / aa);      //处理负方向
	Y = aa <  limit ? Y : Y * (limit / aa);
	Y = aa > -limit ? Y : Y * (limit / aa);
	
	if(Chassis_or_Arm_or_End != 0)
		Chassis_Date_sent(0,0,0,1,0);//下电停止//这里发给底盘
	else
	{
		if(New_VT03_data.shift)
		{
			X = X * 0.2f;
			Y = Y * 0.2f;
		}
		
		// 更新底盘控制输出
		Chassis_Date_sent(X,Y,0,Chassis_or_Arm_or_End,0);//这里发给底盘
	}
}

/***@role:	处理新遥控器数据的函数***/
VT13_USE VT13_use = {0};
New_VT03_data_T New_VT03_data = {0};
void Handle_VT03_Data(New_VT03_data_T* new_VT03_data_t)
{
	new_VT03_data_t->R_rocker_x = VT13_use.use_data.ch_0 - 1024;
	new_VT03_data_t->R_rocker_y = VT13_use.use_data.ch_1 - 1024;
	new_VT03_data_t->L_rocker_y = VT13_use.use_data.ch_2 - 1024;
	new_VT03_data_t->L_rocker_x = VT13_use.use_data.ch_3 - 1024;
	new_VT03_data_t->wheel = VT13_use.use_data.wheel - 1024;
	new_VT03_data_t->shift = VT13_use.use_data.SHIFT;
	new_VT03_data_t->mode_sw = VT13_use.use_data.mode_sw;
	new_VT03_data_t->New_V = VT13_use.use_data.V;
	new_VT03_data_t->New_X = VT13_use.use_data.X;
	
	if(Button_Check_Event(VT13_use.use_data.pause,&Key_Pause))	new_VT03_data_t->pause = !new_VT03_data_t->pause;
	if(Button_Check_Event(VT13_use.use_data.trigger,&Key_trigger))	new_VT03_data_t->trigger = !new_VT03_data_t->trigger;
	if(Button_Check_Event(VT13_use.use_data.fn_1,&Key_fn1))	
	{
		new_VT03_data_t->fn_1 = !new_VT03_data_t->fn_1;
		new_VT03_data_t->New_R = new_VT03_data_t->fn_1;
	}
	if(Button_Check_Event(VT13_use.use_data.fn_2,&Key_fn2))	new_VT03_data_t->fn_2 = !new_VT03_data_t->fn_2;
	if(Button_Check_Event(VT13_use.use_data.R,&Key_R))	
	{
		new_VT03_data_t->New_R = !new_VT03_data_t->New_R;
		new_VT03_data_t->fn_1 = new_VT03_data_t->New_R;
	}
	
	if(new_VT03_data_t->New_R == 0)
	{
		if(Button_Check_Event(VT13_use.use_data.Z,&Key_Z))	
		{
			if(new_VT03_data_t->New_Z == 0)
			{
				new_VT03_data_t->New_Z = 1;
				new_VT03_data_t->New_C = 0;
				Front_flag = 1;
				Behind_flag = 0;
			}
			else
			{
				Front_flag = 0;
				new_VT03_data_t->New_Z = 0;
			}
		}
		if(Button_Check_Event(VT13_use.use_data.C,&Key_C))	
		{
			if(new_VT03_data_t->New_C == 0)
			{
				new_VT03_data_t->New_Z = 0;
				new_VT03_data_t->New_C = 1;
				Front_flag = 0;
				Behind_flag = 1;
			}
			else
			{
				Behind_flag = 0;
				new_VT03_data_t->New_C = 0;
			}
		}
	}
	if(Button_Check_Event(VT13_use.use_data.mouse_left,&Key_mouse_left))	new_VT03_data_t->mouse_left = !new_VT03_data_t->mouse_left;
	if(Button_Check_Event(VT13_use.use_data.F,&Key_F))	new_VT03_data_t->New_F = !new_VT03_data_t->New_F;
}

/***
 *@role：	遥控器内八软件复位,外八数据清零。
***/
uint8_t wait_num1 = 0,wait_num2 = 0;
void remote_save_control(void)
{
	if((New_VT03_data.L_rocker_x>650&&New_VT03_data.L_rocker_y<(-650)
		&&New_VT03_data.R_rocker_x<(-650)&&New_VT03_data.R_rocker_y<(-650))
		||(remote.L_x>650&&remote.L_y<(-650)&&remote.R_x<(-650)&&remote.R_y<(-650)))
	{
		if((wait_num1 ++) > 150)
		{
			Chassis_Date_sent(0,0,0,1,1);
			HAL_Delay(2);
			__disable_irq();
			NVIC_SystemReset();
		}
	}
	else	wait_num1 = 0;
	
	if((New_VT03_data.L_rocker_x<(-650)&&New_VT03_data.L_rocker_y<(-650)
		&&New_VT03_data.R_rocker_x>650&&New_VT03_data.R_rocker_y<(-650))
		||(remote.L_x<(-650)&&remote.L_y<(-650)&&remote.R_x>650&&remote.R_y<(-650)))
	{
		if((wait_num2 ++) > 150)
		{
			Big_Pitch = 0.0f;
			Set_Arm_roll = 0.0f;
			Set_end_Pit = 0.0f;
			Smoll_Pitch = 0.0f;
			Set_Arm_yaw = 0.0f;
			Set_end_roll = 0.0f;
			New_VT03_data.pause = 0;
			white_remote_flag = 0;
		}
	}
	else	wait_num2 = 0;
}

void Turn_Date_sent(float turn) //底盘发送
{
	union
	{
		struct
		{
			float  turn_t;
		}Specific_data_t;
		uint8_t DATA_TX[4];
	}C_to_C_tx_data;

	C_to_C_tx_data.Specific_data_t.turn_t = turn;
	
	canx_bsp_send_data(&hcan2,0x00c,C_to_C_tx_data.DATA_TX,4);	//其中的ID是板子的ID值
}
