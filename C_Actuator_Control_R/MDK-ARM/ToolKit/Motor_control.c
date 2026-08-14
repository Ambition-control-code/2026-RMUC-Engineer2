#include "Motor_control.h"
#include "math.h"
#include "can_bsp.h"
#include "bits_change.h"
#include "cmsis_os.h"

/******begain！！！！！！！！！！结构体重命名*********/

/******end！！！！！！！！！！结构体重命名*********/

//==========================================共用代码如下===================================================================

/***
 *@role:      CAN通讯给电机发送指令的函数
 *@remeber:   达妙电机一拖四模式，大疆电机，本末电机都可以使用这个代码
 *@remeber:   达妙4310和4340找到了对应的一拖四固件，10010L没有找到
 *@remeber:   大疆电机6020的没有写，是因为工程不需要，这样分类写出来太多了，就写了这些，有需要可以接着写
 *@remeber:   这样写是防止电机多了之后数据变串，虽然比控制一部分写一个函数在别的任务中省事，
 *						但是在这个任务中明显需要写很多很多的东西去分辨，两种方法各有优劣吧。
***/
uint8_t dJi200_hcan1_data[8] = {0};        //准备大疆电机can1链路0x200的控制数据【3508】
uint8_t dJi200_hcan2_data[8] = {0};        //准备大疆电机can2链路0x200的控制数据【3508】
uint8_t dJi1FF_hcan1_data[8] = {0};        //准备大疆电机can1链路0x1FF的控制数据【3508】
uint8_t dJi1FF_hcan2_data[8] = {0};        //准备大疆电机can2链路0x1FF的控制数据【3508】
uint8_t BM032_hcan1_data[8]  = {0};        //准备本末电机can1链路0x032的控制数据
uint8_t BM032_hcan2_data[8]  = {0};        //准备本末电机can2链路0x032的控制数据
uint8_t BM033_hcan1_data[8]  = {0};        //准备本末电机can1链路0x033的控制数据
uint8_t BM033_hcan2_data[8]  = {0};        //准备本末电机can2链路0x033的控制数据
void Motor_control(CAN_HandleTypeDef *hcan,uint16_t Control_ID,int16_t Speed,uint8_t esc_id)
{
	if(hcan == &hcan1)
	{
		switch(Control_ID)
		{
			case 0x200:	
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi200_hcan1_data[0] = Speed >> 8;dJi200_hcan1_data[1] = Speed & 0xFF;break;
					case 2:dJi200_hcan1_data[2] = Speed >> 8;dJi200_hcan1_data[3] = Speed & 0xFF;break;
					case 3:dJi200_hcan1_data[4] = Speed >> 8;dJi200_hcan1_data[5] = Speed & 0xFF;break;
					case 4:dJi200_hcan1_data[6] = Speed >> 8;dJi200_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi1FF_hcan1_data[0] = Speed >> 8;dJi1FF_hcan1_data[1] = Speed & 0xFF;break;
					case 6:dJi1FF_hcan1_data[2] = Speed >> 8;dJi1FF_hcan1_data[3] = Speed & 0xFF;break;
					case 7:dJi1FF_hcan1_data[4] = Speed >> 8;dJi1FF_hcan1_data[5] = Speed & 0xFF;break;
					case 8:dJi1FF_hcan1_data[6] = Speed >> 8;dJi1FF_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x032:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:BM032_hcan1_data[0] = Speed >> 8;BM032_hcan1_data[1] = Speed & 0xFF;break;
					case 2:BM032_hcan1_data[2] = Speed >> 8;BM032_hcan1_data[3] = Speed & 0xFF;break;
					case 3:BM032_hcan1_data[4] = Speed >> 8;BM032_hcan1_data[5] = Speed & 0xFF;break;
					case 4:BM032_hcan1_data[6] = Speed >> 8;BM032_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x033:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:BM033_hcan1_data[0] = Speed >> 8;BM033_hcan1_data[1] = Speed & 0xFF;break;
					case 6:BM033_hcan1_data[2] = Speed >> 8;BM033_hcan1_data[3] = Speed & 0xFF;break;
					case 7:BM033_hcan1_data[4] = Speed >> 8;BM033_hcan1_data[5] = Speed & 0xFF;break;
					case 8:BM033_hcan1_data[6] = Speed >> 8;BM033_hcan1_data[7] = Speed & 0xFF;break;
				}
			}	break;
		}
	}
	else
	{
		switch(Control_ID)
		{
			case 0x200:	
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:dJi200_hcan2_data[0] = Speed >> 8;dJi200_hcan2_data[1] = Speed & 0xFF;break;
					case 2:dJi200_hcan2_data[2] = Speed >> 8;dJi200_hcan2_data[3] = Speed & 0xFF;break;
					case 3:dJi200_hcan2_data[4] = Speed >> 8;dJi200_hcan2_data[5] = Speed & 0xFF;break;
					case 4:dJi200_hcan2_data[6] = Speed >> 8;dJi200_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x1FF:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:dJi1FF_hcan2_data[0] = Speed >> 8;dJi1FF_hcan2_data[1] = Speed & 0xFF;break;
					case 6:dJi1FF_hcan2_data[2] = Speed >> 8;dJi1FF_hcan2_data[3] = Speed & 0xFF;break;
					case 7:dJi1FF_hcan2_data[4] = Speed >> 8;dJi1FF_hcan2_data[5] = Speed & 0xFF;break;
					case 8:dJi1FF_hcan2_data[6] = Speed >> 8;dJi1FF_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x032:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 1:BM032_hcan2_data[0] = Speed >> 8;BM032_hcan2_data[1] = Speed & 0xFF;break;
					case 2:BM032_hcan2_data[2] = Speed >> 8;BM032_hcan2_data[3] = Speed & 0xFF;break;
					case 3:BM032_hcan2_data[4] = Speed >> 8;BM032_hcan2_data[5] = Speed & 0xFF;break;
					case 4:BM032_hcan2_data[6] = Speed >> 8;BM032_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
			case 0x033:
			{
				switch(esc_id)	//标识符0x200对应esc_id为1234，标识符0x1FF对应esc_id为5678
				{
					case 5:BM033_hcan2_data[0] = Speed >> 8;BM033_hcan2_data[1] = Speed & 0xFF;break;
					case 6:BM033_hcan2_data[2] = Speed >> 8;BM033_hcan2_data[3] = Speed & 0xFF;break;
					case 7:BM033_hcan2_data[4] = Speed >> 8;BM033_hcan2_data[5] = Speed & 0xFF;break;
					case 8:BM033_hcan2_data[6] = Speed >> 8;BM033_hcan2_data[7] = Speed & 0xFF;break;
				}
			}	break;
		}
	}
}

/***
 *@role：    使能CAN通讯的函数
 *@remeber： 大疆电机3508的控制标识符ID号1~4是0x200，5~8是0x1FF
 *@remeber： 大疆6020的控制标识符电压模式ID号1~4是0x1FF，5~7是0x2FF，电流模式ID号是1~4是0x1FE，5~7是0x2FE
 *@remeber： 本末电机的控制标识符ID号1~4是0x032，5~8是0x033
***/
void Motor_enable(CAN_HandleTypeDef *hcan,uint16_t Control_ID)
{
	if(hcan == &hcan1)
	{
		switch(Control_ID)
		{
			case 0x200:	canx_bsp_send_data(hcan, Control_ID, dJi200_hcan1_data, 8);	break;
			case 0x1FF:	canx_bsp_send_data(hcan, Control_ID, dJi1FF_hcan1_data, 8);	break;
			case 0x032: canx_bsp_send_data(hcan, Control_ID, BM032_hcan1_data , 8);	break;
			case 0x033: canx_bsp_send_data(hcan, Control_ID, BM033_hcan1_data , 8);	break;
		}
	}
	else
	{
		switch(Control_ID)
		{
			case 0x200:	canx_bsp_send_data(hcan, Control_ID, dJi200_hcan2_data, 8);	break;
			case 0x1FF:	canx_bsp_send_data(hcan, Control_ID, dJi1FF_hcan2_data, 8);	break;
			case 0x032: canx_bsp_send_data(hcan, Control_ID, BM032_hcan2_data , 8);	break;
			case 0x033: canx_bsp_send_data(hcan, Control_ID, BM033_hcan2_data , 8);	break;
		}
	}
}

//=========================================================================================================================

//============================================大疆电机代码如下====================================================================

uint8_t RX_data[8] = {0};//数据缓存区
/*****@role: CAN通讯接收大疆电机的数据*****/
void get_dJi_Motor_Data(volatile Motor_Feedback *feedback)//volatile关键字确保编译器不对电机状态结构体做优化（因为会被中断修改）
{
	int32_t temp_sum = 0;                                          	    //用于计算速度平均值的临时累加器
	feedback->last_raw_value = feedback->raw_value;                		  //保存上一次的编码器原始值（用于计算差值）
	feedback->raw_value = (RX_data[0]<<8)|RX_data[1];               	  //编码器的值(这是大疆的通讯协议)
	feedback->diff = feedback->raw_value - feedback->last_raw_value;	  //计算两次采样间的编码器变化量，范围：-8191 ~ +8191（14位编码器）
	if(feedback->diff < -4096)                                 			    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		feedback->round_cnt++;
		feedback->ecd_raw_rate = feedback->diff + 8192;
	}
	else if(feedback->diff>4096)
	{
		feedback->round_cnt--;
		feedback->ecd_raw_rate = feedback->diff - 8192;
	}
	else{feedback->ecd_raw_rate = feedback->diff;}
	//真实角度【0-360】
	feedback->Real_Angle = feedback->raw_value * 0.0439453125f;
	//计算得到连续的编码器输出值??
	feedback->ecd_value = feedback->raw_value + feedback->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	feedback->ecd_angle = ((float)(feedback->raw_value - feedback->ecd_bias)*360/8192 + feedback->round_cnt * 360);
	feedback->rate_buf[feedback->buf_count++] = feedback->ecd_raw_rate;
	if(feedback->buf_count == RATE_BUF_SIZE)
	{feedback->buf_count = 0;}
	
	//计算速度平均值??
	for(int i = 0;i < RATE_BUF_SIZE; i++)
	{temp_sum += feedback->rate_buf[i];}
	
	feedback->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)feedback->filter_rate_max) < fabs((float)feedback->filter_rate))
	{feedback->filter_rate_max = feedback->filter_rate;}
	feedback->current = (uint16_t)(RX_data[4] << 8 | RX_data[5]);         //电流
	feedback->tempature = RX_data[6];                                     //温度
	feedback->re_speed= (int16_t) (RX_data[2] << 8 | RX_data[3]);         //速度用这个
}

//========================================================================================================================//

//=============================================达妙电机相关函数如下=======================================================//

/***@roel: 获取达妙电机数据的函数***/
void get_DM_OTF_Data(volatile Motor_Feedback *feedback)             	//OTF：One To Four一拖四模式
{
	feedback->last_raw_value = feedback->raw_value;                 	//保存上一次的编码器原始值（用于计算差值）
	feedback->raw_value = (uint16_t)(RX_data[0]<<8)|RX_data[1];     	//编码器的值(这是大疆的通讯协议[达妙电机一拖四模式也是这样的协议])
	feedback->diff = feedback->raw_value - feedback->last_raw_value;	//计算两次采样间的编码器变化量，范围：-8191 ~ +8191（14位编码器）
	if(feedback->diff < -4096)                                      	//两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		feedback->round_cnt++;
		feedback->ecd_raw_rate = feedback->diff + 8192;
	}
	else if(feedback->diff>4096)
	{
		feedback->round_cnt--;
		feedback->ecd_raw_rate = feedback->diff - 8192;
	}		
	else	{feedback->ecd_raw_rate = feedback->diff;}
	//计算得到连续的编码器输出值
	feedback->ecd_value = feedback->raw_value + feedback->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	feedback->ecd_angle = ((float)(feedback->raw_value)*360/8192 + feedback->round_cnt * 360);
	feedback->re_speed  = ((int16_t)(RX_data[2] << 8 | RX_data[3])*0.01);          //速度
	feedback->current   = (RX_data[4] << 8 | RX_data[5]);                          //电流
	feedback->tempature =  RX_data[6];                                             //温度
	feedback->state     =  RX_data[7];                            			       		 //错误状态,详见错误状态说明书定义
}

/***@role: 达妙电机常规模式获取电机数据得函数***/
DM_Normol_Feedback Big_Pitch_back   = {0};
DM_Normol_Feedback Big_Roll_back   = {0};
DM_Normol_Feedback Smoll_Pitch_back = {0};
DM_Normol_Feedback DM_YAW_back = {0};
DM_Normol_Feedback DM_ROLL_back = {0};
DM_Normol_Feedback DM_endPitch_back = {0};
DM_Normol_Feedback DM_endRoll_back = {0};
DM_Normol_Feedback DM_Claw_back = {0};

DM_Normol_Feedback DM_riseMotor_back = {0};

void DM_Normal_GetData(DM_Normol_Feedback* DM_Normal_feedback)
{
	int p_int = 0,v_int = 0,t_int = 0;
	
	DM_Normal_feedback->id = ((RX_data[0])&0x0F);								 											//控制器的ID
	DM_Normal_feedback->state = ((RX_data[0])>>4);							 											//电机状态
	p_int = ((RX_data[1]<<8)|RX_data[2]);
	v_int = ((RX_data[3]<<4)|(RX_data[4]>>4));
	t_int = (((RX_data[4]&0xF)<<8)|RX_data[5]);
	
	DM_Normal_feedback->last_raw_pos = DM_Normal_feedback->raw_pos;
	DM_Normal_feedback->raw_pos = (p_int * 0.02185693f) - 716.197f;//0.02185693f = 1432.394/65535,65535是编码器值
	DM_Normal_feedback->diff = DM_Normal_feedback->raw_pos - DM_Normal_feedback->last_raw_pos;
	
	if(DM_Normal_feedback->diff < (-1000.0f))		DM_Normal_feedback->round_cnt ++;
	if(DM_Normal_feedback->diff >   1000.0f )		DM_Normal_feedback->round_cnt --;
	
	//计算得到连续的编码器输出值
	DM_Normal_feedback->engle = DM_Normal_feedback->raw_pos + DM_Normal_feedback->round_cnt * 1432.394f;
	
	DM_Normal_feedback->vel = uint_to_float(v_int, V_MIN, V_MAX, 12);  			    			// (-45.0,45.0)速度
	DM_Normal_feedback->tor = uint_to_float(t_int, T_MIN, T_MAX, 12);  			    			// (-18.0,18.0)扭矩
	DM_Normal_feedback->Tmos  = (float)(RX_data[6]);														  		//驱动上MOS的平均温度，单位℃?J
	DM_Normal_feedback->Tcoil = (float)(RX_data[7]);														  		//电机内部线圈的平均温度，单位℃?J
}

/***@role: 达妙电机的使能函数***/
void DM_Motor_Mode_enable(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id)
{
	uint16_t id = ID + Mode_id;
	uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};					//电机使能指令
	canx_bsp_send_data(hcan,id,Data_Enable,8);																		    //发送使能命令
	osDelay(30);																																		  //延迟，先让电机使能好
}

/***@role: 保存达妙电机零点位置的函数***/
void DM_save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t mode_id)
{
	uint8_t Zero_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};          //电机记录零点的指令
	uint16_t id = ID + mode_id;																												//计算对应电机和模式ID的和
	canx_bsp_send_data(hcan, id, Zero_Data, 8);																				//发送指令
	osDelay(10);																																		  //延迟，先让电机零点记录好
}

/***@role:	达妙电机的失能函数***/
void DM_Motor_Mode_disable(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id)
{
	uint16_t id = ID + Mode_id;
	uint8_t Data_Disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	canx_bsp_send_data(hcan,id,Data_Disable,8);
	osDelay(10);
}

/***@role:	达妙电机的清除错误标志的函数***/
void DM_Motor_clean_erro(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t Mode_id)
{
	uint16_t id = ID + Mode_id;
	uint8_t Data_Clean[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
	canx_bsp_send_data(hcan,id,Data_Clean,8);
	osDelay(10);
}

/***@role:  达妙电机MIT模式下的控制函数***/
void DM_MIT_Control(CAN_HandleTypeDef* hcan, uint16_t ID, float _pos, 
												 float _vel,float _KP, float _KD, float _torq)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint8_t  Tx_Data[8];								                                //数据发送存储
	uint16_t id = ID + DM_MIT_MODE;                                     
	                                                                    
	float _Pos = (_pos * 0.0174532925f);							//0.0174532925f = PI/180
	                                                                    
	pos_tmp = float_to_uint(_Pos , P_MIN , P_MAX , 16);                 
	vel_tmp = float_to_uint(_vel , V_MIN , V_MAX , 12);                 
	kp_tmp  = float_to_uint(_KP  , KP_MIN, KP_MAX, 12);                 
	kd_tmp  = float_to_uint(_KD  , KD_MIN, KD_MAX, 12);                 
	tor_tmp = float_to_uint(_torq, T_MIN , T_MAX , 12);                 //注意这是前馈扭矩不是目标扭矩
	
	Tx_Data[0] = (pos_tmp >> 8);
	Tx_Data[1] =  pos_tmp;
	Tx_Data[2] = (vel_tmp >> 4);
	Tx_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	Tx_Data[4] =  kp_tmp;
	Tx_Data[5] = (kd_tmp >> 4);
	Tx_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	Tx_Data[7] =  tor_tmp;
	
	canx_bsp_send_data(hcan, id, Tx_Data, 8);
}

/***@role： 达妙电机速度控制模式下的控制代码***/
void DM_Speed_Control(CAN_HandleTypeDef* hcan,uint16_t ID,float _vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t Tx_Data[4];
	
	id = ID + DM_SPEED_MODE;
	vbuf=(uint8_t*)&_vel;
	
	Tx_Data[0] = *vbuf;
	Tx_Data[1] = *(vbuf + 1);
	Tx_Data[2] = *(vbuf + 2);
	Tx_Data[3] = *(vbuf + 3);
	
	canx_bsp_send_data(hcan, id, Tx_Data, 4);
}

/***@role： 达妙电机位置速度控制模式下的控制代码***/
void DM_Pos_Speed_Control(CAN_HandleTypeDef* hcan, uint16_t ID, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t Tx_Data[8];
	float Pos = (pos * 0.0174532925f);			//0.0174532925f = PI/180
	
	id = ID + DM_POS_MODE;
	pbuf=(uint8_t*)&Pos;
	vbuf=(uint8_t*)&vel;
	
	Tx_Data[0] = *pbuf;
	Tx_Data[1] = *(pbuf + 1);
	Tx_Data[2] = *(pbuf + 2);
	Tx_Data[3] = *(pbuf + 3);
	
	Tx_Data[4] = *vbuf;
	Tx_Data[5] = *(vbuf + 1);
	Tx_Data[6] = *(vbuf + 2);
	Tx_Data[7] = *(vbuf + 3);
	
	canx_bsp_send_data(hcan, id, Tx_Data, 8);
}

//========================================================================================================================//

//===========================================本末电机有关函数如下=========================================================//

/***@role： 本末电机的使能函数***/
void BM_Motor_enable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
	uint16_t id = 0x038;
	uint8_t Data_Enable[8]={0};																	//电机存储使能指令的数组
	switch(ID)																									//选择对应ID的电机进行使能指令的赋值
	{
		case 1: Data_Enable[0] = 0x02;	break;
		case 2: Data_Enable[1] = 0x02;	break;
		case 3: Data_Enable[2] = 0x02;	break;
		case 4: Data_Enable[3] = 0x02;	break;
		case 5: Data_Enable[4] = 0x02;	break;
		case 6: Data_Enable[5] = 0x02;	break;
		case 7: Data_Enable[6] = 0x02;	break;
		case 8: Data_Enable[7] = 0x02;	break;
	}
	canx_bsp_send_data(hcan,id,Data_Enable,8);									//发送使能命令
	osDelay(1);																								//延迟，先让电机使能好
}

/***@roel: 本末电机的失能函数***/
void BM_Motor_disenable(CAN_HandleTypeDef* hcan,uint8_t ID)
{
	uint16_t id = 0x38;
	uint8_t Data_Disenable[8]={0};															//电机存储失能指令的数组
	switch(ID)																									//选择对应ID的电机进行失能指令的赋值
	{
		case 1: Data_Disenable[0] = 0x01;	break;
		case 2: Data_Disenable[1] = 0x01;	break;
		case 3: Data_Disenable[2] = 0x01;	break;
		case 4: Data_Disenable[3] = 0x01;	break;
		case 5: Data_Disenable[4] = 0x01;	break;
		case 6: Data_Disenable[5] = 0x01;	break;
		case 7: Data_Disenable[6] = 0x01;	break;
		case 8: Data_Disenable[7] = 0x01;	break;
	}
	canx_bsp_send_data(hcan,id,Data_Disenable,8);								//发送失能命令
	osDelay(1);																									//延迟，先让电机失能好
}

/***@role: 本末电机反馈报文解析函数***/
void get_BM_Motor_data(BM_Feedback* BM_feedback)
{
	BM_feedback->v_speed = ((int16_t)((RX_data[0]<<8) | RX_data[1])) * 0.01f;
	
	BM_feedback->I_electric = ((int16_t)((RX_data[2]<<8) | RX_data[3]) * 0.01f);
	BM_feedback->torque = BM_feedback->I_electric * 0.8486562542f;		// 电流/1.414*1.2 = 力矩
	
	BM_feedback->Real_Angle = ((int16_t)((RX_data[4]<<8) | RX_data[5]) * 0.010986328f);
	
	BM_feedback->diff = BM_feedback->Real_Angle - BM_feedback->last_angle;
	if(BM_feedback->diff < (-150.0f))		BM_feedback->round_cnt ++;
	if(BM_feedback->diff >   150.0f )		BM_feedback->round_cnt --;
	
	BM_feedback->ecd_angle = BM_feedback->Real_Angle + BM_feedback->round_cnt * 360.0f;
	BM_feedback->U_voltage = ((int16_t)((RX_data[6]<<8) | RX_data[7]) * 0.1);
	BM_feedback->last_angle = BM_feedback->Real_Angle;
}

/***
 *@role: 	本末电机修改电机ID的指令函数
***/
void BM_Motor_ID_change(CAN_HandleTypeDef* hcan,uint8_t Now_ID,uint8_t target_ID)
{
	BM_Motor_disenable(hcan,Now_ID);															  			//修改电机的参数需要先给电机失能
	
	uint16_t id = 0x036;
	uint8_t Change_ID_Data[8]={0x00,0x2A,0x00,0x00,0x00,0x00,0x00,0x00};	//电机存储修改ID指令的数组
	
	Change_ID_Data[0] = 0x00 + Now_ID;															  		//现在的ID
	Change_ID_Data[2] = 0x00 + target_ID;														  		//目标ID
	
	canx_bsp_send_data(hcan,id,Change_ID_Data,8);													//发送命令
	osDelay(1);	
	
	id = 0x039;
	
	Change_ID_Data[0] = 0x01;																							//存入flash中																							//存入flash中
	for(uint8_t i = 1;i < 8;i ++)
	{
		Change_ID_Data[i] = 0x00;
	}
	
	canx_bsp_send_data(hcan,id,Change_ID_Data,8);													//发送命令
	osDelay(1);	
	
	BM_Motor_enable(hcan,target_ID);															 				//修改完之后重新给电机使能
}

/***
 *@role:	本末电机模式切换函数
***/
void BM_Motor_mode_change(CAN_HandleTypeDef* hcan,uint8_t Now_ID,uint8_t BM_mode)
{
	BM_Motor_disenable(hcan,Now_ID);																			//修改电机的参数需要先给电机失能
	
	uint16_t id = 0x036;
	uint8_t Change_mode_Data[8]={0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00};//电机存储修改模式指令的数组
	
	Change_mode_Data[0] = 0x00 + Now_ID;																	//要修改模式电机的ID号
	Change_mode_Data[2] = BM_mode;														        		//目标模式
	
	canx_bsp_send_data(hcan,id,Change_mode_Data,8);											  //发送命令
	osDelay(1);	
	
	BM_Motor_enable(hcan,Now_ID);																					//修改完之后重新给电机使能
}

/***
 *@role:	本末电机零点保存函数
***/
void BM_Motor_zero_save(CAN_HandleTypeDef* hcan,uint8_t Now_ID)
{
	BM_Motor_disenable(hcan,Now_ID);																			//修改电机的参数需要先给电机失能
	osDelay(3);
	
	uint16_t id = 0x039;
	uint8_t Save_zero_Data[8]={0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};	//电机存储修改模式指令的数组
	
	canx_bsp_send_data(hcan,id,Save_zero_Data,8);											  	//发送命令
	osDelay(5);
	
	BM_Motor_enable(hcan,Now_ID);																					//修改完之后重新给电机使能
	osDelay(3);
}

/***
 *@role:	本末电机flash参数存储
***/
void BM_Motor_flash_save(CAN_HandleTypeDef* hcan)
{
	uint16_t id = 0x039;
	
	uint8_t flash_Data[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//存入flash中
	
	canx_bsp_send_data(hcan,id,flash_Data,8);													//发送命令
	osDelay(5);	
}

//========以上为本末电机部分功能实现的函数，还有更高级的功能指令代码没有编写，例如零点的保存等等，望后来人整理============//


