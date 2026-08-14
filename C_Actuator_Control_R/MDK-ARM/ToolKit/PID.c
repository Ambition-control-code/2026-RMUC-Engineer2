#include "PID.h"
#include "math.h"
#include "MY_math.h"

/***@role: 结构体再命名***/
PID_T Claw_angle = {0};
PID_T Claw_speed = {0};
/*************************/

/*****@role： PID的初始化函数*****/
void PID_Init(PID_T *pid,
	            float kp,float ki,												 //比例项，积分项
							float kd,float kf,       									 //微分项，前馈项
							float dead,float i_threshold,							 //死区，积分分离阈值
	            float poutmax,float ioutmax,							 //比例项输出最大值，积分项输出最大值
							float doutmax,float foutmax,							 //微分项输出最大值，前馈项输出最大值
	            float outmax)                              //PID计算输出最大值（限幅）
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kf = kf;
	pid->dead = dead;
	pid->i_threshold = i_threshold;
	
	pid->poutmax = poutmax;
	pid->ioutmax = ioutmax;
	pid->doutmax = doutmax;
	pid->foutmax = foutmax;
	pid->outmax = outmax;
	
	pid->err = 0;
	pid->integral = 0;
	pid->err_last = 0;
	pid->set_last = 0;
}

/***@role：   常规PID的计算函数***/
float PID_Cal(PID_T *pid, float get, float set)  											// get为实际值  set为目标值
{
	pid->set  = set;																										//设定值
	pid->real = get;																										//实际值
	pid->err = pid->set - pid->real;                                    //计算当前误差 = 设定值 - 实际值
	
	if(fabs(pid->err) < pid->dead)	pid -> err = 0;											//死区限制，当误差在死区以内时则认为没有误差
	
	//====================================比例项=============================================================
	pid->pout = pid->kp * pid->err;                                     //比例项输出 = Kp * 当前误差
	/*通过三元运算符实现双限幅，防止输出超限*/
	pid->pout = pid->pout <  pid->poutmax ? pid->pout :  pid->poutmax;  //上限幅
	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;  //下限幅
	
	//================================积分项(积分分离)========================================================
	if(fabs(pid->err) < pid->i_threshold)
	{
		pid->integral += pid->err;                                         //对误差进行积分（累加）
		
		pid->iout = pid->ki * pid->integral;                               //积分项输出 = Ki * 误差积分
		/*通过三元运算符实现双限幅，防止积分饱和*/
		pid->iout = pid->iout <  pid->ioutmax ? pid->iout :  pid->ioutmax; //上限幅
		pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax; //下限幅
	}
	else
	{
		pid->integral = 0;																								 // 误差大于阈值:远离目标值,清零积分(或保持不变)
		pid->iout = 0;
	}
	
	//====================================微分项=============================================================
	pid->dout = pid->kd * (pid->err - pid->err_last);                    //微分项输出=Kd*(当前误差-上次误差)
	/*通过三元运算符实现双限幅，抑制微分突变*/
	pid->dout = pid->dout <  pid->doutmax ? pid->dout :  pid->doutmax;   //上限幅
	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;   //下限幅
	
	//====================================前馈项=============================================================
	pid->fout = pid->kf * (pid->set - pid->set_last);										 //前馈项输出=kf*(当前设定-上次设定)
	/*通过三元运算符实现双限幅，抑制前馈突变*/
	pid->fout = pid->fout <  pid->foutmax ? pid->fout :  pid->foutmax;   //上限幅
	pid->fout = pid->fout > -pid->foutmax ? pid->fout : -pid->foutmax;   //下限幅
	
	//====================================输出项=============================================================
	/*合成输出与总限幅*/
	pid->out = pid->pout + pid->iout + pid->dout + pid->fout;            //总输出 = P + I + D + F
	/*对总输出进行限幅，确保输出在安全范围内*/
	pid->out = pid->out <  pid->outmax ? pid->out :  pid->outmax;        //上限幅      
	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;        //下限幅
	
	//==================================更新历史量============================================================
	pid->err_last = pid->err;                                            //保存上一次误差值
	pid->set_last = pid->set;																						 //保存上一次设定值
	
	//==================================返回输出值============================================================
	return pid->out;                                                     //返回PID计算值
}

/***@role：	增量式PID的计算函数****/
float  PID_Incremental(PID_T* Pid,float set ,float real)
{
	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = Pid->set - Pid->real;
	
	Pid->pout = Pid->kp * ( Pid->error[0] - Pid->error[1] );
	Pid->pout >  Pid->poutmax ? Pid->pout =  Pid->poutmax : 1;
	Pid->pout < -Pid->poutmax ? Pid->pout = -Pid->poutmax : 1;
	
	Pid->iout =	Pid->ki * Pid->error[0];
	Pid->iout >  Pid->ioutmax ? Pid->iout =  Pid->ioutmax : 1;
	Pid->iout < -Pid->ioutmax ? Pid->iout = -Pid->ioutmax : 1;

	Pid->dout =	Pid->kd * ( Pid->error[0] - 2 * Pid->error[1] + Pid->error[2] );
	Pid->dout >  Pid->doutmax ? Pid->dout =  Pid->doutmax : 1;
	Pid->dout < -Pid->doutmax ? Pid->dout = -Pid->doutmax : 1;

	Pid->error[2] = Pid->error[1];
	Pid->error[1] = Pid->error[0];
	
	Pid->out += Pid->pout + Pid->iout + Pid->dout;
	Pid->out >  Pid->outmax ? Pid->out =  Pid->outmax : 1;
	Pid->out < -Pid->outmax ? Pid->out = -Pid->outmax : 1;

	return Pid->out;	
}


