//=========该文件中的滑膜控制器是借鉴于复旦大学和四川大学的开源，看起来的话川大的滑膜有着更多的可能性======================
//==川大的滑膜更加的吃操作，更需要建模，复旦的更简单易于上手一点，不过两份代码都是我从C++移植过来的，出现问题也是有可能的==
//==测试下来确实有拟合的效果，复旦的位控效果更好一些==
#include "SMC.h"
#include "math.h"
#include "MY_math.h"

//=====================内部实现函数===================
/***@role：	饱和函数***/
static float FD_SMC_Sat(float y)
{
	if(fabsf(y) <= 1.0f)	return y;
	else	return (float)Pos_or_Neg(y);
}
//====================================================

//++++++++++++++++++++++++++复旦大学滑膜控制器开源算法+++++++++++++++++++++++++++++++++++++++++++++++++++
/***@role：	滑膜控制器初始化****/
void FD_SMC_Init(SMC_HandleTypeDef_FD *hsmc, float C, float K, float dead, float u_max, float J, float epsilon)
{
	hsmc->C = C;
	hsmc->K = K;
	hsmc->dead = dead;
	hsmc->u_max = u_max;
	hsmc->J = J;
	hsmc->epsilon = epsilon;
	
	hsmc->angle = 0.0f;
	hsmc->ang_vel = 0.0f;
	hsmc->u = 0.0f;
	
	hsmc->ref = 0.0f;
	hsmc->error = 0.0f;
	hsmc->error_last = 0.0f;
	hsmc->dref = 0.0f;
	hsmc->ddref = 0.0f;
	hsmc->refl = 0.0f;
	hsmc->s = 0.0f;
}

/***@role:	滑膜控制器计算控制算法***/
float FD_SMC_Tick(SMC_HandleTypeDef_FD *hsmc, float set, float angle_now, float angle_vel)
{
	// 读取参数
	hsmc->ref = set;
	hsmc->angle = angle_now;
	hsmc->ang_vel = angle_vel;
	hsmc->error = hsmc->angle - hsmc->ref;

	// 目标值一阶、二阶导数
	hsmc->ddref = (hsmc->ref - hsmc->refl) - hsmc->dref;
	hsmc->dref  = (hsmc->ref - hsmc->refl);

	if(fabsf(hsmc->error) < hsmc->dead)// 误差下限处理
	{
		hsmc->u = 0.0f;
		return hsmc->u;
	}

	hsmc->s = hsmc->C * hsmc->error + (hsmc->ang_vel - hsmc->dref);// 滑模面

	// 控制律
	hsmc->u = hsmc->J * (hsmc->ddref - hsmc->C * (hsmc->ang_vel - hsmc->dref) 
										 - hsmc->epsilon * FD_SMC_Sat(hsmc->s) - hsmc->K * hsmc->s);

	// 控制量限幅
	if(hsmc->u >  hsmc->u_max)	hsmc->u =  hsmc->u_max;
	if(hsmc->u < -hsmc->u_max)	hsmc->u = -hsmc->u_max;

	hsmc->refl = hsmc->ref;// 更新上一次目标值

	return hsmc->u;
}
//+++++++++++++++++++++++++++++++++++++++++复旦大学滑膜控制器算法+++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++四川大学滑膜控制器算法++++++++++++++++++++++++++++++++++++++++++++++

//=============================内部实现函数====================================
static void CD_SMC_OutContinuation(SMC_Handle_CD *h)
{
	/* 目的：参数在线修改时，缩放积分项，减少控制输出突变 */
	if(!h) return;

	/* 原代码条件：if(K!=0 && c2!=0) 才做缩放这里加一点保护，避免除 0 */
	if(h->smc.param.K != 0.0f && h->smc.param_last.K != 0.0f)
	{
		if(h->smc.param.c2 != 0.0f && h->smc.param_last.c2 != 0.0f) 
		{
			h->smc.error.p_error_integral = (h->smc.param_last.K / h->smc.param.K) *
																			(h->smc.param_last.c2 / h->smc.param.c2) *
																			(h->smc.error.p_error_integral);
		}
		if(h->smc.param.c != 0.0f && h->smc.param_last.c != 0.0f)
		{
			h->smc.error.v_error_integral =	(h->smc.param_last.K / h->smc.param.K) *
																			(h->smc.param_last.c / h->smc.param.c) *
																			(h->smc.error.v_error_integral);
		}
	}
	
	h->smc.param_last = h->smc.param;
}

static float CD_SMC_Sat(SMC_Handle_CD *h, float s)
{
	/* y = s/epsilon，若 |y|<=limit 输出 y，否则输出 sign(y) */
	if(!h)	return 0.0f;

	/* 防止 epsilon=0 导致除 0 */
	if(h->smc.param.epsilon == 0.0f)	return Pos_or_Neg(s);

	float y = s / h->smc.param.epsilon;

	if(fabsf(y) <= h->smc.astrict)	return y;
	else	return Pos_or_Neg(y);
}
//=================================================================================

/***@role:	滑膜控制器初始化函数***/
void CD_SMC_Init(SMC_Handle_CD *h)
{
	if(!h) return;
	
	h->smc.param.J = 0.0f;
	h->smc.param.K = 0.0f;
	h->smc.param.c = 0.0f;
	h->smc.param.c1 = 0.0f;
	h->smc.param.c2 = 0.0f;
	h->smc.param.p = 0.0f;
	h->smc.param.q = 0.0f;
	h->smc.param.beta = 0.0f;
	h->smc.param.epsilon = 0.0f;
	
	h->smc.param_last = h->smc.param;
	
	h->smc.flag  = EXPONENT;
	h->smc.u_max = 0.0f;
	h->smc.astrict = 0.0f;
	
	h->smc.u = 0.0f;
	h->smc.s = 0.0f;
	
	h->smc.error.tar_now = 0.0f;
	h->smc.error.tar_last = 0.0f;
	h->smc.error.tar_differential = 0.0f;
	h->smc.error.tar_differential_last = 0.0f;
	h->smc.error.tar_differential_second = 0.0f;
	
	h->smc.error.p_error = 0.0f;
	h->smc.error.v_error = 0.0f;
	h->smc.error.p_error_integral = 0.0f;
	h->smc.error.v_error_integral = 0.0f;
	
	h->smc.error.pos_error_eps = 0.0f;
	h->smc.error.vol_error_eps = 0.0f;
	
	h->smc.error.pos_get = 0.0f;
	h->smc.error.vol_get = 0.0f;
	
	h->smc.error.error_last = 0.0f;
}

/***@role:	指数/幂次趋近律参数设置***/
void CD_SMC_SetParam_ExponentPowerVel(SMC_Handle_CD *h,
																			float J, float K, float c,
																			float epsilon, float astrict, float u_max,
																			Rmode_CD flag, float pos_eps)
{
	if(!h) return;
	
	h->smc.param.J = J;
	h->smc.param.K = K;
	h->smc.param.c = c;
	h->smc.param.epsilon = epsilon;
	
	h->smc.error.pos_error_eps = pos_eps;
	
	h->smc.flag = flag;
	h->smc.u_max = u_max;
	h->smc.astrict = astrict;
	
	CD_SMC_OutContinuation(h);
}

/***@role：	终端滑模(TFSMC)参数设置***/
void CD_SMC_SetParam_TFSMC(SMC_Handle_CD *h,
													 float J, float K, float p, float q, float beta,
													 float epsilon, float astrict, float u_max,
													 Rmode_CD flag, float pos_eps)
{
	if(!h) return;
	
	h->smc.param.J = J;
	h->smc.param.K = K;
	h->smc.param.p = p;
	h->smc.param.q = q;
	h->smc.param.beta = beta;
	h->smc.param.epsilon = epsilon;
	
	h->smc.error.pos_error_eps = pos_eps;
	
	h->smc.flag = flag;
	h->smc.u_max = u_max;
	h->smc.astrict = astrict;
	
	CD_SMC_OutContinuation(h);
}

/***@role：	积分滑模(EISMC)参数设置***/
void CD_SMC_SetParam_EISMC(SMC_Handle_CD *h,
													 float J, float K, float c1, float c2,
													 float epsilon, float astrict, float u_max,
													 Rmode_CD flag, float pos_eps)
{
	if(!h) return;
	
	h->smc.param.J = J;
	h->smc.param.K = K;
	h->smc.param.c1 = c1;
	h->smc.param.c2 = c2;
	h->smc.param.epsilon = epsilon;
	
	h->smc.error.pos_error_eps = pos_eps;
	
	h->smc.flag = flag;
	h->smc.u_max = u_max;
	h->smc.astrict = astrict;
	
	CD_SMC_OutContinuation(h);
}

/***@role：	位置环误差更新（用于位置控制）***/
void CD_SMC_ErrorUpdate_Pos(SMC_Handle_CD *h, float target, float pos_now, float vol_now)
{
	if(!h) return;

	h->smc.error.tar_now = target;

	h->smc.error.tar_differential = (h->smc.error.tar_now - h->smc.error.tar_last) / CD_SAMPLE_PERIOD;

	h->smc.error.tar_differential_second =
						(h->smc.error.tar_differential - h->smc.error.tar_differential_last) / CD_SAMPLE_PERIOD;

	/* 位置/速度误差（与你原代码保持一致：p_error=pos-target，v_error=vol-target_dot）*/
	h->smc.error.p_error = pos_now - target;
	h->smc.error.v_error = vol_now - h->smc.error.tar_differential;
	
	h->smc.error.p_error_integral += h->smc.error.p_error * CD_SAMPLE_PERIOD;
	
	h->smc.error.tar_last = h->smc.error.tar_now;
	h->smc.error.tar_differential_last = h->smc.error.tar_differential;
}

/***@role:	速度环误差更新（用于速度控制）***/
void CD_SMC_ErrorUpdate_Vel(SMC_Handle_CD *h, float target, float vol_now)
{
	if(!h) return;

	h->smc.error.tar_now = target;

	h->smc.error.tar_differential = (h->smc.error.tar_now - h->smc.error.tar_last) / CD_SAMPLE_PERIOD;

	/* 速度环误差：v_error = vol_now - target */
	h->smc.error.v_error = vol_now - h->smc.error.tar_now;
	
	h->smc.error.v_error_integral += h->smc.error.v_error * CD_SAMPLE_PERIOD;
	
	h->smc.error.tar_last = h->smc.error.tar_now;
}

/***@role：	清除所有误差和积分项***/
void CD_SMC_Clear(SMC_Handle_CD *h)
{
	if(!h) return;
	
	h->smc.error.tar_now = 0.0f;
	h->smc.error.tar_last = 0.0f;
	h->smc.error.tar_differential = 0.0f;
	h->smc.error.tar_differential_last = 0.0f;
	h->smc.error.tar_differential_second = 0.0f;
	
	h->smc.error.p_error = 0.0f;
	h->smc.error.v_error = 0.0f;
	h->smc.error.p_error_integral = 0.0f;
	h->smc.error.v_error_integral = 0.0f;
	
	h->smc.error.pos_error_eps = 0.0f;
	h->smc.error.vol_error_eps = 0.0f;
	
	h->smc.error.pos_get = 0.0f;
	h->smc.error.vol_get = 0.0f;
	
	h->smc.error.error_last = 0.0f;
	
	h->smc.u = 0.0f;
	h->smc.s = 0.0f;
}

/***@role：	仅清除积分项（用于抗积分饱和）***/
void CD_SMC_IntegralClear(SMC_Handle_CD *h)
{
	if(!h) return;
	h->smc.error.v_error_integral = 0.0f;
	h->smc.error.p_error_integral = 0.0f;
}

/***
 *@role：	控制量计算函数
 *@role：	根据当前误差和选定的控制算法计算控制输出
 *@EXPONENT：	线性关系滑模面， 指数趋近率 -> 适用于Yaw(推荐)
 *@POWER：		线性关系滑模面， 幂次趋近率 -> 适用于Yaw(较推荐)
 *@TFSMC：		快速终端滑模控制 -> 适用于Yaw(较不推荐)
 *@VELSMC：		比例积分关系滑模面， 指数趋近率 -> 适用于摩擦轮、底盘电机等速度控制场景	//速度控制，只有一个反馈值
 *@EISMC：		比例积分关系滑模面， 指数趋近率 -> 适用于Pitch，拨弹轮
***/
float CD_SMC_Calculate(SMC_Handle_CD *h)
{
	float u = 0.0f;
	float fun = 0.0f;
	
	if(!h) return 0.0f;

	switch(h->smc.flag)
	{
		case EXPONENT: /* 线性滑模面 + 指数趋近率 */
		{
			if(fabsf(h->smc.error.p_error) < h->smc.error.pos_error_eps)
			{
				h->smc.error.p_error = 0.0f;
				h->smc.u = 0.0f;
				return 0.0f;
			}
			
			h->smc.s = h->smc.param.c * h->smc.error.p_error + h->smc.error.v_error;
			fun = CD_SMC_Sat(h, h->smc.s);

			u = h->smc.param.J * ((-h->smc.param.c * h->smc.error.v_error) - h->smc.param.K * h->smc.s
																																		 - h->smc.param.epsilon * fun );
    }break;
		
		case POWER: /* 线性滑模面 + 幂次趋近率 */
		{
			if(fabsf(h->smc.error.p_error) < h->smc.error.pos_error_eps)
			{
				h->smc.error.p_error = 0.0f;
				h->smc.u = 0.0f;
				return 0.0f;
			}

			h->smc.s = h->smc.param.c * h->smc.error.p_error + h->smc.error.v_error;
			fun = CD_SMC_Sat(h, h->smc.s);

			u = h->smc.param.J * ((-h->smc.param.c * h->smc.error.v_error) - h->smc.param.K * h->smc.s
														 -h->smc.param.K * powf(fabsf(h->smc.s),h->smc.param.epsilon) * fun);
		}break;
		
		case TFSMC:
		{
			if(fabsf(h->smc.error.p_error) < h->smc.error.pos_error_eps)
			{
				h->smc.error.p_error = 0.0f;
				h->smc.u = 0.0f;
				return 0.0f;
			}
		
			/* pos_pow = |e|^(q/p) * sign(e) */
			float pos_pow = powf(fabsf(h->smc.error.p_error),(h->smc.param.q / h->smc.param.p));
			if(h->smc.error.p_error <= 0.0f)	pos_pow = -pos_pow;

			h->smc.s = h->smc.param.beta * pos_pow + h->smc.error.v_error;
			fun = CD_SMC_Sat(h, h->smc.s);

			if(h->smc.error.p_error != 0.0f)
			{
				u = h->smc.param.J *(h->smc.error.tar_differential_second
														-h->smc.param.K * h->smc.s
														-h->smc.param.epsilon * fun
														-h->smc.error.v_error *((h->smc.param.q * h->smc.param.beta) * pos_pow)
																									 /(h->smc.param.p * h->smc.error.p_error));
			}
			else	u = 0.0f;
		} break;

		case VELSMC: /* PI 滑模面 + 指数趋近律，速度控制 */
		{
			h->smc.s = h->smc.error.v_error + h->smc.param.c * h->smc.error.v_error_integral;
			fun = CD_SMC_Sat(h, h->smc.s);

			u = h->smc.param.J * (h->smc.error.tar_differential
													-(h->smc.param.c * h->smc.error.v_error)
													- h->smc.param.K * h->smc.s
													- h->smc.param.epsilon * fun );
		} break;

		case EISMC: /* PI(位置积分) 滑模面 + 指数趋近律，位置控制 */
		{
			if(fabsf(h->smc.error.p_error) < h->smc.error.pos_error_eps)
			{
				h->smc.error.p_error = 0.0f;
				h->smc.u = 0.0f;
				return 0.0f;
			}

			h->smc.s = h->smc.param.c1 * h->smc.error.p_error
																 + h->smc.error.v_error
																 + h->smc.param.c2 * h->smc.error.p_error_integral;

			fun = CD_SMC_Sat(h, h->smc.s);

			u = h->smc.param.J * ((-h->smc.param.c1 * h->smc.error.v_error)
														 -h->smc.param.c2 * h->smc.error.p_error
														 -h->smc.param.K * h->smc.s
														 -h->smc.param.epsilon * fun );
		}break;
		default:	u = 0.0f;	break;
	}

	h->smc.error.error_last = h->smc.error.p_error;

	/* 输出限幅 */
	if(u >  h->smc.u_max)	u =  h->smc.u_max;
	if(u < -h->smc.u_max)	u = -h->smc.u_max;

	h->smc.u = u;
	
	return u;
}
//+++++++++++++++++++++++++++++++++++++++++四川大学滑膜控制器算法+++++++++++++++++++++++++++++++++++++++++++++++


