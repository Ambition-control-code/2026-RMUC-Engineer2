#include "filter.h"
#include "math.h"
#include "MY_math.h"

/***@role: 滑动窗口滤波函数
 ***@advantage： 稳定，易于理解，简单好用
 ***@disadvantage： 使响应变慢，对内存消耗也大（如果是小内存的MCU，C板这些应该不至于），初始化也慢
***/
WindowFilter_Parameter WindowFilter_BigPitch = {20};
WindowFilter_Parameter WindowFilter_Yaw = {20};
WindowFilter_Parameter WindowFilter_SmollPitch = {25};
WindowFilter_Parameter WindowFilter_BigRoll = {10};
WindowFilter_Parameter WindowFilter_EndPitch = {10};
WindowFilter_Parameter WindowFilter_EndRoll = {15};
WindowFilter_Parameter WindowFilter_SmollRoll = {10};

float WindowFilter(WindowFilter_Parameter *Window_Struct,float inPut)
{
	float outPut = 0;
  
  Window_Struct -> buf[Window_Struct -> index ++] = inPut;
  
  if(Window_Struct -> index >= Window_Struct -> len)	{Window_Struct -> index = 0;}
      
  for(uint8_t i = 0;i < Window_Struct -> len;i ++)
  {
		outPut += Window_Struct -> buf[i];
  }
  
  outPut /= Window_Struct -> len;
  
  return outPut;
}

/***
 * @role	一阶卡尔曼滤波器初始化函数
 * @role	一阶卡尔曼滤波计算函数
 * @advantage	能够根据噪声特性自动调整滤波强度
 * @disadvantage	如果模型不准确或参数设置不当，可能导致滤波发散,同时初始状态和协方差的选择会影响收敛速度
 * @remember	Q:越大表示系统变化越快滤波器对新测量值响应越快,越小表示系统模型越精确滤波器更相信预测值
 * @remember	R:越大表示测量越不可靠滤波器更相信预测值,越小表示测量越精确滤波器更相信测量值
 * @remember	Q_MIN = 0.0001f		Q_MAX = 1.0f		R_MIN = 0.01f		R_MAX = 10.0f
 *
 * void kalman_init(KalmanFilter_Parameter* Kalman_Struct, float q, float r, float init_x, float init_p)
 * @brief 初始化卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 * @param q 过程噪声协方差
 * @param r 测量噪声协方差
 * @param init_x 初始状态值
 * @param init_p 初始估计误差协方差
 *
 * float kalman_update(KalmanFilter_Parameter* Kalman_Struct, float input)
 * @brief 卡尔曼滤波迭代计算
 * @param kf 卡尔曼滤波器指针
 * @param measurement 测量值
 * @return 滤波后的估计值
***/
void kalman_init(KalmanFilter_Parameter* Kalman_Struct, float q, float r, float init_x, float init_p)
{
	Kalman_Struct->q = q;
	Kalman_Struct->r = r;
	Kalman_Struct->x = init_x;
	Kalman_Struct->p = init_p;
	Kalman_Struct->k = 0.0f;
}

float kalman_update(KalmanFilter_Parameter* Kalman_Struct, float input)
{
	// 预测步骤
	Kalman_Struct->p = Kalman_Struct->p + Kalman_Struct->q;                               // 更新估计误差协方差
	
	// 更新步骤
	Kalman_Struct->k = Kalman_Struct->p / (Kalman_Struct->p + Kalman_Struct->r);          // 计算卡尔曼增益
	Kalman_Struct->x = Kalman_Struct->x + Kalman_Struct->k * (input - Kalman_Struct->x);  // 更新状态估计
	Kalman_Struct->p = (1 - Kalman_Struct->k) * Kalman_Struct->p;                         // 更新估计误差协方差
	
	return Kalman_Struct->x;
}

/***@role:	一阶低通滤波函数
 ***@advantage：	稳定性好，不会出现不稳定振荡
 ***@disadvantage：	α值的选择对性能影响很大
 ***@remember：	α值越接近1，响应越快，但滤波效果越差；反之越接近0.响应越慢滤波效果越好
 ***@remember：	一般情况alpha = 采样周期 / (时间常数 + 采样周期)，但本人不懂时间常数是如何求来，所以一般都是经验调alpha
 ***@BRB：	可以当作斜坡缓冲来使用，能使阶跃信号变得平滑
***/
LowpassFilter_Parameter Filter_Claw_angle = {0};

float Lowpass_Filter(LowpassFilter_Parameter* Lowpass_Struct,float input,float alpha,float dead)
{
	Lowpass_Struct->Input = input;
	Lowpass_Struct->Output = ((alpha * Lowpass_Struct->Input) + ((1 - alpha) * Lowpass_Struct->Last_Output));
	
	if(fabs(Lowpass_Struct->Input - Lowpass_Struct->Last_Output) < dead)	Lowpass_Struct->Output = Lowpass_Struct->Input;
	
	Lowpass_Struct->Last_Output = Lowpass_Struct->Output;
	
	return Lowpass_Struct->Output;
}

/***
 *@role:	微分追踪器初始化函数	void TD_Init(TD_t *td, float r, float h0)
 *@role:	微分追踪器的工作函数	float TD_Calculate(TD_t *td, float input)
 *
 *@remember:	r越大，跟踪更快，但可能超调/振荡，越慢跟踪变慢，更平滑，推荐范围【10-1000】
 *@remember:	h0越大，更平滑，但延迟增加，越小延迟减小，但噪声敏感，推荐范围【0.001-0.1】
***/
void TD_Init(TD_t *td, float r, float h0)
{
	td->r = r;
	td->h0 = h0;
	
	td->x = 0;
	td->dx = 0;
	td->ddx = 0;
	td->last_dx = 0;
	td->last_ddx = 0;
}

float TD_Calculate(TD_t *td, float input)
{
	static float d, a0, y, a1, a2, a, fhan;
	
	td->dt=0.001f;
	
	td->Input = input;
	
	d  = td->r * td->h0 * td->h0;
	a0 = td->dx * td->h0;
	y  = td->x - td->Input + a0;
	a1 = sqrt(d * (d + 8 * MY_ABS(y)));
	a2 = a0 + Pos_or_Neg(y) * (a1 - d) / 2;
	a  = (a0 + y) * (Pos_or_Neg(y + d) - Pos_or_Neg(y - d)) / 2 + a2 * (1 - (Pos_or_Neg(y + d) - Pos_or_Neg(y - d)) / 2);
	fhan = -td->r * a / d * (Pos_or_Neg(a + d) - Pos_or_Neg(a - d)) / 2
	       -td->r * Pos_or_Neg(a) * (1 - (Pos_or_Neg(a + d) - Pos_or_Neg(a - d)) / 2);
	
	td->ddx = fhan;
	td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
	td->x  += (td->dx + td->last_dx) * td->dt / 2;
	
	td->last_ddx = td->ddx;
	td->last_dx  = td->dx;
	
	return td->x;
}



/***@role: 斜坡缓冲函数,本人自己写的逻辑,使用的if可能不优雅，也有别的实现方式，好使就行***/
float Slope_Buffer(float Set_Angle,float real_Angle,float Turn_Speed)
{
	float Change_Angle = Set_Angle;
	float Ramp_step = ((Set_Angle - real_Angle) / Turn_Speed);
	
	if(fabs(Ramp_step) < 10.0f)	
	{
		if((Set_Angle - real_Angle) > 0.0f)	Ramp_step = ( 10.0f);
		if((Set_Angle - real_Angle) < 0.0f)	Ramp_step = (-10.0f);
	}
	if(fabs(real_Angle - Set_Angle) > 10.0f)
	{
		if(Set_Angle > real_Angle)
		{
			Change_Angle = (real_Angle + Ramp_step);
			if(Change_Angle > Set_Angle)
			{
				Change_Angle = Set_Angle;
			}
		}
		else if(Set_Angle < real_Angle)
		{
			Change_Angle = (real_Angle + Ramp_step);
			if(Change_Angle < Set_Angle)
			{
				Change_Angle = Set_Angle;
			}
		}
	}
	else	Change_Angle = Set_Angle;
	return Change_Angle;
}


