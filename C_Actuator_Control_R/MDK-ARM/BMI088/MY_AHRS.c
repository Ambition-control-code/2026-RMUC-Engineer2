//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MY_AHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
//变量定义
volatile float twoKp = twoKpDef;																							// 2 * 比例增益（Kp）
volatile float twoKi = twoKiDef;																							// 2 * 积分增益（Ki）
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;		//用Ki缩放的积分误差项

/**********************************************************************************
@role： 这段代码是完整的AHRS（Attitude and Heading Reference System）更新函数，
        在原有IMU（加速度计+陀螺仪）基础上增加了磁力计数据，
			  用于实现全姿态解算（包括航向角）。
@centre： 增加了磁力计处理，解决IMU版本中航向角（Yaw）漂移问题。
**********************************************************************************/
//AHRS算法更新
void AHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	//如果磁力计测量无效，使用IMU算法（避免磁力计归一化中的错误导致解算错误）
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		AHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	//仅当加速度计测量有效时才计算反馈（避免加速度计归一化中的错误）
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		//归一化加速度计测量
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		//归一化磁力计测量
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

    //辅助变量避免重复计算
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];   

    /*地球磁场的参考方向*/
		//将磁力计数据旋转到世界坐标系
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		//计算水平面投影
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		/*估计重力和磁场的方向*/
		//估计重力方向（同IMU版本）
		halfvx = q1q3 - q0q2;							//重力向量x分量的一半
		halfvy = q0q1 + q2q3;							//重力向量y分量的一半
		halfvz = q0q0 - 0.5f + q3q3;			//重力向量z分量的一半
		// 估计地磁方向
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		/*误差是场矢量估计方向与实测方向的叉积之和*/
		//误差计算（融合重力+地磁）
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		/*计算和应用反馈，如果启用*/
		//反馈控制（同IMU版本）
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);//积分误差由ki缩放
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			//比例反馈（动态响应）
			gx += integralFBx;	//应用积分补偿
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	//防止整体清盘
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		//应用比例反馈
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	//四元数微分方程更新
	gx *= (0.5f * (1.0f / sampleFreq));			//预乘公因数【转换为角度增量×0.5】
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	//其余分量更新
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	//四元数归一化
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	//各分量归一化
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
//IMU算法更新
void AHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	//仅当加速度计测量有效时才计算反馈（避免加速度计归一化中的NaN）
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		//归一化加速度计测量
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;            										//归一化x分量
		ay *= recipNorm;            										//归一化y分量
		az *= recipNorm;            										//归一化z分量

		//估计重力方向和垂直于磁通量的矢量
		halfvx = q[1] * q[3] - q[0] * q[2];       			// 重力向量X分量的一半
		halfvy = q[0] * q[1] + q[2] * q[3];       			// 重力向量Y分量的一半
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];			// 重力向量Z分量的一半
	
		//误差是估计的重力方向和测量的重力方向的叉积之和【计算重力向量误差（叉积）】
		halfex = (ay * halfvz - az * halfvy);						//x轴误差
		halfey = (az * halfvx - ax * halfvz);						//y轴误差
		halfez = (ax * halfvy - ay * halfvx);						//z轴误差

		//计算和应用积分反馈，如果启用【积分反馈（消除静态漂移）】
		if(twoKi > 0.0f) 
		{
			//积分误差由Ki缩放
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);		//X积分项
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);		//y积分项
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);		//z积分项
			gx += integralFBx;																			//应用x积分反馈
			gy += integralFBy;  																		//应用y积分反馈
			gz += integralFBz;  																		//应用z积分反馈
		}
		else 
		{
			//禁用积分时清零
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
		//应用比例反馈
		gx += twoKp * halfex;			            //应用x比例补偿
		gy += twoKp * halfey;			            //应用y比例补偿
		gz += twoKp * halfez;			            //应用z比例补偿
	}
	//积分四元数的变化率【 四元数更新（陀螺仪数据积分）】
	gx *= (0.5f * (1.0f / sampleFreq));			//预乘公因数	//转换为角度增量 × 0.5
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	//四元数微分方程
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);			//w分量更新
	q[1] += (qa * gx + qc * gz - q[3] * gy);			//x分量更新
	q[2] += (qa * gy - qb * gz + q[3] * gx);			//y分量更新
	q[3] += (qa * gz + qb * gy - qc * gx); 				//z分量更新
	
	//正常化四元数【四元数归一化】
	/***@role： 防止数值误差累积导致四元数模长偏离1*****/
	/***@must：四元数必须为单位四元数才能正确表示旋转***/
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
//平方根倒数速算法
//见网站: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//====================================================================================================
// END OF CODE
//====================================================================================================

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
	quat[0] = 1.0f;
	quat[1] = 0.0f;
	quat[2] = 0.0f;
	quat[3] = 0.0f;
}

void AHRS_update(fp32 quat[4], fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
	AHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
	*yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
	*pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
	*roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

