#include "arm_control.h"
#include "arm_math.h"
#include "math.h"

// 转换常量
#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.295779513f

// 机械臂参数 (米)
#define L1 0.38f
#define L2 0.33f

RobotState_Deg robot_deg = {0};

/**
 * 向前平移函数 (全角度操作版)
 * @param dx_m : 向前移动距离 (米)
 * @return 0: 成功, -1: 超出范围
 */
int8_t moveForward_Degree(RobotState_Deg* deg, float dx_m)
{
	// 1. 角度转弧度
	float r1 = deg->d1 * DEG2RAD;
	float r2 = deg->d2 * DEG2RAD;
	
	// 2. 正运动学 (FK)：匹配你验证正确的坐标系
	// x = -L1*cos(d1) + L2*cos(d1 - d2)
	// y = -L1*sin(d1) + L2*sin(d1 - d2)
	float cur_x = -L1 * arm_cos_f32(r1) + L2 * arm_cos_f32(r1 - r2);
	float cur_y = -L1 * arm_sin_f32(r1) + L2 * arm_sin_f32(r1 - r2);
	
	// 3. 设定目标位置 (水平平移)
	float tar_x = cur_x + dx_m;
	float tar_y = cur_y;
	
	// 4. 逆运动学 (IK)
	float S_sq = tar_x * tar_x + tar_y * tar_y;
	float S;
	arm_sqrt_f32(S_sq, &S);
	
	// 物理极限检查
	if (S > (L1 + L2) || S < fabsf(L1 - L2)) return -1;
	
	// A. 计算中间变量 r2_math (对应小臂相对于大臂的折回角)
	float cos_r2_m = (S_sq - L1*L1 - L2*L2) / (2.0f * L1 * L2);
	if (cos_r2_m > 1.0f) cos_r2_m = 1.0f;
	if (cos_r2_m < -1.0f) cos_r2_m = -1.0f;
	
	// 取负值以维持“肘部向上”的姿态
	float r2_m = -acosf(cos_r2_m); 
	
	// B. 计算中间变量 r1_math (大臂指向)
	float alpha = atan2f(tar_y, tar_x);
	float beta = atan2f(L2 * arm_sin_f32(r2_m), L1 + L2 * arm_cos_f32(r2_m));
	float r1_m = alpha - beta;
	
	// 5. 映射回电机反馈系统
	float target_d1 = (r1_m * RAD2DEG) - 180.0f;
	float target_d2 = (-r2_m * RAD2DEG) - 180.0f;
	
	// 周期处理：确保 d1 在电机反馈量程内 (-360, 0)
	while(target_d1 > 0.0f) target_d1 -= 360.0f;
	while(target_d1 < -360.0f) target_d1 += 360.0f;
	
	// 6. 末端姿态补偿逻辑 (重点)
	// 根据描述，末端相对于地面的角度 θ = d1 - d2 + d3
	// 保持平移前后 θ 不变
	float total_orientation = deg->d1 - deg->d2 + deg->d3;
	
	deg->target_d1 = target_d1;
	deg->target_d2 = target_d2;
	// 逆推新的 target_d3 = θ_total - (target_d1 - target_d2)
	deg->target_d3 = total_orientation - (deg->target_d1 - deg->target_d2);
	
	return 0;
}

/**
 * 向上平移函数 (保持 X 不变)
 * @param dy_m : 向上移动距离 (米)，负数为向下
 * @return 0: 成功, -1: 超出范围
 */
int8_t moveUpward_Degree(RobotState_Deg* deg, float dy_m)
{
	// 1. 角度转弧度
	float r1 = deg->d1 * DEG2RAD;
	float r2 = deg->d2 * DEG2RAD;
	
	// 2. 正运动学 (FK)：使用已验证的坐标系
	// x = -L1*cos(d1) + L2*cos(d1 - d2)
	// y = -L1*sin(d1) + L2*sin(d1 - d2)
	float cur_x = -L1 * arm_cos_f32(r1) + L2 * arm_cos_f32(r1 - r2);
	float cur_y = -L1 * arm_sin_f32(r1) + L2 * arm_sin_f32(r1 - r2);
	
	// 3. 设定目标位置 (X轴不动，Y轴增加)
	float tar_x = cur_x; 
	float tar_y = cur_y + dy_m;
	
	// 4. 逆运动学 (IK)
	float S_sq = tar_x * tar_x + tar_y * tar_y;
	float S;
	arm_sqrt_f32(S_sq, &S);
	
	// 物理极限检查
//	if (S > (L1 + L2) || S < fabsf(L1 - L2)) return -1;
	
	// A. 计算中间变量 r2_math
	float cos_r2_m = (S_sq - L1*L1 - L2*L2) / (2.0f * L1 * L2);
	if (cos_r2_m > 1.0f) cos_r2_m = 1.0f;
	if (cos_r2_m < -1.0f) cos_r2_m = -1.0f;
	
	float r2_m = -acosf(cos_r2_m); 
	
	// B. 计算中间变量 r1_math
	float alpha = atan2f(tar_y, tar_x);
	float beta = atan2f(L2 * arm_sin_f32(r2_m), L1 + L2 * arm_cos_f32(r2_m));
	float r1_m = alpha - beta;
	
	// 5. 映射回电机反馈系统
	float target_d1 = (r1_m * RAD2DEG) - 180.0f;
	float target_d2 = (-r2_m * RAD2DEG) - 180.0f;
	
	// 周期处理
	while(target_d1 > 0.0f) target_d1 -= 360.0f;
	while(target_d1 < -360.0f) target_d1 += 360.0f;
	
	// 6. 末端姿态补偿逻辑
	// 保持 θ = d1 - d2 + d3 恒定
	float total_orientation = deg->d1 - deg->d2 + deg->d3;
	
	deg->target_d1 = target_d1;
	deg->target_d2 = target_d2;
	deg->target_d3 = total_orientation - (deg->target_d1 - deg->target_d2);
	
	return 0;
}
