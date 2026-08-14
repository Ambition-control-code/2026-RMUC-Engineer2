#ifndef __SMC_H__
#define __SMC_H__

//=========复旦大学滑膜控制器结构体========================//
typedef struct																						 //
{                                           							 //
	float C;		   // 滑膜面系数                           	 //
	float K;		   // 滑膜增益	越大响应越快              	 //
	float ref;         // 目标值                          	 //
	float dead;   	   // 死区                            	 //
	float u_max;       // 输出最大值                      	 //
	float J;           // 估计惯量                       	   //
	float epsilon;	   // 滑膜项增益	抑制超调，拖慢响应  	 //
																													 //
	float angle;       // 当前角度反馈                   	   //
	float ang_vel;     // 当前角速度反馈                  	 //
	float u;           // 控制输出                        	 //
																													 //
	// 内部变量                                          	   //
	float error;                                             //
	float error_last;                                        //
	float dref;        // 目标值一阶导                       //
	float ddref;       // 目标值二阶导                       //
	float refl;        // 上一次目标值                       //
	float s;           // 滑模面                             //
}SMC_HandleTypeDef_FD;                                     //
//=========================================================//

//==================================复旦大学滑膜控制器函数声明==================================================//
void FD_SMC_Init(SMC_HandleTypeDef_FD *hsmc, float C, float K, float dead, float u_max, float J, float epsilon);
float FD_SMC_Tick(SMC_HandleTypeDef_FD *hsmc, float set, float angle_now, float angle_vel);
//==============================================================================================================//

//===========四川大学宏定义================//
#define CD_SAMPLE_PERIOD 0.002f            //
#define CD_V_EORROR_INTEGRAL_MAX 2000.0f   //
#define CD_P_EORROR_INTEGRAL_MAX 2000.0f   //
//=========================================//

//=========================四川大学滑膜控制器结构体=======================================//
typedef enum																																							//
{                                                                            							//
	EXPONENT = 0,                                                                         	//
	POWER,                                                                                	//
	TFSMC,                                                                                	//
	VELSMC,                                                                               	//
	EISMC                                                                                 	//
}Rmode_CD;                                                                               	//
																																													//
typedef struct																																						//
{                                                                          								//
	float tar_now;                 // 当前目标值                                          	//
	float tar_last;                // 上一次目标值                                        	//
	float tar_differential;        // 目标值一阶导                                        	//
	float tar_differential_last;   // 上一次目标值一阶导                                  	//
	float tar_differential_second; // 目标值二阶导                                        	//
																																													//
	float pos_get;                 // 当前位置（本代码未更新，仅保留字段）                	//
	float vol_get;                 // 当前速度（本代码未更新，仅保留字段）                	//
																																													//
	float p_error;                 // 位置误差 = pos_now - target                         	//
	float v_error;                 // 速度误差（位置误差一阶导意义）或速度环误差          	//
																																													//
	float p_error_integral;        // 位置误差积分                                        	//
	float v_error_integral;        // 速度误差积分                                        	//
																																													//
	float pos_error_eps;           // 位置误差死区/精度                                   	//
	float vol_error_eps;           // 速度误差精度（本代码未使用）                        	//
	float error_last;              // 上一次误差（原代码用于记录 p_error）                	//
}RError_CD;                                                                               //
																																													//
typedef struct																																						//
{                                                                          								//
	float J;                                                                              	//
	float K;                                                                              	//
	float c;                                                                              	//
																																													//
	float c1;       // EISMC 参数                                                         	//
	float c2;       // EISMC 参数                                                         	//
																																													//
	float p;        // TFSMC 参数，正奇数 p>q                                             	//
	float q;        // TFSMC 参数，正奇数                                                	  //
	float beta;     // TFSMC 参数，正数                                                  		//
	float epsilon;  // 饱和函数边界层参数（也被用于趋近律参数）                           	//
}SlidingParam_CD;                                                                        	//
																																													//
typedef struct																																						//
{                                                                          								//
	float u;            // 控制器输出                                                     	//
	float s;            // 滑模面                                                         	//
																																													//
	SlidingParam_CD param;                                                                	//
	SlidingParam_CD param_last;                                                          		//
																																													//
	RError_CD error;                                                                      	//
	float u_max;        // 输出限幅                                                       	//
	Rmode_CD flag;      // 模式选择                                                       	//
	float astrict;      // 饱和函数上下限（|y|<=limit 时输出 y，否则输出 sign(y)）        	//
}Sliding_CD;                                                                             	//
																																													//
/* 句柄：等价于原 C++ 的 cSMC 对象 */                                                     //
typedef struct																																						//
{                                                                          								//
	Sliding_CD smc;                                                                       	//
}SMC_Handle_CD;                                                                          	//
//========================================================================================//

//========================四川大学函数声明================================================
void CD_SMC_Init(SMC_Handle_CD *h);
void CD_SMC_SetParam_ExponentPowerVel(SMC_Handle_CD *h,
																			float J, float K, float c,
																			float epsilon, float astrict, float u_max,
																			Rmode_CD flag, float pos_eps); 
void CD_SMC_SetParam_TFSMC(SMC_Handle_CD *h,
													 float J, float K, float p, float q, float beta,
													 float epsilon, float astrict, float u_max,
													 Rmode_CD flag, float pos_eps);
void CD_SMC_SetParam_EISMC(SMC_Handle_CD *h,
													 float J, float K, float c1, float c2,
													 float epsilon, float astrict, float u_max,
													 Rmode_CD flag, float pos_eps);
void CD_SMC_ErrorUpdate_Pos(SMC_Handle_CD *h, float target, float pos_now, float vol_now);
void CD_SMC_ErrorUpdate_Vel(SMC_Handle_CD *h, float target, float vol_now);
void CD_SMC_Clear(SMC_Handle_CD *h);
void CD_SMC_IntegralClear(SMC_Handle_CD *h);
float CD_SMC_Calculate(SMC_Handle_CD *h);
//========================================================================================

#endif


