#include "MY_math.h"

/***@role:	求绝对值的函数***/
float MY_ABS(float a)
{
	if(a<0) a=-a;
	return a;
}

/***@role:	求正负的函数***/
float Pos_or_Neg(float x)
{
	if(x>0)	
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}


