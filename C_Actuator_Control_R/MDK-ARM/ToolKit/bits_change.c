#include "bits_change.h"

/**
 * @brief  将两个8位无符号整数（uint8_t）组成的数组转换为一个16位有符号整数（int16_t）
*/
int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}

/**
 * @brief  将四个8位无符号整数（uint8_t）组合成一个32位无符号整数（uint32_t）
*/
uint32_t bit8TObit32(uint8_t change_info[4])
{
	union
	{
    uint32_t bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}

/**
 * @brief  返回的是输入数组前4个字节(change_info[0]到change_info[3])按小端序组合成的32位值
 * @remeber 不要被这个函数名欺骗，这个函数将高位截断丢弃，只有前四个自己被使用
 * @know   从别处扒来的代码，这个是否正确有待商议，开源代码是如此使用，这里只做解释不做修改
*/
uint32_t bit8TObit64(uint8_t change_info[8])
{
 	union
	{
    uint64_t bit32;
		uint8_t  byte[4];
	}u64val;

  u64val.byte[0] = change_info[0];
  u64val.byte[1] = change_info[1];
  u64val.byte[2] = change_info[2];
  u64val.byte[3] = change_info[3];

	return u64val.bit32;
}

/**
 * @brief  将一个包含4个8位无符号整数（uint8_t）的数组转换为一个32位浮点数（float）
*/
float bit8TOfloat32(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}

/**
 * @brief  将一个16位有符号整数（int16_t）拆分成两个8位无符号整数（uint8_t），并返回指定索引位置的8位字节
*/
uint8_t bit16TObit8(uint8_t Index_need,int16_t bit16)
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.bit16 = bit16;
	return u16val.byte[Index_need];
}

/**
 * @brief  将一个32位无符号整数（uint32_t）拆分成4个8位无符号整数（uint8_t），并返回其中指定索引位置的8位值
*/
uint8_t bit32TObit8(uint8_t Index_need,uint32_t bit32)
{
	union
	{
    uint32_t  bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.bit32 = bit32;

	return u32val.byte[Index_need];
}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


