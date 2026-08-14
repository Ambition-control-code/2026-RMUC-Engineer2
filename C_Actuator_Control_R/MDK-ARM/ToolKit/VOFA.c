#include "VOFA.h"
#include "usart.h"
#include "stdarg.h"

extern UART_HandleTypeDef huart1;

/***@role:	对应VOFA+串口调试软件使用的函数，需要改成justfloat那种格式使用***/
static uint8_t tx_buffer[VOFA_MAX_DATA * 4 + 4];
void VOFA_Print(uint8_t number,...)
{
    if(number > VOFA_MAX_DATA || number == 0)   return;

    va_list args;//声明参数指针
    va_start(args, number);//初始化可变参数列表

    //一个简易的共用体来转换float和字节
    union
    {
        float float_data;
        uint8_t bytes[4];   //共用体一个float类型要四个八字节
    } converter;

    uint32_t buffer_index = 0;

    // 遍历所有参数，存入缓冲区
    for (uint8_t i = 0; i < number; i++) 
    {
			//强转，因为使用可变参数在C语言中会默认将float类型升为double
			converter.float_data = (float)va_arg(args, double);//获取当前参数并移位至下一个
			
			tx_buffer[buffer_index++] = converter.bytes[0];
			tx_buffer[buffer_index++] = converter.bytes[1];
			tx_buffer[buffer_index++] = converter.bytes[2];
			tx_buffer[buffer_index++] = converter.bytes[3];
    }

    va_end(args);//清理工作，结束可变参数访问
    
    // 添加VOFA帧尾
    tx_buffer[buffer_index++] = 0x00;
    tx_buffer[buffer_index++] = 0x00;
    tx_buffer[buffer_index++] = 0x80;
    tx_buffer[buffer_index++] = 0x7f;

    // 发送数据	*注意使用这个DMA发送数据量很大的话记得加一些延时保证能把数据完整的发完发出去(osDelay就可以)*
    HAL_UART_Transmit_DMA(&huart1, tx_buffer, buffer_index);
}


