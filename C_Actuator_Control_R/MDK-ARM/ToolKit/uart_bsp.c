#include "uart_bsp.h"
#include "Get_Referee_Data.h"
#include "remote.h"

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart      pointer to a UART_HandleTypeDef structure that contains
  *                    the configuration information for the specified USART Stream.  
	*                    【UART句柄指针，包含UART配置信息】
  * @param  SrcAddress pointer to The source memory Buffer address
	*                    【源地址（UART数据寄存器地址）】
  * @param  DstAddress pointer to The destination memory Buffer address
	*                    【第一个目标缓冲区地址】
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer
                       【第二个目标缓冲区地址】  
  * @param  DataLength The length of data to be transferred from source to destination
	                     【每个缓冲区的数据长度（字节数）】	
  * @retval none
	* @remember 以下的这个代码似乎是与串口通讯使用DMA有关，但这个对寄存器的直接使用羞涩难懂，建议仔细学习加深理解，但不强求
	* @role     这段代码实现了UART（通用异步收发器）使用DMA（直接内存访问）进行双缓冲接收的功能，并启用了IDLE中断
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	//这段代码实现了UART（通用异步收发器）使用DMA（直接内存访问）进行双缓冲接收的功能，并启用了IDLE中断
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;              //设置接收类型

  /* configuare the huart Receicve Size */
	//设置总接收长度为两个缓冲区之和（双缓冲总容量）
   huart->RxXferSize = DataLength * 2;                           //配置接收总长度
	
  /* Enable the DMA transfer for the receiver request */
	//在UART控制寄存器3(CR3)中使能DMA接收功能，当UART收到数据时自动触发DMA传输
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);                 //使能DMA接收请求

  /* Enalbe IDLE interrupt */
	//启用总线空闲中断，当UART线路空闲（无数据）时触发中断
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);                     //使能IDLE中断

  /* Disable DMA */
	//循环禁用DMA直到确认其真正停止（确保DMA处于可配置状态）
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);                          //安全禁用DMA
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
	//设置DMA外设地址寄存器(PAR)为UART数据寄存器地址（数据来源）
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;           //配置DMA源地址

  /* Configure the destination memory Buffer address */
	//设置DMA内存0地址寄存器(M0AR)为第一个接收缓冲区地址
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;          //配置第一个内存缓冲区

  /* Configure DMA Stream destination address */
	//设置DMA内存1地址寄存器(M1AR)为第二个接收缓冲区地址
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;    //配置第二个内存缓冲区

  /* Configure the length of data to be transferred from source to destination */
	//配置DMA数据传输数量寄存器(NDTR)为单个缓冲区长度
  huart->hdmarx->Instance->NDTR = DataLength;                    //设置传输数据量

  /* Enable double memory buffer */
	//在DMA控制寄存器(CR)中使能双缓冲模式(Double Buffer Mode)
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);            //启用双缓冲模式

  /* Enable DMA */
	//最后使能DMA，开始数据传输
  __HAL_DMA_ENABLE(huart->hdmarx);                               //启动DMA传输
}

/**
  * @brief  Configures the USART.
  * @role   串口DMA双缓冲区的初始化函数，具体如何使用请仔细学习。
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
  /* Starts the remote control multi_buffer DMA Transfer with interrupt enabled. */
	//-------该初始化是uart3的，目前我只使用uart6来作为自定义控制器的串口来接收图传信息，后续如果使用可以开启----------------------//
//	USART_RxDMA_MultiBufferStart(&huart3,(uint32_t *)&(huart3.Instance->DR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

	/* Starts the Referee multi_buffer DMA Transfer with interrupt enabled. */
	USART_RxDMA_MultiBufferStart(&huart6,(uint32_t *)&(huart6.Instance->DR),(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER USART6 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
                  reception buffer until which, data are available)
  * @role   处理USART6接收数据的回调函数，采用DMA双缓冲模式实现高效数据接收
  * @teach  USER_USART6_RxHandler 是一个静态回调函数，在DMA传输完成一半或全部完成时被触发，用于处理USART6接收到的数据。
            核心机制是DMA双缓冲循环模式，通过两个缓冲区交替接收数据
  * @retval None
 **/

extern uint32_t life_state_new;
extern uint32_t Zhou;
static void USER_USART6_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
  /* Current memory buffer used is Memory 0 */
	//检查DMA控制寄存器(CR)的CT位:(CT == RESET (0)：当前使用缓冲区0)(CT != RESET (1)：当前使用缓冲区1)
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)   //判断当前激活的缓冲区
  {
		//Disable DMA    禁用DMA以安全修改配置
		__HAL_DMA_DISABLE(huart->hdmarx);

		//切换CT位指向缓冲区1
		huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
		
    if(Size >= 10)
    {
      /* reset the receive count    重置DMA计数器（准备下一次接收）*/
			
			Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
			
			for(uint8_t i=0;i<30;i++)
			{
				custom_data.RX_custom_data[i] = Referee_System_Info.custom_robot_data.data[i];
			}
			if(remote_MyControl_flag == 0)	
			{
				if(Zhou<1500)	Control_angle_filter();
				else
				{
					Set_Arm_yaw = 0.0f;
					Big_Pitch = 70.0f;
					Smoll_Pitch = -70.0f;
				}
			}
			/*检测，如果是这两个帧头的话就是新遥控器的数据，不是裁判系统*/
			if(Referee_System_Info_MultiRx_Buf[0][0] == 0xA9 && Referee_System_Info_MultiRx_Buf[0][1] == 0x53)
			{
				life_state_new = 0;//生命标志清零，保证能稳定运行
				memcpy(VT13_use.temp_data,Referee_System_Info_MultiRx_Buf[0],21);
			}
			
			
			if(Referee_System_Info_MultiRx_Buf[0][1] == 0xA9 && Referee_System_Info_MultiRx_Buf[0][2] == 0x53)
			{
				life_state_new = 0;//生命标志清零，保证能稳定运行
				for(uint8_t i = 0; i < 20; i ++)
				{
					VT13_use.temp_data[i] = Referee_System_Info_MultiRx_Buf[0][i+1];
				}
			}
			
			Handle_VT03_Data(&New_VT03_data);
			
			__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
    }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
		//Disable DMA    禁用DMA
		__HAL_DMA_DISABLE(huart->hdmarx);

		//切换CT位指向缓冲区0
		huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
		
    if(Size >= 10)
    {
      /* reset the receive count    重置DMA计数器*/
			
			Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
			
			for(uint8_t i=0;i<30;i++)
			{
				custom_data.RX_custom_data[i] = Referee_System_Info.custom_robot_data.data[i];
			}
			if(remote_MyControl_flag == 0)	
			{
				if(Zhou<1500)	Control_angle_filter();
				else
				{
					Set_Arm_yaw = 0.0f;
					Big_Pitch = 70.0f;
					Smoll_Pitch = -70.0f;
				}
			}
			
			/*检测，如果是这两个帧头的话就是新遥控器的数据，不是裁判系统*/
			if(Referee_System_Info_MultiRx_Buf[1][0] == 0xA9 && Referee_System_Info_MultiRx_Buf[1][1] == 0x53)
			{
				life_state_new = 0;//生命标志清零，保证能稳定运行
				memcpy(VT13_use.temp_data,Referee_System_Info_MultiRx_Buf[1],21);
			}
			
			if(Referee_System_Info_MultiRx_Buf[1][1] == 0xA9 && Referee_System_Info_MultiRx_Buf[1][2] == 0x53)
			{
				life_state_new = 0;//生命标志清零，保证能稳定运行
				for(uint8_t i = 0; i < 20; i ++)
				{
					VT13_use.temp_data[i] = Referee_System_Info_MultiRx_Buf[1][i+1];
				}
			}
			
			Handle_VT03_Data(&New_VT03_data);
			
			__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
    }
  }
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
 if(huart->Instance == USART6)
	{
		USER_USART6_RxHandler(huart,Size);
	}

  /* reset the Reception Type */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------


