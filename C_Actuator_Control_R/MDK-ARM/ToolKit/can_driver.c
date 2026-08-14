#include "can_driver.h"

uint8_t can_buff[CAN_BUFFER_SIZE] = {0};
can_fifo_buffer_t can_rx_buffer;
uint8_t can_rx_buff[CAN_BUFFER_SIZE];

can_fifo_buffer_t can_tx_buffer;
uint8_t can_tx_buff[CAN_BUFFER_SIZE];

/**
************************************************************************
* @brief:      	can_driver_init(void)
* @param[in]:   void
* @retval:     	void
* @details:    	CAN设备初始化
************************************************************************
**/
void can_driver_init(void)
{
	can_bsp_init();
}
/**
************************************************************************
* @brief:      	canx_send_data: 用户驱动层发送函数
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	CAN总线发送数据
************************************************************************
**/
void canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	canx_bsp_send_data(hcan, id, data, len);
}

/**
************************************************************************
* @brief:      	can_para_init(void)
* @param[in]:   void
* @retval:     	void
* @details:    	数据结构体初始化
************************************************************************
**/
void can_para_init(void)
{
	can_tx_buffer.data = can_tx_buff;
	can_tx_buffer.size = CAN_BUFFER_SIZE;
	can_tx_buffer.read_index = 0;
	can_tx_buffer.write_index = 0;
	
	can_rx_buffer.data = can_rx_buff;
	can_rx_buffer.size = CAN_BUFFER_SIZE;
	can_rx_buffer.read_index = 0;
	can_rx_buffer.write_index = 0;
}
/**
************************************************************************
* @brief:      	can_serial_available(void)
* @param[in]:   void
* @retval:     	ch: 字节数据
* @details:    	从缓存区读取一个字节
************************************************************************
**/
uint8_t can_serial_read_char(void)
{
    uint8_t ch = 0;
    ch = can_rx_buffer.data[can_rx_buffer.read_index];
    can_rx_buffer.read_index = (can_rx_buffer.read_index + 1) % can_rx_buffer.size;
    return ch;
}
/**
************************************************************************
* @brief:      	can_serial_read(uint8_t *buffer, uint16_t length)
* @param[in]:   buffer: 数据
* @param[in]:   length: 缓冲区数据长度
* @retval:     	i: 数据个数
* @details:    	从缓存区读取数据
************************************************************************
**/
uint16_t can_serial_read(uint8_t *buffer, uint16_t length)
{
	uint16_t i = 0;

	for (i = 0; i < length; i++) {
		buffer[i] = can_rx_buffer.data[can_rx_buffer.read_index];
		can_rx_buffer.read_index = (can_rx_buffer.read_index + 1) % can_rx_buffer.size;
	}
	return i;
}
/**
************************************************************************
* @brief:      	can_serial_available(void)
* @param[in]:   void
* @retval:     	void
* @details:    	从缓存区读取数据长度
************************************************************************
**/
uint16_t can_serial_available(void)
{
	uint16_t len = 0;
	uint16_t can_rx_buffer_read_index = can_rx_buffer.read_index;
	uint16_t can_rx_buffer_write_index = can_rx_buffer.write_index;

	if (can_rx_buffer_read_index > can_rx_buffer_write_index) {
		len = can_rx_buffer.size + can_rx_buffer_write_index - can_rx_buffer_read_index;
	}
	else if (can_rx_buffer_read_index  < can_rx_buffer_write_index) {
		len = can_rx_buffer_write_index - can_rx_buffer_read_index;
	}
	return len;
}
/**
************************************************************************
* @brief:      	can_tx_buf
* @param[in]:   buffer: 数据
* @param[in]:   length: 缓冲区数据长度
* @param[in]:   tx_buffer: 发送结构体句柄
* @retval:     	i: 数据个数
* @details:    	把待发送的数据写入缓冲区
************************************************************************
**/
uint16_t can_tx_buf(uint8_t *buffer, uint16_t length, can_fifo_buffer_t *tx_buffer)
{
	uint16_t i = 0;

	for (i = 0; i < length; i++) {
		tx_buffer->data[tx_buffer->write_index] = buffer[i];
		tx_buffer->write_index = (tx_buffer->write_index + 1) % tx_buffer->size;
	}
	return i;
}
/**
************************************************************************
* @brief:      	can_serial_write(uint8_t *buffer, uint16_t length)
* @param[in]:   buffer: 数据
* @param[in]:   length: 缓冲区数据长度
* @retval:     	i: 数据个数
* @details:    	把待发送的数据写入缓冲区
************************************************************************
**/
uint16_t can_serial_write(uint8_t *buffer, uint16_t length)
{
	uint16_t i = 0;
	i = can_tx_buf(buffer, length, &can_tx_buffer);
	return i;
}
/**
************************************************************************
* @brief:      	can_tx_available(can_fifo_buffer_t * tx_buffer)
* @param[in]:   tx_buffer: 发送结构体句柄
* @retval:     	len: 缓冲区数据长度
* @details:    	判断缓冲区数据长度
************************************************************************
**/
uint16_t can_tx_available(can_fifo_buffer_t *tx_buffer)
{
	uint16_t len = 0;
	if (tx_buffer->read_index > tx_buffer->write_index) {
		len = tx_buffer->size + tx_buffer->write_index - tx_buffer->read_index;
	}
	else if (tx_buffer->read_index  < tx_buffer->write_index) {
		len = tx_buffer->write_index - tx_buffer->read_index;
	}
	return len;
}


