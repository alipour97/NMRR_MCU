/*
 * adc.c
 *
 *  Created on: Nov 28, 2024
 *      Author: alipo
 */


#include "main.h"
#include "global.h"
#include "ad717x.h"

void send_adc_data();

void adc_to_buf(uint32_t new_val)
{
//	time_buff[adc_buff_idx / 2] = TIM.Instance->CNT;

//	if(adc_buff_idx == 0 && new_val < 8000000)
//	{
//		int a = 0;
//	}
//	if(adc_buff_idx == 0 && adc_channel_offset > 0)
//	{
//		adc_channel_offset -= 2;
//		return;
//	}
	if(adc_channel_offset == 0)
	{
		time_buff[adc_buff_idx / 2] = TIM.Instance->CNT;
		adc_buff[adc_buff_idx / 2] = new_val;
	}
	else
	{
		tq_time_buff[adc_buff_idx / 2] = TIM.Instance->CNT;
		tq_buff[adc_buff_idx / 2] = new_val;
	}

	adc_buff_idx ++;
	if(adc_buff_idx >= ADC_BUFFER_SIZE * ENABLED_CHANNELS)
	{
		send_adc_data();
		adc_buff_idx = 0;
//		TIM.Instance->CNT = 0;
	}
}

void send_adc_data()
{
	int array_length = sizeof(uint32_t) * ADC_BUFFER_SIZE;

	sprintf((char*)tx_buffer, "{fdb,\r\n");
	memcpy(tx_buffer + 7, (uint8_t*) time_buff, array_length);
	memcpy(tx_buffer + 7 + array_length, (uint8_t*) adc_buff, array_length);
	memcpy(tx_buffer + 7 + 2 * array_length, (uint8_t*) tq_time_buff, array_length);
	memcpy(tx_buffer + 7 + 3 * array_length, (uint8_t*) tq_buff, array_length);
	sprintf((char*)tx_buffer+ 7 + 4 * array_length, ",end}\r\n");

	HAL_UART_Transmit_DMA(PC_UART, tx_buffer, 4 * array_length + 14);
//	HAL_UART_Transmit(PC_UART, (uint8_t*)"{fb,", 4, 10);
//	HAL_UART_Transmit(PC_UART, (uint8_t*)time_buff, sizeof(uint32_t) * ADC_BUFFER_SIZE, 100);
//	HAL_UART_Transmit(PC_UART, (uint8_t*)adc_buff, sizeof(uint32_t) * ADC_BUFFER_SIZE, 100);
//	HAL_UART_Transmit(PC_UART, (uint8_t*)",end}\r\n", 8, 10);
}

uint32_t read_adc_data()
{
	uint8_t Tx = 0x47;
	uint8_t Rx[4] = {0};
	HAL_SPI_Transmit(SPI, &Tx, 1, 10);
	HAL_SPI_Receive(SPI, Rx, 4, 100);
	uint32_t receivedData = 0;
	for(int i = 0; i < 4; i++) {
		receivedData <<= 8;
		receivedData += Rx[i];
	}
	return receivedData;
}

void adc_start(int start)
{
	if(start)
	{
		adc_buff_idx = 0;
		TIM.Instance->CNT = 0;
		spi_status = SENDING;
	}
	else
	{
		adc_buff_idx = 0;
		TIM.Instance->CNT = 0;
		spi_status = IDLE;
		adc_sm = ADC_IDLE;
	}

}
