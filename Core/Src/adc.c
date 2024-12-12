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
	time_buff[adc_buff_idx] = TIM.Instance->CNT;
	adc_buff[adc_buff_idx++] = new_val;

	if(adc_buff_idx >= ADC_BUFFER_SIZE)
	{
		send_adc_data();
		adc_buff_idx = 0;
//		TIM.Instance->CNT = 0;
	}
}

void send_adc_data()
{

	sprintf((char*)tx_buffer, "{fdb,\r\n");
	memcpy(tx_buffer + 7, (uint8_t*) time_buff, sizeof(uint32_t) * ADC_BUFFER_SIZE);
	memcpy(tx_buffer + 7 + sizeof(uint32_t) * ADC_BUFFER_SIZE, (uint8_t*) adc_buff, sizeof(uint32_t) * ADC_BUFFER_SIZE);
	sprintf((char*)tx_buffer+ 7 + 2 * sizeof(uint32_t) * ADC_BUFFER_SIZE, ",end}\r\n");

	HAL_UART_Transmit_DMA(PC_UART, tx_buffer, 2 * sizeof(uint32_t) * ADC_BUFFER_SIZE + 14);
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
