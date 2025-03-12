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

void adc_window()
{
	uint64_t pos = 0 , tq = 0;
	int i = 0;
	for(i = 0; i < window_buff_idx/2; i++)
	{
		pos += pos_window[i];
		tq += tq_window[i];
	}
	// if there is one data mismatch between pos and tq, add the latest data to window, but because in next window, the other one may be once more, I let it go for now ...
//	if(window_buff_idx % 2 == 1)
//	{
//		pos += pos_window[i];
//	}
	pos_time_buff[adc_buff_idx] = TIM.Instance->CNT;
	pos_buff[adc_buff_idx] = pos / i; // copy mean of pos window to pos_buff
	tq_buff[adc_buff_idx] = tq / i; // copy mean of tq window to tq_buff
	adc_buff_idx++;
	window_buff_idx = 0;
	current_pos = ((float)pos / (1 << 23) - 1) * 25;
	current_tq = ((float)tq / (1 << 23) - 1) * 25;
	if(!DAC_EN)
		init_motor_pos = current_pos;
	if(adc_buff_idx >= ADC_BUFFER_SIZE)
	{
		send_adc_data();
		adc_buff_idx = 0;

		if(DAC_last_bulk)
		{
			DAC_idx = 0;
			DAC_EN = 0;
			DAC_last_bulk = 0;
			delay_us(10000); // wait 10 ms
			send_string("{cmd,\r\nend_pattern,end}\r\n");
		}
//		TIM.Instance->CNT = 0;
	}
}

void adc_to_buf(uint32_t new_val)
{

	if(adc_channel_offset == 0)
	{
//		pos_time_buff[adc_buff_idx / 2] = TIM.Instance->CNT;
//		pos_buff[adc_buff_idx / 2] = new_val;
		pos_window[window_buff_idx / 2] = new_val;
	}
	else
	{
//		tq_time_buff[adc_buff_idx / 2] = TIM.Instance->CNT;
//		tq_buff[adc_buff_idx / 2] = new_val;
		tq_window[window_buff_idx / 2] = new_val;
	}
	window_buff_idx++;
//	adc_buff_idx ++;
//	if(adc_buff_idx >= ADC_BUFFER_SIZE * ENABLED_CHANNELS)
//	{
//		send_adc_data();
//		adc_buff_idx = 0;
////		TIM.Instance->CNT = 0;
//	}
}

void send_adc_data()
{
	int array_length = sizeof(uint32_t) * ADC_BUFFER_SIZE;

	sprintf((char*)tx_buffer, "{fdb,\r\n");
	memcpy(tx_buffer + 7, (uint8_t*) pos_time_buff, array_length);
	memcpy(tx_buffer + 7 + array_length, (uint8_t*) pos_buff, array_length);
	memcpy(tx_buffer + 7 + 2 * array_length, (uint8_t*) tq_time_buff, array_length);
	memcpy(tx_buffer + 7 + 3 * array_length, (uint8_t*) tq_buff, array_length);
	sprintf((char*)tx_buffer+ 7 + 4 * array_length, ",end}\r\n");

	HAL_UART_Transmit_DMA(PC_UART, tx_buffer, 4 * array_length + 14);

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
		ADC_EN = 1;
	}
	else
	{
		adc_buff_idx = 0;
		TIM.Instance->CNT = 0;
		spi_status = IDLE;
		adc_sm = ADC_IDLE;
		ADC_EN = 0;
	}

}
