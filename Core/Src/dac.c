/*
 * dac.c
 *
 *  Created on: Jan 3, 2025
 *      Author: alipo
 */

#include "main.h"
#include "global.h"



uint32_t dac_read(uint8_t addr, int print)
{
	uint8_t Tx[3] = {addr,0,0};
	uint8_t Rx[3] = {0};
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
	delay_us(1);
	HAL_SPI_Transmit(DAC_SPI, Tx, 3, 100);
	delay_us(1);
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
	delay_us(1);
	HAL_SPI_Receive(&hspi2, Rx, 3, 100);
	delay_us(1);
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	uint32_t reg_data = (Rx[0] << 16) | (Rx[1] << 8) | Rx[2];
	if(print)
	{
		char hexString[35];  // Buffer to store "0x" + 4 hex digits + null terminator
		sprintf(hexString, "{inf,\r\ndac,0x%02x%02x%02x,end}\r\n", Rx[0], Rx[1], Rx[2]);  // Format as hex string with "0x" prefix
		send_string(hexString);
	}
	return reg_data;

}

void dac_write(uint8_t addr, uint16_t value)
{
	uint8_t Tx[3] = {addr,(value & 0xFF00) >> 8, (value & 0x00FF) >> 0};
	for(int i = 0; i < 1; i++)
	{
		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
		delay_us(1);
		HAL_SPI_Transmit(DAC_SPI, Tx, 3, 100);
		delay_us(1);
		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	}
}

void dac_update(float v)
{
	uint16_t val = (uint16_t) (((v+10) / 20) * (1 << 16));
	dac_write(0x01, val);
}

void dac_init(int num)
{
	//send full reset
	for(int i = 0; i < 3; i++)
		dac_write(0x0f, 0x00);
	for(int i = 0; i < 3; i++)
		dac_write(0x04, 0x08); // Set Control Register to mid scale and -+10V so V_out = 0

//	uint32_t Rx = 0;
//	for(int i = 0; i < 5; i++)
//	{
//		Rx = dac_read(0x0c,0); // Read Control Register 3 times
//	}
//
//	if(((Rx & 0xffff) == 8))
//	{
//		char hexString[35];  // Buffer to store "0x" + 4 hex digits + null terminator
//		sprintf(hexString, "{inf,\r\ndac,0x%04x,end}\r\n", (uint16_t)Rx);  // Format as hex string with "0x" prefix
//		send_string(hexString);
//		return;
//	}
//	else
//	{
//		send_string("{inf,\r\nCouldn't Init DAC, try again,end}\r\n");
//		if(num < DAC_MAX_TRY)
//			dac_init(num + 1);
//		else
//		{
//			char hexString[56];  // Buffer to store "0x" + 4 hex digits + null terminator
//			sprintf(hexString, "{inf,\r\nCouldn't Init DAC, after %d tries,end}\r\n", num);  // Format as hex string with "0x" prefix
//			send_string(hexString);
//		}
//
//	}


}

void new_pattern(uint16_t length, uint8_t* pattern_ptr)
{
	adc_start(0);
	DAC_EN = 0;
	DAC_idx = 0;
	DAC_length = length;
	memset(DAC_pattern, 0, DAC_PATTERN_SIZE);
	length = (length < DAC_BULK_SIZE) ? length : DAC_BULK_SIZE; // if length is more than BULK_SIZE, just copy the BULK
	memcpy(DAC_pattern, pattern_ptr, length * sizeof(float));
	DAC_idx += length;

//	send_string("{dac,\r\n");
	char message[30];
	sprintf(message, "{dac,\r\n%d,end}\r\n", DAC_idx);
	send_string(message);
}

void bulk_pattern(uint16_t length, uint8_t* pattern_ptr)
{
	memcpy(DAC_pattern + DAC_idx, pattern_ptr, length * sizeof(float));
	DAC_idx += length;

//	send_string("{dac,end}\r\n");
	char message[30];
	sprintf(message, "{dac,\r\n%d,end}\r\n", DAC_idx);
	send_string(message);
	if(DAC_idx >= DAC_length)
	{
		DAC_idx = 0;
		DAC_EN = 1;
		adc_start(1);
	}
}
// Check if pattern loads correctly -> To-Do
void check_pattern()
{
}
