/*
 * handle_uart.cpp
 *
 *  Created on: Nov 10, 2024
 *      Author: alipo
 */



#include "main.h"
#include "global.h"
#include "ad717x.h"
extern struct spi_write spi_write_reg;

extern ad717x_dev *pad717x_dev;
//extern enum SPI_STATUS spi_status;
extern int check_command(void)
{

	if(strchr((char*)uart_buffer, ',') != NULL)
	{
		char *command;
		char *endptr = strchr((char*)uart_buffer, ',');
		int length = endptr - (char*)uart_buffer;
		command = (char*) malloc(length);
		strncpy(command, (char*) (uart_buffer+1), length);
		command[length-1] = '\0';
		handle_command(command);
		free(command);
		return 1;
	}
	else if(strlen((char*) uart_buffer) > 200){
		reset_uart();
	}
	return 0;
}

extern void send_string(const char *msg)
{
	HAL_UART_Transmit(PC_UART, (uint8_t *)msg, strlen(msg), 10);
}

extern void reset_uart(void)
{
	strncpy((char*)uart_buffer, "", UART_BUFFER_SIZE);
	HAL_UART_DMAStop(PC_UART);
	HAL_UART_Receive_DMA(PC_UART, uart_buffer, UART_BUFFER_SIZE);
}


extern void handle_command(char* txt_in)
{
	char *command = txt_in;
	char *addr_str = (char*)uart_buffer + strlen(command) + 2;
	if(!strcmp(command, "get_id")){
		spi_status = GETID;

	}
	else if(!strcmp(command, "getid"))
	{
		while(HAL_GPIO_ReadPin(DRY_GPIO_Port, DRY_Pin) == GPIO_PIN_SET);
		ad717x_st_reg *pReg;
//		ad717x_dev *pad717x_dev1 = NULL;
		AD717X_ReadRegister(pad717x_dev, 0x07);
		pReg = AD717X_GetReg(pad717x_dev, 0x07);
		char hexString[9];  // Buffer to store "0x" + 4 hex digits + null terminator
		sprintf(hexString, "0x%04x\r\n", (unsigned int)pReg->value);  // Format as hex string with "0x" prefix
		send_string(hexString);
	}
	else if(!strcmp(command, "getreg"))
	{
		if(strchr((char*)uart_buffer, '}') != NULL)
		{
			uint8_t addr = (uint8_t)strtoul(addr_str, NULL, 16);
//				ad717x_dev *pad717x_dev1 = NULL;
//				ad717x_st_reg *pReg;
			AD717X_ReadRegister(pad717x_dev, addr);
//				pReg = AD717X_GetReg(pad717x_dev, addr);
//				char hexString[12];  // Buffer to store "0x" + 4 hex digits + null terminator
//				sprintf(hexString, "%u:0x%04x\r\n", spi_read_reg, (unsigned int)pReg->value);  // Format as hex string with "0x" prefix
//				send_string(hexString);

		} else return;

	}
	else if(!strcmp(command, "writereg"))
	{
		char *endptr = strchr((char*)addr_str, ',');
		int length = endptr - (char*)addr_str;
		char *value_str = (char*)addr_str + length + 1;
		if(strchr((char*)uart_buffer, '}') != NULL)
		{
			uint8_t addr = (uint8_t)strtoul(addr_str, NULL, 16);
			uint32_t value = (uint32_t)strtoul(value_str, NULL, 16);

			ad717x_st_reg *pReg = AD717X_GetReg(pad717x_dev, addr);
			pReg->value = value;
			spi_write_reg.value = value;
			AD717X_WriteRegister(pad717x_dev, addr);
		} else return;
	}
	else if(!strcmp(command, "send"))
	{
		if(strchr((char*)uart_buffer, '}') != NULL)
		{
			if(spi_status != SENDING && adc_sm == ADC_IDLE)
			{
				spi_status = SENDING;
				adc_buff_idx = 0;
			}
			else
				spi_status = IDLE;
		} else return;
	}
	else{
		send_string("{Unknown msg,end}\r\n");
		send_string(command);
		reset_uart();
	}
	reset_uart();

}


int32_t no_os_spi_write_and_read(SPI_HandleTypeDef *desc,
				 uint8_t *data,
				 uint16_t bytes_number)

{
	HAL_SPI_Transmit(SPI, data, 1, 10);
	return 0;

}
