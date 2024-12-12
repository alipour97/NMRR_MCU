/*
 * global.h
 *
 *  Created on: Nov 10, 2024
 *      Author: alipo
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_


#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern uint8_t uart_buffer[];
extern uint8_t tx_buffer[];

extern uint32_t time_buff[];
extern uint32_t adc_buff[];
extern uint32_t tq_buff[];
extern uint16_t adc_buff_idx;


// function declarationCNY
void adc_to_buf(uint32_t new_val);
uint32_t read_adc_data();
//extern uint8_t spi_read_reg;

#define PC_UART &huart2
#define SPI &hspi1
#define UART_BUFFER_SIZE 1024
#define ADC_BUFFER_SIZE 50
#define TIM htim2

enum SPI_STATUS {
  IDLE,
  TRIGGER,
  READING,
  WRITING,
  GETID,
  SPI_NOP,
  SENDING
};

enum ADC_SM {
  ADC_IDLE,
  ADC_READING,
//  WRITING,
//  GETID,
  ADC_NOP,
//  SENDING
};


extern volatile enum SPI_STATUS spi_status;
extern volatile enum ADC_SM adc_sm;

//struct ad717x_dev;
//struct ad717x_st_reg;

//typedef struct ad717x_dev ad717x_dev;
//typedef struct ad717x_st_reg ad717x_st_reg;

//struct spi_read; // Declaration without definition
struct spi_read{
	uint8_t Tx;
	void *pReg;
};

struct spi_write{
	uint8_t Tx[8];
	uint32_t value;
	void *pReg;
};



#endif /* INC_GLOBAL_H_ */
