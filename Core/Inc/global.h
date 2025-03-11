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
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t uart_buffer[];
extern uint8_t tx_buffer[];
extern uint16_t uart_timeout;

extern uint32_t pos_time_buff[];
extern uint32_t pos_buff[];
extern uint32_t pos_window[];
extern uint32_t tq_window[];
extern uint32_t tq_buff[];
extern uint32_t tq_time_buff[];
extern volatile uint16_t adc_buff_idx;
extern volatile uint16_t window_buff_idx;
extern uint32_t adc_channel_offset;
extern volatile uint8_t ADC_EN;

extern float DAC_pattern[];
extern uint16_t DAC_idx;
extern uint16_t DAC_length;
extern volatile uint8_t DAC_EN;
extern volatile uint8_t DAC_last_bulk;

// function declarationCNY
void delay_us(uint32_t us);

void adc_start(int start);
void adc_to_buf(uint32_t new_val);
void adc_window();
uint32_t read_adc_data();
void dac_readreg(uint8_t addr);
void dac_writereg(uint8_t addr, uint16_t value);

uint32_t dac_read(uint8_t addr, int print);
void dac_write(uint8_t addr, uint16_t value);
void dac_update(float v);
void dac_init(int num);
void new_pattern(uint16_t length, uint8_t* pattern_ptr);
void bulk_pattern(uint16_t length, uint8_t* pattern_ptr);
void check_pattern();
//extern uint8_t spi_read_reg;

#define PC_UART &huart2
#define SPI &hspi1
#define DAC_SPI &hspi2
#define UART_BUFFER_SIZE 1024
#define ADC_BUFFER_SIZE 50
#define WINDOW_BUFFER_SIZE 10
#define DAC_PATTERN_SIZE 30 * 1000
#define DAC_BULK_SIZE 250
#define TIM htim2
#define ENABLED_CHANNELS 2
#define DAC_MAX_TRY 5
#define TS 0.001

enum SPI_STATUS {
  IDLE,
  TRIGGER,
  READING,
  WRITING,
  GETID,
  SPI_NOP,
  SENDING,
  CHANNEL_OFFSET
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
