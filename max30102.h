/*
 * max30102.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Eng.Hazem
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

/**********************************INCLUDE LIBRARY*************************************/
#include "stm32g4xx_hal.h"
#include "filters.h"
#include "stdbool.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/// MAX30102 Register Address Map
///
/// Status
#define INT_STATUS_1			0x00
#define INT_STATUS_2			0x01
#define INT_ENABLE_1			0x02
#define INT_ENABLE_2			0x03
/// FIFO
#define FIFO_WRITE_PTR			0x04
#define FIFO_OVF_COUNTER		0x05
#define FIFO_READ_POINTER		0x06
#define FIFO_DATA				0x07
/// Configuration
#define FIFO_CONFIG				0x08
#define MODE_CONFIG				0x09
#define SPO2_CONFIG				0x0A
#define LED_PULSE_AMP_1			0x0C
#define LED_PULSE_AMP_2			0x0D
#define LED_MODE_CONTROL_1		0x11
#define LED_MODE_CONTROL_2		0x12
/// Die Temperature
#define DIE_TEMP_INTEGER		0x1F
#define DIE_TEMP_FRACTION		0x20
#define DIE_TEMP_CONFIG			0x21

/// Part ID
#define REVISION_ID				0xFE
#define PART_ID					0xFF



/***************************************MACROS DEFINATION*******************/
#define I2C_SLAVE_ID			0xAE
#define I2C_WRITE				0x00
#define I2C_READ				0x01

#define OPEN                   1
#define CLOSE                   0
#define DEBUG_MODE_UART    CLOSE

typedef struct{
	uint32_t redLedRaw;
	uint32_t irLedRaw;
}FIFO_LED_DATA;


typedef struct {
  bool pulseDetected;
  float heartBPM;
  float irCardiogram;
  float irDcValue;
  float redDcValue;
  float SpO2;
  uint32_t lastBeatThreshold;
  float dcFilteredIR;
  float dcFilteredRed;
}MAX30102;


typedef enum {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PULSE_STATE_MACHINE;

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

extern LEDCurrent IrLedCurrent;
#define RED_LED	1
#define IR_LED	2
/************************prototype*********************************************/
uint8_t Max30102_readRegister(uint8_t reg, uint8_t* value);
HAL_StatusTypeDef Max30102_writeRegister(uint8_t reg, uint8_t value);
void Max30102_Init(void);

FIFO_LED_DATA Max30102_readFifo(void);
MAX30102 Max30102_HeartRate_Value(FIFO_LED_DATA m_fifoData);
bool detectPulse(float sensor_value);
void balanceIntesities( float redLedDC, float IRLedDC );
uint32_t millis(void);
void  Max30102_setLedCurrent(uint8_t led, float currentLevel);
void  Max30102_resetFifo(void);
void print_Sensor_value(float x);
void Max30102_resetRegisters(void);
void uart_PrintString(char * str);
void uart_PrintFloat(float value);
void uart_PrintInt(unsigned int value, unsigned char base);
#endif /* INC_MAX30102_H_ */
