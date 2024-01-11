/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
  Description: header file for the Accelerometer module ADXL375
*/
#ifndef ADXL375_DRIVER_H
#define ADXL375_DRIVER_H

#include "mcu.h"

#define ADXL375_DEVID		                  0x00
#define ADXL375_DEVID_ID			            0xe5

#define ADXL375_INT_ENABLE		            0x2E  // Interrupt enable control
#define ADXL375_INT_MAP		                0x2F  // Interrupt enable control
#define ADXL375_INT_SOURCE		            0x30  // Interrupt mapping control

#define ADXL375_DATA_FORMAT		            0x31
#define ADXL375_DATA_FORMAT_FIXED		      0x0B	// these bits must be set to 1 
#define ADXL375_DATA_FORMAT_SELF_TEST	    7
#define ADXL375_DATA_FORMAT_SPI		        6
#define ADXL375_DATA_FORMAT_SPI_3_WIRE    1
#define ADXL375_DATA_FORMAT_SPI_4_WIRE		0
#define ADXL375_DATA_FORMAT_INT_INVERT	  5
#define ADXL375_DATA_FORMAT_JUSTIFY		    2

#define AO_ADXL375_BW_RATE		            0x2C
#define AO_ADXL375_BW_RATE_LOW_POWER		  4
#define AO_ADXL375_BW_RATE_RATE		        0
#define AO_ADXL375_BW_RATE_RATE_3200	  	0xF
#define AO_ADXL375_BW_RATE_RATE_1600		  0xE
#define AO_ADXL375_BW_RATE_RATE_800		    0xD
#define AO_ADXL375_BW_RATE_RATE_400		    0xC
#define AO_ADXL375_BW_RATE_RATE_200		    0xB
#define AO_ADXL375_BW_RATE_RATE_100		    0xA
#define AO_ADXL375_BW_RATE_RATE_50		    0x9
#define AO_ADXL375_BW_RATE_RATE_25		    0x8
#define AO_ADXL375_BW_RATE_RATE_12_5		  0x7
#define AO_ADXL375_BW_RATE_RATE_6_25		  0x6
#define AO_ADXL375_BW_RATE_RATE_3_13		  0x5
#define AO_ADXL375_BW_RATE_RATE_1_56		  0x4
#define AO_ADXL375_BW_RATE_RATE_0_78		  0x3
#define AO_ADXL375_BW_RATE_RATE_0_39		  0x2
#define AO_ADXL375_BW_RATE_RATE_0_20		  0x1
#define AO_ADXL375_BW_RATE_RATE_0_10		  0x0

#define ADXL375_OFSX			                0x1E
#define ADXL375_OFSY			                0x1F
#define ADXL375_OFSZ			                0x20

#define AO_ADXL375_POWER_CTL		          0x2D
#define AO_ADXL375_POWER_CTL_LINK		      5
#define AO_ADXL375_POWER_CTL_AUTO_SLEEP	  4
#define AO_ADXL375_POWER_CTL_MEASURE		  3
#define AO_ADXL375_POWER_CTL_SLEEP		    2
#define AO_ADXL375_POWER_CTL_WAKEUP		    0
#define AO_ADXL375_POWER_CTL_WAKEUP_8			0
#define AO_ADXL375_POWER_CTL_WAKEUP_4			1
#define AO_ADXL375_POWER_CTL_WAKEUP_2			2
#define AO_ADXL375_POWER_CTL_WAKEUP_1			3

#define ADXL375_X_REG_DATAX0              0x32
#define ADXL375_X_REG_DATAX1              0x33
#define ADXL375_Y_REG_DATAY0              0x34
#define ADXL375_Y_REG_DATAY1              0x35
#define ADXL375_Z_REG_DATAZ0              0x36
#define ADXL375_Z_REG_DATAZ1              0x37

#define ADXL375_MEASURE                   0x08

typedef struct ADXL375_data
{
  int16_t x;
  int16_t y;
  int16_t z;
} ADXL375_data;

SPI_TypeDef ADXL375_SPI;


#define ADXL375_SELF_TEST_SAMPLES	       10
#define ADXL375_SELF_TEST_SETTLE	       4
#define MIN_LSB_G	                       18.4
#define MAX_LSB_G	                       22.6
#define SELF_TEST_MIN_G	                 5.0
#define SELF_TEST_MAX_G	                 6.8
#define MIN_SELF_TEST	((int32_t) (MIN_LSB_G * SELF_TEST_MIN_G * ADXL375_SELF_TEST_SAMPLES + 0.5))

#pragma region Public

/**
  @brief Initialization of the ADXL375 Accelerometer module
  @note This function should be called before attempting to read data from the accelerometer.
  @return Success/Failure
*/
int8_t ADXL375_init(SPI Spi);

/**
  @brief Get data from the ADXL375 Accelerometer module
  @note
  @param data ptr to ADXL375_data struct for returning data
  @return Success/Failure
*/
ADXL375_data ADXL375_get_data(ADXL375_data *data);

#pragma endregion Public

#pragma region Private
void ADXL375_reg_write();

void ADXL375_self_test();
#pragma endregion Private

#endif /* ADXL375_DRIVER_H */