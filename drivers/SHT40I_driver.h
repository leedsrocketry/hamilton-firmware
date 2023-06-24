/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: header file for the Temp/Humidity module SHT40I-AD1B-R2
*/
#ifndef SHT40I_DRIVER_H
#define SHT40I_DRIVER_H
#include "stdint.h"


/**
  @brief Header commands for the SHT40-AD1B-R2 sensor
  @warning Do not use header for extended periods of time. The heater is designed for a maximal duty cycle of less than 5% when it is periodically heated
*/
typedef enum Header {
	SensorHeader_200mWFor1s	 	= 0x39,		// 200mW @ 3.3V for 1s
	SensorHeader_200mWFor0p1s = 0x32,		// 200mW @ 3.3V for 0.1s
	SensorHeader_110mWFor1s	 	= 0x2F,		// 110mW @ 3.3V for 1s
	SensorHeader_110mWFor0p1s	= 0x24,		// 110mW @ 3.3V for 0.1s
	SensorHeader_20mWFor1s	 	= 0x1E,		// 20mW  @ 3.3V for 1s
	SensorHeader_20mWFor0p1s	= 0x15,		// 20mW  @ 3.3V for 0.1s
} Header;


/**
  @brief Measurement precision settings for the SHT40-AD1B-R2 sensor
*/
typedef enum Precision {
	SensorPrecision_High   = 0xFD,		// measure T & RH with High precision (High repeatability)
	SensorPrecision_Medium = 0xF6,		// measure T & RH with medium precision (medium repeatability)
	SensorPrecision_Low    = 0xE0,		// measure T & RH with lowest precision (low repeatability)
} Precision;


/**
  @brief Initialization of the SHT40-AD1B-R2 relative humidity and temperature sensor
  @note The sensor does not need a special initialization, this function just does a soft reset
  @return Success
*/
void init(void);


/**
  @brief Get the temperature without using the header
  @note The measurement takes about 10ms
  @param precision Precision for the conversion
  @return Temperature sensor temperature
*/
float getTemperature(Precision precision);


/**
  @brief Get the temperature without using the header
  @note The measurement takes about 10ms
  @param precision Precision for the conversion
  @return Temperature sensor temperature
*/
float getHumidity(Precision precision);


/**
  @brief Enable the header module on the sensor
  @warning The heater is designed for a maximal duty cycle of less than 5% when it is periodically heated
  @param level Power and duration command
 */
void enableHeater(Header level);

#endif /* SHT40I_DRIVER_H */
