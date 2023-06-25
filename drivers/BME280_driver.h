/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: header file for the Pressure/Temp/humidity module BME280 (https://www.mouser.co.uk/datasheet/2/783/bst_bme280_ds002-2238172.pdf)
*/
#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H
#include "stdint.h"


#define BME280_STATUS_OK 0


/**
  @brief Commands for the SHT40-AD1B-R2 sensor
*/
typedef enum Command {
  readSerial = 0x89,		// Every single sensor has a unique serial number
  softReset  = 0x94,		// Soft reset
} Command;


/**
  @brief Heater commands for the SHT40-AD1B-R2 sensor
  @warning Do not use heater for extended periods of time. The heater is designed 
  for a maximal duty cycle of less than 10% when it is periodically heated
*/
typedef enum Heater {
  sensorHeater_200mWFor1s	 	= 0x39,		// 200mW @ 3.3V for 1s
  sensorHeater_200mWFor0p1s = 0x32,		// 200mW @ 3.3V for 0.1s
  sensorHeater_110mWFor1s	 	= 0x2F,		// 110mW @ 3.3V for 1s
  sensorHeater_110mWFor0p1s	= 0x24,		// 110mW @ 3.3V for 0.1s
  sensorHeater_20mWFor1s	 	= 0x1E,		// 20mW  @ 3.3V for 1s
  sensorHeater_20mWFor0p1s	= 0x15,		// 20mW  @ 3.3V for 0.1s
} Heater;


/**
  @brief Measurement precision settings for the SHT40-AD1B-R2 sensor
*/
typedef enum Precision {
	sensorPrecision_High   = 0xFD,		// measure T & RH with High precision (High repeatability)
	sensorPrecision_Medium = 0xF6,		// measure T & RH with medium precision (medium repeatability)
	sensorPrecision_Low    = 0xE0,		// measure T & RH with lowest precision (low repeatability)
} Precision;


/**
 @brief Sensor data
*/
typedef struct SensorData {
    float humidity;				// Humidity sensor value
    float temperature;	        // Temperature sensor value
} SensorData;


/**
  @brief Raw sensor data
*/
typedef struct SensorDataRaw {
    uint8_t th;			// Temperature high byte
    uint8_t tl;			// Temperature low byte
    uint8_t tcrc8;		// Temperature crc8 checksum
    uint8_t rhh;		// Relative humidity high byte
    uint8_t rhl;		// Relative humidity low byte
    uint8_t rhcrc8;		// Relative humidity crc8 checksum
} SensorDataRaw;


/**
  @brief Initialization of the SHT40-AD1B-R2 relative humidity and temperature sensor
  @note The sensor does not need a special initialization, this function just does a soft reset
  and a serial check
  @note set the SHT40I_STATUS_OK to 1 if the initialization is successful
*/
void init_SHT40I(void);


/**
  @brief Get the temperature without using the header
  @note The measurement takes about 10ms
  @param precision Precision for the conversion
  @return Temperature sensor temperature
*/
float get_temperature(Precision precision);


/**
  @brief Get the temperature without using the header
  @note The measurement takes about 10ms
  @param precision Precision for the conversion
  @return Temperature sensor temperature
*/
float get_humidity(Precision precision);


/**
  @brief Enable the header module on the sensor
  @warning The heater is designed for a maximal duty cycle of less than 10% when it is 
  periodically heated
  @param level Power and duration command
 */
void enable_heater(Heater level);

#endif /* SHT40I_DRIVER_H */
