/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Oliver Martin
  Created on: 10 June 2023
  Description: header file for the Thermocoupler sensor MAX31855KASA+T (http://datasheets.maximintegrated.com/en/ds/MAX31855.pdf)
*/
#ifndef MAX31855_DRIVER_H
#define MAX31855_DRIVER_H

#include "mcu.h"

SPI_TypeDef MAX31855_SPI;


typedef struct MAX31855_data
{
  float temp;
  float internalTemp;
  bool fault; //0 is no fault, 1 is fault
  uint8_t faultType; //0 is no fault, 1 is short to VCC, 2 is short to GND, 3 is open circuit, 4 is unknown fault reason.
} MAX31855_data;



#pragma region Public
/**
  @brief Initialization of the MAX31855 Temperature sensor
  @note
  @return Success
*/
int8_t MAX31855_init(SPI_TypeDef spi);

/**
  @brief Get float of temperature from the MAX31855 Accelerometer module
  @note
  * @param 0.25 resolution
  @return Success/Failure
*/
MAX31855_data MAX31855_get_data();

/**
  @brief Get data from the MAX31855 Accelerometer module
  @note
  * @param data ptr to MAX31855_data struct for returning data
  @return Success/Failure
*/
MAX31855_data MAX31855_get_full_data();


#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* MAX31855_DRIVER_H */
