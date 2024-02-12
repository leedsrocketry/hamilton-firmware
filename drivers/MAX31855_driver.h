/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Oliver Martin
  Created on: 10 June 2023
  Description: header file for the Thermocoupler sensor MAX31855KASA+T (http://datasheets.maximintegrated.com/en/ds/MAX31855.pdf)
*/
#ifndef MAX31855_DRIVER_H
#define MAX31855_DRIVER_H

#include "mcu.h"

#define MAX31855_CS       0; //update this
SPI_TypeDef MAX31855_SPI;

/**
 * @brief MAX31855 device structure
 */
typedef struct MAX31855_data
{
  //Float storing temperature result
  float temp;
  //Float storing internal temperature result
  float internalTemp;
  //Bool storing fault state
  bool fault; //0 is no fault, 1 is fault
  //uint8_t storing fault type
  uint8_t faultType; //0 is no fault, 1 is short to VCC, 2 is short to GND, 3 is open circuit, 4 is unknown fault reason.
} MAX31855_data;



#pragma region Public
/**
  @brief Initialization of the MAX31855 Temperature sensor
  @note Saves the SPI objects
  @return Success
*/
int8_t MAX31855_init(SPI_TypeDef spi);

/**
  @brief Get float of temperature from the MAX31855 Accelerometer module
  @note Gets the first part of the data which includes temperature with 0.25 resolution
  @return MAX31855_data structure with temperature and fault
*/
MAX31855_data MAX31855_get_data();

/**
  @brief Get data full from the MAX31855 Accelerometer module
  @note Gets full 32 bit data from sensor with Temperature, internal temperature, and fault information
  @return MAX31855_data structure
*/
MAX31855_data MAX31855_get_full_data();

#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* MAX31855_DRIVER_H */
