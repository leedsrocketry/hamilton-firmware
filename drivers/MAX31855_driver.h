/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
  Description: header file for the Temperature sensor
*/
#ifndef MAX31855_DRIVER_H
#define MAX31855_DRIVER_H

#include "mcu.h"

SPI_TypeDef MAX31855_SPI;


typedef struct MAX31855_data
{
  int16_t temp;

} MAX31855_data;



#pragma region Public
/**
  @brief Initialization of the MAX31855 Temperature sensor
  @note
  @return Success
*/
int8_t MAX31855_init(SPI spi);

/**
  @brief Get data from the MAX31855 Accelerometer module
  @note
  * @param data ptr to MAX31855_data struct for returning data
  @return Success/Failure
*/
int8_t MAX31855_get_data();

/**
  @brief Get data from the MAX31855 Accelerometer module
  @note
  * @param data ptr to MAX31855_data struct for returning data
  @return Success/Failure
*/
int8_t MAX31855_get_full_data();


#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* MAX31855_DRIVER_H */
