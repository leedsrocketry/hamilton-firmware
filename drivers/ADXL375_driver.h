/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
  Description: header file for the Accelerometer module ADXL375
*/
#ifndef ADXL375_DRIVER_H
#define ADXL375_DRIVER_H

typedef struct ADXL375_data
{

} ADXL375_data;

#pragma region Public

/**
  @brief Initialization of the ADXL375 Accelerometer module
  @note This function should be called before attempting to read data from the accelerometer.
  @return Success/Failure
*/
int8_t init_ADXL375();

/**
  @brief Get data from the ADXL375 Accelerometer module
  @note
  @param data ptr to ADXL375_data struct for returning data
  @return Success/Failure
*/
int8_t get_data_ADXL375(ADXL375_data *data);

#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* ADXL375_DRIVER_H */