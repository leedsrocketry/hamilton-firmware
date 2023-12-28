/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
  Description: header file for the Gyroscope module LSM6DS3
*/
#ifndef LSM6DS3_DRIVER_H
#define LSM6DS3_DRIVER_H

typedef struct LSM6DS3_data
{

} LSM6DS3_data;

#pragma region Public

/**
  @brief Initialization of the LSM6DS3 Gyroscope module
  @note
  @return Success/Failure
*/
int8_t init_LSM6DS3();

/**
 * @brief Get data from the LSM6DS3 Gyroscope module
 * @note
 * @param data ptr to LSM6DS3_data struct for returning data
 * @return Success/Failure
 */
int8_t get_data_LSM6DS3(LSM6DS3_data *data);

#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* LSM6DS3_DRIVER_H */
