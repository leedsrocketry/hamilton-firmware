/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
  Description: header file for the Pad Radio module SI446
*/
#ifndef SI446_DRIVER_H
#define SI446_DRIVER_H

typedef struct SI446_data
{

} SI446_data;

#pragma region Public

/**
  @brief Initialization of the SI4463 module
  @note
  @return Success/Failure
*/
int8_t init_SI446();

/**
  @brief Get data from the SI446 Pad Radio module
  @note
  @param data ptr to SI446_data struct for returning data
  @return Success/Failure
*/
int8_t get_data_SI446(SI446_data *data);

#pragma endregion Public

#pragma region Private
#pragma endregion Private

#endif /* SI446_DRIVER_H */
