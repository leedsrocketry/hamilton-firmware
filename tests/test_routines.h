/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 03 March 2024
  Description: Header file for the drivers test routines.
*/
#ifndef TESTING_ROUTINE_H

#include "HAL/mcu.h"
#include "HAL/STM32_init.h"
#include "drivers/MS5611_driver.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/SI446_driver.h"
#include "drivers/BME280_driver.h"

/**
  @brief Initial Routine to run on hardware. Should trigger RGB blink sequence.
*/
void run_test_routine();

/**
  @brief Routine to test the MS5611 barometer.
*/
void run_MS5611_routine();

/**
  @brief Routine to test the ADXL375 accelerometer.
*/
void run_ADXL375_routine();

/**
  @brief Routine to test the SI446 radio module.
*/
void SI446_Test_routine();

/**
  @brief Routine to test the LSM6DS3 IMU.
*/
void LSM6DS3_test_routine();

/**
  @brief Routine to test the BME280 sensor.
*/
void BME280_test_routine();

/**
  @brief Routine to test the SPI communication.
*/
void spi_test_routine();

/**
  @brief Routine to test NAND Flash reading and writing.
*/
//void NAND_flash_test_routine();

#endif /*TESTING_ROUTINE_H*/
