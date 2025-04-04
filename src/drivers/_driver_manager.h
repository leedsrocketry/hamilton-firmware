/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 29 Mar 2025
  Description: Overall driver manager for the HFC sensors.
*/

#ifndef DRIVER_MANAGER_H
#define DRIVER_MANAGER_H

#include "HAL/mcu.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"

void initalise_drivers();

#endif /* DRIVER_MANAGER_H */