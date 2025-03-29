/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 15 December 2023
  Description: header file for the driver manager
*/


#ifndef DRIVER_MANAGER_H
#define DRIVER_MANAGER_H

#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "HAL/mcu.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"

void initalise_drivers();

#endif /* DRIVER_MANAGER_H */