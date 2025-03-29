/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 22 March 2025
  Description: Overall driver manager
*/

#include "HAL/mcu.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"

M5611_data _M5611_data;
ADXL375_data _ADXL375_data;
LSM6DS3_data _LSM6DS3_data;
BME280_data _BME280_data;
GNSS_Data _GNSS_data;

void initalise_drivers() {
  _BME280_data.temperature = 0;
  _BME280_data.pressure = 0;
  _BME280_data.humidity = 0;
  _GNSS_data.latitude = 0;
  _GNSS_data.longitude = 0;
  _GNSS_data.altitude = 0;
  _GNSS_data.velocity = 0;

  // Sensor initialisation

  if (MS5611_init(SPI1)) {
    LOG("ERROR INITIALISING MS5611 BAROMETER\n");
  }
  if (ADXL375_init(SPI1)) {
    LOG("ERROR INITIALISING ADXL375 ACCEL\n");
  }
  if (LSM6DS3_init(SPI1, &_LSM6DS3_data)) {
    LOG("ERROR INITIALISING LSM6DS3 IMU\n");
  }
}
