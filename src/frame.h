/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 04 April 2024
  Description: Frame struture used for storing and reading off the NAND Flash
*/

#ifndef FRAME_ARRAY_H
#define FRAME_ARRAY_H

#include "HAL/mcu.h"
#include "debug.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"

// Max 128 bytes

typedef enum SensorReading {
  MS5611_PRESSURE,
  MS5611_TEMP,
  ALTITUDE
} SensorReading;

typedef struct Frame {
  DateTime date;         // 56 bits
  uint8_t changeFlag;    // 8 bits
  ADXL375_data accel;    // 48 bits
  LSM6DS3_data imu;      // 240 bits
  M5611_data barometer;  // 64 bits
  GNSS_Data GNSS;        // 64 bits
  BME280_data bme;       // 80 bits
  uint8_t hammingCode[8];
  uint16_t CRC_Check;

  uint32_t time;
  double altitude;  // calculated value

  int successFlag;  // Not used in zip
} Frame;

#endif /* FRAME_ARRAY_H */