
#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include <stdio.h>

#include "HAL/NAND_flash_driver.h"
#include "HAL/STM32_init.h"
#include "HAL/mcu.h"
#include "data_buffer.h"
#include "debug.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"
#include "drivers/MAXM10S_driver.h"
#include "frame_array.h"
#include "stm32l4r5xx.h"

#define PADREADFREQ 100      // frequency to read data during ascent
#define ASCENTREADFREQ 1000  // frequency to read data during ascent
#define APOGEEREADFREQ 1000  // frequency to read data during ascent
#define DESCENTREADFREQ 100  // frequency to read data during descent

typedef enum FlightStage {
  LAUNCHPAD,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDING
} FlightStage;

extern FlightStage flightStage;

FlightStage get_flight_stage();

void set_flight_stage(FlightStage fs);

void run_flight();

#endif /* FLIGHT_MANAGER_H */