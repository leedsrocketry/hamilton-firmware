
#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include <stdio.h>

#include "HAL/NAND_flash_driver.h"
#include "HAL/STM32_init.h"
#include "HAL/mcu.h"
#include "frame_buffer.h"
#include "debug.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"
#include "frame.h"
#include "stm32l4r5xx.h"
#include "sensors.h"

#define PADREADFREQ 100      // frequency to read data during ascent
#define ASCENTREADFREQ 1000  // frequency to read data during ascent
#define APOGEEREADFREQ 1000  // frequency to read data during ascent
#define DESCENTREADFREQ 100  // frequency to read data during descent

#define ACCEL_LAUNCH_THRESHOLD -100.0 // TBD
#define BARO_LAUNCH_THRESHOLD 400.0 // TBD

#define BARO_APOGEE_THRESHOLD 0.0
#define ALTITUDE_APOGEE_THRESHOLD 10.0 // meters

#define EVENT_TRIG_1 PIN('C', 2)
#define EVENT_TRIG_1 PIN('C', 1)

// Not used as of 04/08/2024, should be moved later
#define BATTERY_ADC PIN('C', 3)

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