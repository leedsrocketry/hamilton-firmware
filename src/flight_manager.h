/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 26 July 2024
  Description: System for managing flight
*/

#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include <stdio.h>

#include "HAL/NAND_flash_driver.h"
#include "HAL/STM32_init.h"
#include "HAL/mcu.h"
#include "debug.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/HC12_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"
#include "frame.h"
#include "sensors.h"
#include "stm32l4r5xx.h"

#define ACCEL_LAUNCH_THRESHOLD -3000.0  // TBD
#define BARO_LAUNCH_THRESHOLD 400.0     // TBD

#define BARO_APOGEE_THRESHOLD 0.0
#define ALTITUDE_APOGEE_THRESHOLD 7.5  // 7.5m

#define GROUND_THRESHOLD 30

// Unsure which of these is correct as of 15/09/2024
#define EVENT_TRIG_1 PIN('C', 2)
// #define EVENT_TRIG_1 PIN('C', 1)

// Not used as of 04/08/2024, should be moved later
#define BATTERY_ADC PIN('C', 3)

typedef enum FlightStage { LAUNCHPAD, ASCENT, APOGEE, DESCENT, LANDING } FlightStage;

extern FlightStage flightStage;

FlightStage get_flight_stage();
void set_flight_stage(FlightStage fs);

void initalise_drivers();

void run_flight();

#endif /* FLIGHT_MANAGER_H */
