
#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>

#include "HAL/STM32_init.h"
#include "HAL/mcu.h"
#include "frame.h"
#include "stm32l4r5xx.h"
// #include "HAL/NAND_flash_driver.h"
#include "frame_buffer.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"
#include "flight_manager.h"
#include "debug.h"

static double Lb = -0.0065; // temperature lapse rate (K/m)
static double hb = 0; // reference height (sea level)
static double R = 8.3144598; // universal gas constant
static double g = 9.80665; // gravitational acceleration
static double M = 0.0289644; // molar mass of Earth's air
static double Pb = 101325; // reference pressure at sea level (in Pa)


void read_sensors(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data);

void build_frame(Frame* _frameArray, M5611_data _M5611_data,
                     ADXL375_data _ADXL375_data, LSM6DS3_data _LSM6DS3_data,
                     BME280_data _BME280_data, GNSS_Data _GNSS_data);

void format_sensor_data(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                        LSM6DS3_data* _LSM6DS3_data, char* buffer, size_t buffer_size);

double barometric_equation(double pressure, double temp);


#endif /* SENSORS_H */