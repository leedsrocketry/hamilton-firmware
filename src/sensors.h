
#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>

#include "HAL/STM32_init.h"
#include "HAL/mcu.h"
#include "frame_array.h"
#include "stm32l4r5xx.h"
// #include "HAL/NAND_flash_driver.h"
#include "data_buffer.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/BME280_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/MS5611_driver.h"
#include "flight_manager.h"
#include "debug.h"

void read_sensors(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data);

void build_frame(Frame* _frameArray, M5611_data _M5611_data,
                     ADXL375_data _ADXL375_data, LSM6DS3_data _LSM6DS3_data,
                     BME280_data _BME280_data, GNSS_Data _GNSS_data);

void format_sensor_data(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                        LSM6DS3_data* _LSM6DS3_data, char* buffer, size_t buffer_size);

#endif /* SENSORS_H */