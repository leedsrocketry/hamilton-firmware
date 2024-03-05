/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "mcu.h"
#include "STM32_init.h"
#include "stm32l4r5xx.h"
#include "NAND_flash_driver.h"
#include "drivers/MS5611_driver.h"
#include "drivers/ADXL375_driver.h"
#include "data_buffer.h"
//#include "drivers/LSM6DS3_driver.h"
//#include "drivers/SI446_driver.h"

// Flags
FlightStages flightStage = LAUNCHPAD;
volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

#pragma region Updates
/**
  @brief Send data to the ground station
  @param frameAddressPointer - Address of the frame in the NAND flash
  @param _M5611_data - MS5611 barometer data
  @param _ADXL375_data - ADXL375 accelerometer data
*/
void log_data(uint32_t frameAddressPointer,
              M5611_data* _M5611_data, 
              ADXL375_data* _ADXL375_data) {
  // Convert data to frame
  Vector3 _acc_high_g = { _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z };

  // Add data to the frame
  FrameArray _frameArray;
  _frameArray.barometer = _M5611_data->pressure;
  _frameArray.temp = _M5611_data->temp;
  _frameArray.accelHighG = _acc_high_g; 

  // Add data to NAND flash
  log_frame(_frameArray);
}

/**
  @brief Retrieve data from sensors
  @param _M5611_data - MS5611 barometer data
  @param _ADXL375_data - ADXL375 accelerometer data
  @param _LSM6DS3_data - LSM6DS3 IMU data

*/
void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data) {
  
  MS5611_get_data(&_M5611_data);
  //ADXL375_get_data(&_ADXL375_data);
}
#pragma endregion Updates

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void)
{
  // STM32 setup
  STM32_init_clock(RCC_CFGR_SW_HSI); // Set clock to 16MHz internal HSI
  STM32_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);

  printf("================ PROGRAM START ================\r\n");
  STM32_indicate_on();

  printf("============ INITIALISE NAND FLASH ============\r\n");
  // init_flash();

  printf("============== INITIALISE DRIVERS =============\r\n");
  //MS5611_init(SPI1);      // Barometer
  //ADXL375_init(SPI1);     // Accelerometer

  M5611_data _M5611_data;
  dataBuffer _M5611_buffer;
  ADXL375_data _ADXL375_data;
  dataBuffer _ADXL375_buffer;

  //printf("============== ADD TESTS HERE ==============\r\n");

  /*
  printf("============= ENTER MAIN PROCEDURE ============\r\n");
  for (;;) {
    #pragma region Flight Stages
    switch (flightStage) {
      case LAUNCHPAD:
        update_sensors(&_M5611_data, &_ADXL375_data);
        //update_buffer(&_M5611_data, &_M5611_buffer);
        break;

      case ASCEND:
        update_sensors(&_M5611_data, &_ADXL375_data);
        update_buffer(&_M5611_data, &_M5611_buffer);
        break;

      case APOGEE:
        update_sensors(&_M5611_data, &_ADXL375_data);
        update_buffer(&_M5611_data, &_M5611_buffer);
        break;

      case DESCENT:
        update_sensors(&_M5611_data, &_ADXL375_data);
        update_buffer(&_M5611_data, &_M5611_buffer);
        break;

      case LANDING:
        //indicate_on_buzzer();
        break;
    }

    //send_data();
  }*/

  printf("===================== PROGRAM END ====================");
}
