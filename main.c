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
void get_frame_array(FrameArray* _frameArray, 
                    M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data) {
  // Convert data to frame TODO
  Vector3 _acc_high_g = { _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z };

  // Add data to the frame
  _frameArray->barometer = _M5611_data->pressure;
  _frameArray->temp = _M5611_data->temp;
  _frameArray->accelHighG = _acc_high_g;  
}

/**
  @brief Retrieve data from sensors
  @param _M5611_data - MS5611 barometer data
  @param _ADXL375_data - ADXL375 accelerometer data
  @param _LSM6DS3_data - LSM6DS3 IMU data

*/
void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data) {
  
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  printf("Accel: %d, %d, %d\r\n", _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z);
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
  // Sensor initialisation
  MS5611_init(SPI1);          // Barometer
  ADXL375_init(SPI1);         // Accelerometer

  // Buffer
  FrameArray frame;                         // initialise the frameArray that keeps updating
  uint8_t dataArray[128];                   // dummy array to store the frame data
  _memset(dataArray, 0, sizeof(dataArray)); // set the necessary memory and set values to 0
  zip(frame, dataArray);                    // convert from normal array into FrameArray
  dataBuffer frame_buffer;                  // contains FrameArrays
  init_buffer(&frame_buffer);               // initialise the buffer

  // Buffer data
  M5611_data _M5611_data;
  ADXL375_data _ADXL375_data;

  // Additional variables
  int _data[WINDOW_SIZE];
  int previous_value = 999999999;
  int current_value = 999999999;
  int apogee_incr = 3;

  //printf("============== ADD TESTS HERE ==============\r\n");

  printf("============= ENTER MAIN PROCEDURE ============\r\n");
  for (;;) {
    #pragma region Flight Stages
    switch (flightStage) {
      case LAUNCHPAD:
        // Get the sensor readings
        update_sensors(&_M5611_data, &_ADXL375_data);
        get_frame_array(&frame, &_M5611_data, &_ADXL375_data);  
        
        // Update buffer and window  
        update_buffer(&frame, &frame_buffer);
        if (frame_buffer.count > WINDOW_SIZE*2) {
          // Get the window barometer median
          int _data[WINDOW_SIZE];
          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data[i] = frame_buffer.window[i].barometer;
          }
          current_value = get_median(_data, WINDOW_SIZE); // get pressure median

          // Check for launch given pressure decrease
          if ((frame_buffer.ground_ref - current_value) > LAUNCH_THRESHOLD) {
            printf("Ground: %d\r\n, Now: %d\r\n", frame_buffer.ground_ref, current_value);
            printf("Difference: %d\r\n", (frame_buffer.ground_ref - current_value));
            flightStage = ASCEND;

            // Log all data from the buffer
            //for (int i = 0; i < WINDOW_SIZE; i++) {
            // log_frame(frame_buffer.frames[1]);
          }
        }
      break;

      case ASCEND:
        // TODO: set a sampling rate

        printf("ASCEND\r\n");
        // Get the sensor readings
        update_sensors(&_M5611_data, &_ADXL375_data);
        get_frame_array(&frame, &_M5611_data, &_ADXL375_data); 

        // Log data
        log_frame(frame);

        // Update buffer and window  
        update_buffer(&frame, &frame_buffer);

        // Get window median readings
        for (int i = 0; i < WINDOW_SIZE; i++) {
          _data[i] = frame_buffer.window[i].barometer;
        }
        current_value = get_median(_data, WINDOW_SIZE); // get pressure median

        // Check for apogee given pressure increase
        if (current_value - previous_value > 0)
          flightStage = APOGEE;
        else
          previous_value = current_value;
        break;

      case APOGEE:
        // Get the sensor readings
        update_sensors(&_M5611_data, &_ADXL375_data);
        get_frame_array(&frame, &_M5611_data, &_ADXL375_data); 

        // Log data
        log_frame(frame);

        // Update buffer and window  
        update_buffer(&frame, &frame_buffer);

        // Run for a few cycles to record apogee when switch to descent
        if (apogee_incr == 0)
          flightStage = DESCENT;
        else
          apogee_incr--;
        break;

      case DESCENT:
        // TODO: reduce the sampling rate

        // Get the sensor readings
        update_sensors(&_M5611_data, &_ADXL375_data);
        get_frame_array(&frame, &_M5611_data, &_ADXL375_data); 

        // Log data
        log_frame(frame);

        // Update buffer and window  
        update_buffer(&frame, &frame_buffer);
        // Get window median readings
        int _data[WINDOW_SIZE];
        for (int i = 0; i < WINDOW_SIZE; i++) {
          _data[i] = frame_buffer.window[i].barometer;
        }

        // Check for landing
        if (is_stationary(_data)) {
          flightStage = LANDING;
        }
        break;

      case LANDING:
        STM32_indicate_on();
        break;
    }
  }
}
