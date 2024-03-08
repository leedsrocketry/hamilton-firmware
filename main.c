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
*/
void log_data(FrameArray _frameArray) {
  // Add data to NAND flash
  log_frame(_frameArray);
}

void  get_frame_array(FrameArray* _frameArray, 
                    M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data) {
  // Convert data to frame
  Vector3 _acc_high_g = { _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z };

  //add time stamp
  _frameArray->date.minute = (get_time_us()/(1000000*60))%60; //minuts
  _frameArray->date.second = (get_time_us()/1000000)%60; //seconds
  _frameArray->date.millisecond = (get_time_us()/1000)%1000; //milli seconds
  _frameArray->date.microsecond = get_time_us()%1000; //Mirco seconds
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
  //printf("Accel: %d, %d, %d\r\n", _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z);
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
  zip(frame, dataArray);
  dataBuffer frame_buffer;                  // contains FrameArrays
  FrameArray window[WINDOW_SIZE];           // contains last WINDOW_SIZE readings
  init_buffer(&frame_buffer);               // initialise the buffer

  // Buffer data
  M5611_data _M5611_data;
  ADXL375_data _ADXL375_data;

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
          for (int i = 0; i < WINDOW_SIZE; i++) {
            int frame_number = (frame_buffer.index - WINDOW_SIZE + i);
            if (frame_number < 0) {
              frame_number = BUFFER_SIZE + frame_number;
            }
            window[i] = frame_buffer.frames[i];
          }

          // get the window barometer median
          int _data[WINDOW_SIZE];
          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data[i] = window[i].barometer;
          }
          int current_val = get_median(_data, WINDOW_SIZE);

          // Check for launch given pressure decrease
          if ((frame_buffer.ground_ref - current_val) > LAUNCH_THRESHOLD) {
            printf("Ground: %d\r\n, Now: %d\r\n", frame_buffer.ground_ref, current_val);
            printf("Difference: %d\r\n", (frame_buffer.ground_ref - current_val));
            flightStage = ASCEND;
          }
        }
      break;

      case ASCEND:
        printf("ASCEND\r\n");
        update_sensors(&_M5611_data, &_ADXL375_data);
        break;

      case APOGEE:
        update_sensors(&_M5611_data, &_ADXL375_data);
        break;

      case DESCENT:
        update_sensors(&_M5611_data, &_ADXL375_data);
        break;

      case LANDING:
        STM32_indicate_on();
        break;
    }

    //send_data();
  }

  printf("===================== PROGRAM END ====================");
}
