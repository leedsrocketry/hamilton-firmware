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
#include "drivers/LSM6DS3_driver.h"
#include "data_buffer.h"

#define PADREADFREQ 100 //frequency to read data during ascent
#define ASCENTREADFREQ 1000 //frequency to read data during ascent
#define APOGEEREADFREQ 1000 //frequency to read data during ascent
#define DESCENTREADFREQ 100 //frequency to read data during descent

// Flags
FlightStages flightStage = LAUNCHPAD;
volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

#pragma region Updates
void get_frame_array(FrameArray* _frameArray, 
                    M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data, 
                    LSM6DS3_data* _LSM6DS3_data) {
  // Convert data to frame TODO
  Vector3 _acc_high_g = { _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z };

  // Add time stamp
  _frameArray->date.minute = (get_time_us()/(1000000*60))%60; // Minuts
  _frameArray->date.second = (get_time_us()/1000000)%60;      // Seconds
  _frameArray->date.millisecond = (get_time_us()/1000)%1000;  // Milli seconds
  _frameArray->date.microsecond = get_time_us()%1000;         // Mirco seconds
  
  // Add data to the frame
  _frameArray->changeFlag = flightStage;
  _frameArray->barometer = _M5611_data->pressure;
  _frameArray->temp = _M5611_data->temp;
  _frameArray->accelHighG = _acc_high_g;
  _frameArray->gyroscope.x = (uint16_t) (_LSM6DS3_data->x/10+18000);
  _frameArray->gyroscope.y = (uint16_t) (_LSM6DS3_data->y/10+18000);
  _frameArray->gyroscope.z = (uint16_t) (_LSM6DS3_data->z/10+18000);
}

void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data) {
  
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  lsm6ds6GyroReadAngle(SPI1, _LSM6DS3_data);

  printf("Barometer: %d, Temp: %d, Accel: %d, %d, %d, Gyro: %d, %d, %d\r\n", 
          _M5611_data->pressure, _M5611_data->temp, 
          _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z, 
          _LSM6DS3_data->x, _LSM6DS3_data->y, _LSM6DS3_data->z);
}
#pragma endregion Updates

#pragma region NAND
void run_nand_flash_erase(){
  watchdog_pat();
  erase_all();
  while(1);
}

/**
  @brief Routine to test NAND Flash reading and writing.
*/
void NAND_flash_read()
{
  printf("==================== Reading NAND FLASH ====================\r\n");
  read_all_csv();
  print_capacity_info();
}
#pragma endregion NAND

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void)
{
  // STM32 setup
  STM32_init();

  printf("================ PROGRAM START ================\r\n");
  STM32_indicate_on();  

  printf("============ INITIALISE NAND FLASH ============\r\n");
  //init_flash();

  printf("============== INITIALISE DRIVERS =============\r\n");
  // Buffer data
  M5611_data _M5611_data;
  ADXL375_data _ADXL375_data;
  LSM6DS3_data _LSM6DS3_data;

  // Sensor initialisation
  MS5611_init(SPI1);                    // Barometer
  ADXL375_init(SPI1);                   // Accelerometer
  lsm6ds6_init(SPI1, &_LSM6DS3_data);   // IMU

  // Buffer
  FrameArray frame;                         // initialise the frameArray that keeps updating
  uint8_t dataArray[128];                   // dummy array to store the frame data
  _memset(dataArray, 0, sizeof(dataArray)); // set the necessary memory and set values to 0
  frame = unzip(&dataArray);                // convert from normal array into FrameArray
  dataBuffer frame_buffer;                  // contains FrameArrays
  init_buffer(&frame_buffer);               // initialise the buffer

  // Additional variables
  int _data[WINDOW_SIZE];
  int previous_value = 999999999;
  int current_value = 999999999;
  int apogee_incr = 3;

  printf("============== ADD TESTS HERE ==============\r\n");

  printf("============= ENTER MAIN PROCEDURE ============\r\n");
  uint32_t newTime = get_time_us();
  uint32_t oldTime = get_time_us();

  for (;;) {
    switch (flightStage) {
      case LAUNCHPAD:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / PADREADFREQ)) {
          oldTime = newTime;  // old time = new time

          // Get the sensor readings
          update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          get_frame_array(&frame, &_M5611_data, &_ADXL375_data, &_LSM6DS3_data);  
          
          // Update buffer and window  
          update_buffer(&frame, &frame_buffer);
          if (frame_buffer.count > WINDOW_SIZE*2) {
            // Get the window barometer median
            for (int i = 0; i < WINDOW_SIZE; i++) {
              _data[i] = frame_buffer.window[i].barometer;
            }
            current_value = get_median(_data, WINDOW_SIZE); // get pressure median

            // Check for launch given pressure decrease
            if ((frame_buffer.ground_ref - current_value) > LAUNCH_THRESHOLD) {
              flightStage = ASCENT;
              // Log all data from the buffer
              for (int i = 0; i < BUFFER_SIZE; i++) {
                log_frame(frame_buffer.frames[i]);
              }
            }
          }
        }
        break;

      case ASCENT:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / ASCENTREADFREQ)) {
          oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          get_frame_array(&frame, &_M5611_data, &_ADXL375_data, &_LSM6DS3_data); 

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
          if (current_value - previous_value > APOGEE_THRESHOLD){
            flightStage = APOGEE;
          } else if (previous_value > current_value){  // Storing the minimum, (median), pressure value during ascent
            previous_value = current_value;
          }
        }
        break;

      case APOGEE:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / APOGEEREADFREQ)) {
          oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          get_frame_array(&frame, &_M5611_data, &_ADXL375_data, &_LSM6DS3_data); 

          // Log data
          log_frame(frame);

          // Update buffer and window  
          update_buffer(&frame, &frame_buffer);

          // Run for a few cycles to record apogee when switch to descent
          if (apogee_incr == 0)
            flightStage = DESCENT;
          else
            apogee_incr--;
        }
        break;

      case DESCENT:
        newTime = get_time_us();  // Get current time
          if ((newTime - oldTime) > (1000000 / DESCENTREADFREQ)) {
            oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          get_frame_array(&frame, &_M5611_data, &_ADXL375_data, &_LSM6DS3_data); 

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
        }
        break;

      case LANDING:
        STM32_beep_buzzer(200, 200, 1);
        break;
    }
  }
}
