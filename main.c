/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "mcu.h"
#include "frame_array.h"
#include "STM32_init.h"
#include "stm32l4r5xx.h"
#include "NAND_flash_driver.h"
#include "drivers/MS5611_driver.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/BME280_driver.h"
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
                    M5611_data _M5611_data, 
                    ADXL375_data _ADXL375_data, 
                    LSM6DS3_data _LSM6DS3_data,
                    BME280_data _BME280_data,
                    GNSS_Data _GNSS_data) {
  // Add time stamp
  uint32_t time = get_time_us();
  _frameArray->date.minute = (time/(1000000*60))%60; //minuts
  _frameArray->date.second = (time/1000000)%60; //seconds
  _frameArray->date.millisecond = (time/1000)%1000; //milli seconds
  _frameArray->date.microsecond = time%1000; //Mirco seconds
  
  // Add data to the frame
  _frameArray->changeFlag = flightStage;
  _frameArray->accel = _ADXL375_data;
  _frameArray->imu =_LSM6DS3_data;
  _frameArray->barometer = _M5611_data;
  _frameArray->GNSS = _GNSS_data;
  _frameArray->bme = _BME280_data;
}

void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data) {
  
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  Lsm6ds3GyroRead(SPI1, _LSM6DS3_data);
  Lsm6ds3AccRead(SPI1, _LSM6DS3_data);

  printf("Barometer: %d, Temp: %d, Accel: %d, %d, %d, Gyro: %d, %d, %d\r\n", 
          _M5611_data->pressure, _M5611_data->temp, 
          _LSM6DS3_data->x_accel, _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel, 
          _LSM6DS3_data->x_rate, _LSM6DS3_data->y_rate, _LSM6DS3_data->z_rate);
}
#pragma endregion Updates

#pragma region NAND
void NAND_flash_erase(){
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
  init_flash();

  printf("============== INITIALISE DRIVERS =============\r\n");
  // Buffer data
  M5611_data _M5611_data;
  ADXL375_data _ADXL375_data;
  LSM6DS3_data _LSM6DS3_data;
  BME280_data _BME280_data;
  _BME280_data.temperature = 0;
  _BME280_data.pressure = 0;
  _BME280_data.humidity = 0;
  GNSS_Data _GNSS_data;
  _GNSS_data.latitude = 0;
  _GNSS_data.longitude = 0;
  _GNSS_data.altitude = 0;
  _GNSS_data.velocity = 0;

  // Sensor initialisation
  MS5611_init(SPI1);                    // Barometer
  ADXL375_init(SPI1);                   // Accelerometer
  Lsm6ds3Init(SPI1, &_LSM6DS3_data);    // IMU

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
  bool toggle_LED = true;

  printf("============== ADD TESTS HERE ==============\r\n");
  //NAND_flash_read();
  //NAND_flash_erase();

  printf("============= ENTER MAIN PROCEDURE ============\r\n");
  uint32_t newTime = get_time_us();
  uint32_t oldTime = get_time_us();
  uint32_t dt = 0;
  delay_microseconds(1000*1000);  // One second delay before launch for sensors to stabilise 

  for (;;) {
    switch (flightStage) {
      case LAUNCHPAD:
        newTime = get_time_us();              // Get current time
        if ((newTime - oldTime) > (1000000 / PADREADFREQ)) {
          oldTime = newTime;                  // old time = new time
          gpio_write(GREEN_LED, toggle_LED);  // Flick LED
          toggle_LED = !toggle_LED;

          // Get the sensor readings
          update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, \
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);
          
          // Update buffer and window  
          update_buffer(&frame, &frame_buffer);
          if (frame_buffer.count > WINDOW_SIZE*2) {
            // Get the window barometer median
            for (int i = 0; i < WINDOW_SIZE; i++) {
              _data[i] = frame_buffer.window[i].barometer.pressure;
            }
            current_value = get_median(_data, WINDOW_SIZE); // get pressure median

            // Check for launch given pressure decrease
            if ((frame_buffer.ground_ref - current_value) > LAUNCH_THRESHOLD) {
              flightStage = ASCENT;
              printf("FLIGHT STAGE = ASCENT\r\n");

              // Log all data from the buffer
              for (int i = 0; i < WINDOW_SIZE; i++) {
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
          get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, \
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);

          // Update buffer and window  
          update_buffer(&frame, &frame_buffer);

          // Get window median readings
          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data[i] = frame_buffer.window[i].barometer.pressure;
          }
          current_value = get_median(_data, WINDOW_SIZE); // get pressure median

          // Check for apogee given pressure increase
          if (current_value - previous_value > APOGEE_THRESHOLD){
            flightStage = APOGEE;
            printf("FLIGHT STAGE = APOGEE\r\n");
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
          get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, \
                          _BME280_data, _GNSS_data);

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
          get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, \
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);

          // Update buffer and window  
          update_buffer(&frame, &frame_buffer);

          // Get window median readings
          int _data[WINDOW_SIZE];
          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data[i] = frame_buffer.window[i].barometer.pressure;
          }

          // Check for landing
          if (is_stationary(_data)) {
            flightStage = LANDING;
            printf("FLIGHT STAGE = LANDING\r\n");
          }
        }
        break;

      case LANDING:
        STM32_beep_buzzer(200, 200, 1);
        break;
    }
  }
}
