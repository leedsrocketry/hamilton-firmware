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
//#include "drivers/LSM6DS3_driver.h"
//#include "drivers/SI446_driver.h"

// Flags
FlightStages flightStage = LAUNCHPAD;
volatile uint32_t s_ticks;

#pragma region Buzzer-LED
/**
  @brief Required for compilation
*/
void SysTick_Handler(void)
{
  s_ticks++;
}

/**
  @brief Buzzer sound
  @param onDurationMs
  @param offDurationMs
  @param noOfBeeps
*/
void STM32_beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps)
{
  for (int i = 0; i < noOfBeeps; i++) {
      gpio_write(_buzzer, HIGH);
      delay_ms(onDurationMs);
      gpio_write(_buzzer, LOW); 
      delay_ms(offDurationMs);
  }
}

/**
  @brief Buzzer sound to indicate power on
*/
void STM32_indicate_on_buzzer()
{
  STM32_beep_buzzer(100, 100, 3);
}

/**
  @brief Led light to indicate power on
*/
void STM32_indicate_on_led()
{
  STM32_led_on();
  delay_ms(100);
  STM32_led_off();
  delay_ms(100);
  STM32_led_on();
}
#pragma endregion Buzzer-LED

/**
  @brief Retrieve data from sensors
  @param _M5611_data - MS5611 barometer data
  @param _ADXL375_data - ADXL375 accelerometer data
  @param _LSM6DS3_data - LSM6DS3 IMU data

*/
void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data) {
  
  MS5611_get_data(&_M5611_data);
  ADXL375_get_data(&_ADXL375_data);
}

/**
  @brief Send data to the ground station
  @param frameAddressPointer - Address of the frame in the NAND flash
  @param _M5611_data - MS5611 barometer data
  @param _ADXL375_data - ADXL375 accelerometer data
*/
void log_data(uint32_t frameAddressPointer,
              M5611_data* _M5611_data, 
              ADXL375_data* _ADXL375_data) {
  set_control_pins(WRITE_PROTECT);      // Write Protection
  set_control_pins(WRITE_PROTECT_OFF);  // Write Protection Off

  // Convert data to frame
  Vector3 _acc_high_g = { _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z };

  // Add data to the frame
  FrameArray _frameArray;
  int startAddr = frameAddressPointer;
  _frameArray.barometer = _M5611_data->pressure;
  _frameArray.temp = _M5611_data->temp;
  _frameArray.accelHighG = _acc_high_g; 

  // Add data to NAND flash
  log_frame(_frameArray);
}

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
  pwr_vdd2_init();
  STM32_indicate_on_led();
  STM32_indicate_on_buzzer();
  printf("================ PROGRAM START ================\r\n");

  printf("=============== INITIALISE FLASH ==============\r\n");
  // init_flash();
  // erase_all();
  // uint32_t frameAddressPointer = 0;

  printf("============== INITIALISE DRIVERS =============\r\n");
  MS5611_init(SPI1);      // Barometer
  ADXL375_init(SPI1);     // Accelerometer
  // LSM6DS3_init(SPI1);  // IMU
  // BME280_init(SPI1);   // Temperature + Humidity
  // SI446_init(SPI1);    // Pad Radio

  printf("============= ENTER MAIN PROCEDURE ============\r\n");
  for (;;) {
    #pragma region Flight Stages
    switch (flightStage) {
      case LAUNCHPAD:
        // TODO
        // save a circular buffer of sensor readings (when launch is detected)
        // check for altitude off GPS and -> Barometer
        // check for acceleration (mostly)
        // if above threshold, change flightStage to ASCEND
        break;

      case ASCEND:
        // TODO
        // update all sensor readings at high rate
        // save sensor readings to FDR/over telemetry link
        // detect apogee based on gradient of altitude
        break;

      case APOGEE:
        // TODO
        // update all sensor readings at high rate
        // save sensor readings to FDR/over telemetry link
        // trigger recovery system
        break;

      case DESCENT:
        // TODO
        // update all sensor readings at lower rate
        // save sensor readings to FDR/over telemetry link
        // if no acceleration/costant altitude, change flightStage to LANDING
        break;

      case LANDING:
        // TODO
        // stop recording
        // change buzzer sequence
        // change LED sequence
        break;
    }

    //send_data();
  }

  printf("===================== PROGRAM END ====================");
}
