/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "mcu.h"
// #include "NAND_flash_driver.h"
#include "STM32_init.h"
#include "stm32l4r5xx.h"

// Flags
FlightStages flightStage = LAUNCHPAD;

/**
  @brief Required for compilation
*/
static volatile uint32_t s_ticks;
void SysTick_Handler(void)
{
  s_ticks++;
}

/**
  @brief TODO
*/
void update_sensors(){

};

/**
  @brief TODO
*/
void toggle_timeout_flag()
{
}

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void)
{

  init_STM32();

  while (1)
  {
    spi_write_byte(SPI1, 32);
    if (spi_ready_read(SPI1))
    {
      printf("Received Value: %hu\r\n", spi_read_byte(SPI1));
    }
  }
}

/*
init_STM32(); // Initialise the board
printf("==================== PROGRAM START ==================\r\n");

// Initialise the drivers
printf("================ INITIALISE FC DRIVERS ===============\r\n");
init_flash();
//erase_all();
frameAddressPointer = 0;
// init_MAXM10S(); GNSS
// init_BME280(); Temperature + Humidity
// init_MS5611(); Barometer
// init_ADXL375(); Accelerometer
// init_LSM6DS3(); Gyroscope
// init_MAX31855(); Temperature
// init_SI446(); Pad Radio

printf("================ ENTER MAIN PROCEDURE ================\r\n");

for (;;) {
  // Complete based on flight stage
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
  #pragma endregion Flight Stages

  //send_data();
}

// Exit program
printf("===================== PROGRAM END ====================");
*/