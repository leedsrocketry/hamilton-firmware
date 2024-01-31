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

//#include "drivers/ADXL375_driver.h"
//#include "drivers/LSM6DS3_driver.h"

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

// void send_data() {
//   //setControlPins(WRITE_PROTECT);                  // Write Protection
//   //setControlPins(WRITE_PROTECT_OFF);              // Write Protection Off

//   uint8_t dataArray[128];
//   _memset(dataArray, 0x0, 128);

//   for (uint8_t i = 0; i < 128; i ++) {
//     dataArray[i] = i;
//   }
//   //eraseBlock(0);
//   //eraseALL();
//   //writeFrame(0, dataArray);
//   //readFrame(10000, dataArray);

//   FrameArray _input = unzip(dataArray);
//   // frameArray _output;
//   //int data_intact = 0;
//   //int data_fixed = 0;
//   //int data_error = 0;

//   //int startAddr = frameAddressPointer;
//   int numOfFramesToTest = 100;

//   for (int i = 0; i < numOfFramesToTest; i++) {
//     for (uint8_t j = 0; j < 128; j ++) {
//       dataArray[j] = j;
//     }

//     dataArray[0] = 0;
//     dataArray[1] = 0;
//     _input = unzip(dataArray);

//     log_frame(_input);
//     printf("======================== DONE ========================");
//   }

//   printf("==================== DONE WRITING ====================\r\n");
//   read_all();
//   print_capacity_info();
// };

/**
  @brief TODO
*/
void toggle_timeout_flag()
{
}

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
#define RESET 0x1E
#define PROM_READ(address) (0xA0 | ((address) << 1))
#define CONVERT_D1_COMMAND 0x40
#define CONVERT_D2_COMMAND 0x50
#define READ_ADC_COMMAND 0x00
int main(void)
{

  init_STM32();

  uint8_t init = spi_transmit(SPI1, 0x1E);
  //   delay(1);

  printf("hello world!!\n");

  uint32_t result = 0;

  for (int i = 0; i < 8; i++)
  {
    // Method 1: single function method
    //uint32_t r = spi_transmit_receive(SPI1, PROM_READ(i), 1, 2);

    // Method 2: manual method
    // Enable chip select, transmit command, receive data
    spi_enable_cs(SPI1);
    spi_transmit(SPI1, PROM_READ(i));
    // Transmit Dummy data to receive data
    uint8_t r0 = spi_transmit(SPI1, 0x00);
    uint8_t r1 = spi_transmit(SPI1, 0x00);
    // Combine byte response into one uint16
    uint16_t result = (uint16_t)((r1 << 8) | r0);
    printf("r: %hu\n", result);
    spi_disable_cs(SPI1);
  }

  // Read more data specific to barometer
  spi_transmit_receive(SPI1, CONVERT_D2_COMMAND, 1, 1);
  delay(1);
  uint32_t p = spi_transmit_receive(SPI1, READ_ADC_COMMAND, 1, 3);
  printf("p: %hu\n", p);
}

/*
init_STM32(); // Initialise the board
printf("==================== PROGRAM START ==================\r\n");

// Initialise the drivers
printf("================ INITIALISE FC DRIVERS ===============\`r\n");
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
