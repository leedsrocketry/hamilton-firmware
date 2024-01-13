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

#include "drivers/ADXL375_driver.h"
#include "drivers/LSM6DS3_driver.h"

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
  // printf("\n");

  // spi_transmit(SPI1, 0x1E);

  printf("hello world!\n");

  uint16_t b;
  uint16_t b1;
  uint16_t b2;

  for (int i = 0; i < 8; i++)
  {
    b1 = spi_transmit(SPI1, PROM_READ(i));
    b2 = spi_transmit(SPI1, 0x00);

    // Byte swap on the received value 'b'
    uint8_t *toSwap = (uint8_t *)&b1;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;

    // toSwap = (uint8_t *)&b2;
    // secondByte = toSwap[0];
    // toSwap[0] = toSwap[1];
    // toSwap[1] = secondByte;

    // b = (b1 << 8) | b2;

    printf("Received Value %d (after byte-swap): %hu\r\n", i, b1);
  }

  // b = spi_transmit(SPI1, CONVERT_D1_COMMAND);
  // b = spi_transmit(SPI1, READ_ADC_COMMAND);
  // printf("Received Value D1 (pre byte-swap): %hu\r\n", b);

  // uint8_t e = spi_transmit(SPI1, PROM_READ(1));

  // printf("Received Value: %hu\r\n", e);

  // printf("%d\n", d);
  // printf("A %hu\r\n", SPI1->SR & BIT(7));
  // spi_write_byte(SPI1, 0x1E);
  // printf("B %hu\r\n", SPI1->SR & BIT(7));
  // spi_ready_read(SPI1);
  // printf("C %hu\r\n", SPI1->SR & BIT(7));
  // spi_read_byte(SPI1);

  // Assuming spi_write_byte and spi_ready_read functions take correct parameters
  // gpio_set_mode(PIN('A', 4), GPIO_MODE_OUTPUT);
  // gpio_write(PIN('A', 4), LOW);
  // spi_write_byte(SPI1, RESET);
  // delay(3);
  // gpio_write(PIN('A', 4), HIGH);
  // // spi_ready_read(SPI1);
  // // spi_read_byte(SPI1);

  // // while (1)
  // // {
  // printf("\n");
  // uint8_t address;
  // for (address = 0; address < 32; address++)
  // {
  //   // Assuming spi_ready_read returns a non-zero value when ready

  //   gpio_write(PIN('A', 4), LOW);
  //   spi_write_byte(SPI1, PROM_READ(address));
  //   // spi_ready_read(SPI1);
  //   uint16_t testByte1 = spi_read_byte(SPI1);
  //   gpio_write(PIN('A', 4), HIGH);

  //   printf("Received Values %i: %hu \r\n", address, testByte1);
  // }
  // gpio_write(PIN('A', 4), HIGH);
  //  }
  //  while (1)
  //  {
  //    // spi_write_byte(SPI1, 32);
  //    spi_write_byte(SPI1, RESET);
  //    if (spi_ready_read(SPI1))
  //    {
  //      printf("Received Value: %hu\r\n", spi_read_byte(SPI1));
  //    }
  //  }
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
