/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "mcu.h"
#include "NAND_flash_driver.h"
#include "STM32_init.h"


// Flags
FlightStages flightStage = LAUNCHPAD;


/**
  @brief Required for compilation
*/
static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}


/**
  @brief TODO
*/
void update_sensors() {

};


/**
  @brief TODO
*/
void send_data() {
  //setControlPins(WRITE_PROTECT);                  // Write Protection
  //setControlPins(WRITE_PROTECT_OFF);              // Write Protection Off
  
  uint8_t dataArray[128];
  _memset(dataArray, 0x0, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    dataArray[i] = i;
  }
  //eraseBlock(0);
  //eraseALL();
  //writeFrame(0, dataArray);
  //readFrame(10000, dataArray);
  
  FrameArray _input = unzip(dataArray);
  // frameArray _output;
  //int data_intact = 0;
  //int data_fixed = 0;
  //int data_error = 0;

  //int startAddr = frameAddressPointer;
  int numOfFramesToTest = 100;
  
  for (int i = 0; i < numOfFramesToTest; i++) {
    for (uint8_t j = 0; j < 128; j ++) {
      dataArray[j] = j;
    } 

    dataArray[0] = 0;
    dataArray[1] = 0;
    _input = unzip(dataArray);

    log_frame(_input);
    printf("======================== DONE ========================");
  }  
  
  printf("==================== DONE WRITING ====================\r\n");
  read_all();
  print_capacity_info();
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
int main(void) {
  init_STM32(); // Initialise the board
  printf("==================== PROGRAM START ==================\r\n");
  
  // Initialise the drivers
  printf("================ INITIALISE FC DRIVERS ===============\r\n");
  init_flash();
  //erase_all();
  frameAddressPointer = 0;
  // init_MAXM10S(); GNSS
  // init_SHT40I(); Temperature + Humidity
  // init_MS5611(); Barometer
  // init_ADXL375(); Accelerometer
  // init_L3GD20HTR(); Gyroscope
  // init_init_MAX31855(); Temperature
  // init_SI4463(); Pad Radio

  printf("================ ENTER MAIN PROCEDURE ================\r\n");
  uint16_t valueToSend = 1;

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

    // test spi
    // Write data to SPI
    valueToSend++;

    // Convert the integer to a byte array
    uint8_t byteBuffer[sizeof(valueToSend)];
    for (size_t i = 0; i < sizeof(valueToSend); ++i) {
      byteBuffer[i] = (uint8_t)(valueToSend >> (i * 8)) & 0xFF;
    }

    // Calculate the length of the byte array
    size_t bufferLength = sizeof(byteBuffer);
    
    spi_write_buf(SPI1, (char *)byteBuffer, bufferLength);

    // Wait for transfer to complete (until receive buffer is not empty)
    spi_ready_read(SPI1); 

    // Read received data from SPI
    uint16_t receivedValue = spi_read_byte(SPI1);

    // Convert the received byte array back to an integer
    for (size_t i = 0; i < sizeof(receivedValue); ++i) {
      receivedValue |= ((uint16_t)byteBuffer[i] << (i * 8));
    }

    // Print the received integer
    printf("Received Value: %hu\r\n", receivedValue);

    // Exit program
    //printf("===================== PROGRAM END ===================="); 
  }
  return 0;
}
