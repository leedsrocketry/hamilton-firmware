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
  //uint16_t led_R = PIN('B', 9);
  //uint16_t led_G = PIN('B', 8);
  uint16_t led_B = PIN('H', 3);

  //gpio_set_mode(led_R, GPIO_MODE_OUTPUT);
  //gpio_set_mode(led_G, GPIO_MODE_OUTPUT);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  pwr_vdd2_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);
  //uart_init(UART1, 9600);
  //uart_init(UART3, 9600); 

  uint32_t timer = 0, period = 500;

  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on = true;                                 // This block is executed
      gpio_write(led_B, on);                            // Every `period` milliseconds
      on = !on;                                       // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks);  // Write message
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
  return 0;
}
