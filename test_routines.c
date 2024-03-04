/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 03 March 2024
  Description: Header Drivers test routines.
*/

#include "test_routines.h"

/**
  @brief Initial Routine to run on hardware. Should trigger RGB blink sequence
  and Serial printing via LUART1.
*/
void run_test_routine()
{
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);

  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message
    }
  }
}

/**
  @brief Routine to test the MS5611 barometer.
*/
void run_MS5611_routine()
{
  printf("================ MS5611_routine ================\r\n");
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  M5611_data _data;

  uint32_t timer = 0, period = 100;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                // This block is executed
      gpio_write(led_B, on);                // Every `period` milliseconds
      on = !on;
      MS5611_get_data(&_data); // Write message
      printf("Temp: %u Pressure: %u \r\n", _data.temp, _data.pressure);
    }
  }
}

/**
  @brief Routine to test the ADXL375 accelerometer.
*/
void run_ADXL375_routine()
{
  printf("================ ADXL375_routine ================\r\n");
  ADXL375_data _data;
  
  uint32_t timer = 0, period = 500;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      ADXL375_get_data(&_data);
    }
  }
}

/**
  @brief Routine to test the SI446 radio module.
*/
void SI446_Test_routine()
{
  printf("================ SI446_routine ================\r\n");
  int8_t ret_val = 123;
  ret_val = SI446_init(SPI1);
  printf("completed: %d \r\n ", ret_val);
}

/**
  @brief Routine to test the LSM6DS3 IMU.
*/
void LSM6DS3_test_routine()
{
  printf("================ LSM6DS3_routine ================\r\n");
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message

      int8_t ret_val = 123;
      ret_val = LSM6DS3_init(SPI1);
      printf("completed: %d \r\n ", ret_val);
    }
  }
}

/**
  @brief Routine to test the SPI communication.
*/
void spi_test_routine()
{
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message
      //spi_write_byte(SPI1,0);
      uint8_t ret_val = 0;
      //spi_write_byte(SPI1,124);
      //ret_val = spi_read_byte(SPI1);
      printf("return: %d", ret_val);
    }
  }
}


/**
  @brief Routine to test NAND Flash reading and writing.
*/
/*
void NAND_flash_test_routine()
{
  printf("==================== START WRITING ====================\r\n");
  set_control_pins(WRITE_PROTECT);      // Write Protection
  set_control_pins(WRITE_PROTECT_OFF);  // Write Protection Off

  uint8_t dataArray[128];
  _memset(dataArray, 0x0, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    dataArray[i] = i;
  }

  erase_block(0);
  erase_all();

  write_frame(0, dataArray);
  read_frame(10000, dataArray, 8);
  FrameArray _input = unzip(dataArray);
  FrameArray _output;

  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  // int startAddr = frameAddressPointer;

  int numOfFramesToTest = 100;
  for (int i = 0; i < numOfFramesToTest; i++) {
    for (uint8_t j = 0; j < 128; j ++) {
      dataArray[j] = j;
    }
    dataArray[0] = 0;
    dataArray[1] = 0;
    _input = unzip(dataArray);
    log_frame(_input);
    printf("======================== DONE ========================\r\n");
  }
  printf("==================== DONE WRITING ====================\r\n");
  
  read_all();
  print_capacity_info();
}*/
