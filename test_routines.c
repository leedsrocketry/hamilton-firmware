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
  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(GREEN_LED, on);                         // Every `period` milliseconds
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
  M5611_data _data;

  uint32_t timer = 0, period = 100;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                // This block is executed
      gpio_write(GREEN_LED, on);                // Every `period` milliseconds
      on = !on;
      M5611_data data;
      MS5611_get_data(&data);
      printf("p: %d, t: %d, \r\n", data.pressure, data.temp);
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
void run_SI446_routine()
{
  printf("================ SI446_routine ================\r\n");
  int8_t ret_val = 123;
  ret_val = SI446_init(SPI1);
  printf("completed: %d \r\n ", ret_val);
}

void run_LSM6DS3_routine()
{
  printf("================ LSM6DS3_routine ================\r\n");
  LSM6DS3_data gyro_data;
  delay_ms(50);
  lsm6ds6_init(SPI1, &gyro_data);
  
  uint32_t timer = 0, period = 500;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      lsm6ds6GyroReadAngle(SPI1, &gyro_data);
      printf("Gyro: %d, %d, %d, \r\n", gyro_data.x, gyro_data.y, gyro_data.z);
    }
  
  }
}

/**
  @brief Routine to test the SPI communication.
*/
void spi_test_routine()
{
  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(GREEN_LED, on);                         // Every `period` milliseconds
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
  uint8_t dataArray[128];
  _memset(dataArray, 0x0, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    dataArray[i] = i;
  }
  
  FrameArray _input = unzip(dataArray);

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
}
*/
