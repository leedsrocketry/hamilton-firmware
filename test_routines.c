/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 03 March 2024
  Description: Header Drivers test routines.
*/

#include "test_routines.h"
#include "mcu.h"

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
  MS5611_init(SPI1);
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);

  uint32_t timer = 0, period = 100;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                // This block is executed
      gpio_write(led_B, on);                // Every `period` milliseconds
      on = !on;
      MS5611_get_data_test(); // Write message
    }
  }
}

/**
  @brief Routine to test the ADXL375 accelerometer.
*/
void run_ADXL375_routine()
{
  printf("================ ADXL375_routine ================\r\n");
  ADXL375_init(SPI1);
  ADXL375_data data;
  
  uint32_t timer = 0, period = 500;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      data = ADXL375_get_data();
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