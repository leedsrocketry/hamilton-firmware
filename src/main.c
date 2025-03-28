/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>

#include "HAL/mcu.h"
#include "debug.h"
#include "flight_manager.h"
#include "frame.h"
#include "frame_buffer.h"
#include "stm32l4r5xx.h"
#include "drivers/_driver_manager.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/

int main(void) {
  // STM32 setup
  STM32_init();

  LOG("================ PROGRAM START ================\r\n");
  STM32_indicate_on();

  LOG("============ INITIALISE NAND FLASH ============\r\n");
  init_flash();

#ifdef ERASE_NAND
  erase_all();
  NAND_flash_read();
  delay_ms(100);
  return 0;
#endif

#ifdef READ_NAND
  read_all_csv();
#endif

  LOG("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

#ifdef SENSOR_TEST
  test_sensors();
#endif

#ifdef CALIBRATE
  printf("Getting calibration data...\r\n");
  calibrate_ADXL375();
  return 0;
#endif

  run_flight();
  return 0;
}
