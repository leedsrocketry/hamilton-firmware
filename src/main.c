/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>

#include "../segger-rtt/RTT/SEGGER_RTT.h"
#include "HAL/mcu.h"
#include "debug.h"
#include "drivers/_driver_manager.h"
#include "flight_manager.h"
#include "frame.h"
#include "lib/log.h"
#include "stm32l4r5xx.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/

int main(void) {
  // STM32 setup
  STM32_init();

  logi("================ PROGRAM START ================\r\n");
  STM32_indicate_on();

  logi("============ INITIALISE NAND FLASH ============\r\n");
  init_flash();
  print_capacity_info();

#ifdef ERASE_NAND
  erase_all();
  NAND_flash_read();
  delay_ms(100);
  return 0;
#endif

#ifdef READ_NAND
  delay_ms(1000);
  print_capacity_info();
  NAND_flash_read();
  return 0;
#endif

  logi("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

#ifdef SENSOR_TEST
  test_sensors();
#endif

#ifdef CALIBRATE
  logi("Getting calibration data...\r\n");
  calibrate_ADXL375();
  return 0;
#endif
  logi("============== INITIALISE FLIGHT ==============\r\n");
  delay_ms(2000);
  STM32_super_beep();
  run_flight();

  return 0;
}
