/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>

#include "HAL/mcu.h"
#include "data_buffer.h"
#include "debug.h"
#include "flight_manager.h"
#include "frame_array.h"
#include "stm32l4r5xx.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void) {

  // STM32 setup
  STM32_init();

  // gpio_write(RBG_LED_RED, HIGH);
  // gpio_write(RGB_LED_GREEN, HIGH);
  // gpio_write(RGB_LED_BLUE, HIGH);

  // gpio_write(BLUE_LED_0, HIGH);
  // gpio_write(BLUE_LED_1, HIGH);
  // gpio_write(BLUE_LED_2, HIGH);

  LOG("================ PROGRAM START ================\r\n");
  STM32_indicate_on();

  LOG("============ INITIALISE NAND FLASH ============\r\n");
  // init_flash();

  LOG("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

  run_flight();
  return 0;
}
