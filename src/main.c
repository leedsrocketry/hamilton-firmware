/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>

#include "HAL/mcu.h"
#include "frame_buffer.h"
#include "debug.h"
#include "flight_manager.h"
#include "frame.h"
#include "stm32l4r5xx.h"

#include "drivers/sd_card.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void) {

  // STM32 setup
  STM32_init();

  LOG("================ PROGRAM START %d ================\r\n", random);
  STM32_indicate_on();

  LOG("============ INITIALISE NAND FLASH ============\r\n");
  init_flash();

  LOG("============= INITIALISE SD CARD =============\r\n");

  LOG("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

  run_flight();
  return 0;
}
