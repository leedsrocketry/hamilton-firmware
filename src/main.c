/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "HAL/mcu.h"
#include "frame_array.h"
#include "stm32l4r5xx.h"
#include "data_buffer.h"
#include "flight_manager.h"
#include "debug.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void)
{
  run_flight();
  return 0;
}
