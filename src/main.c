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

  // gpio_write(RBG_LED_RED, HIGH);
   gpio_write(RGB_LED_GREEN, HIGH);
  // gpio_write(RGB_LED_BLUE, HIGH);

   gpio_write(BLUE_LED_0, HIGH);
   gpio_write(BLUE_LED_1, HIGH);
   gpio_write(BLUE_LED_2, HIGH);

    int a = 16807;
    int m = 2147483647;
    int seed = (a * seed) % m;
    int random = seed / m;

  LOG("================ PROGRAM START %d ================\r\n", random);
//  STM32_indicate_on();

  LOG("============ INITIALISE NAND FLASH ============\r\n");
//  init_flash();


  LOG("============= INITIALISE SD CARD =============\r\n");

  SD_Card sd_card = init_sd_card(SPI1, CS5);
  printf("Version: %d\r\n", sd_card.version);
  printf("High capacity: %d\r\n", sd_card.high_capacity);
  printf("Compatible: %d\r\n", sd_card.compatible);
  printf("Initialised: %d\r\n", sd_card.initialised);

  uint8_t data[512];
  uint8_t err = sd_read_block(&sd_card, 0x00000020, data);
  LOG("%d \r\n", err);
  LOG("0x");
  for (uint16_t i = 0; i < 512; i++) {
      LOG("%02x", data[i]);
  }
  LOG("\r\n");

  for (;;){};
  LOG("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

  //run_flight();
  return 0;
}
