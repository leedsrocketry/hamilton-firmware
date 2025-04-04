/*
        Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: Header file for the STM32L4R5 firmware
*/
#ifndef STM32_INIT_H
#define STM32_INIT_H

#include "HAL/mcu.h"
#include "debug.h"

// System parameters
#define SEA_LEVEL_PRESSURE_AT_SITE 1013.25  // change on the day
#define MSL_ALT_TIMER_THRESHOLD_MS 60000    // 1 minute

// Pins
#define BUZZER PIN('E', 6)

#define BLUE_LED_0 PIN('C', 13)
#define BLUE_LED_1 PIN('C', 14)
#define BLUE_LED_2 PIN('C', 15)

#define RBG_LED_RED PIN('E', 2)
#define RGB_LED_GREEN PIN('E', 3)
#define RGB_LED_BLUE PIN('E', 4)
typedef struct Date {
  unsigned int year;
  unsigned int month;
  unsigned int day;
} Date;

typedef struct Time {
  unsigned int hour;
  unsigned int minute;
  unsigned int second;
} Time;

// Functions
/**
  @brief Initialisation of the STM32L5 board
*/
void STM32_init(void);

/**
  @brief Sets the system clock frequency
  @param frequency - Pass either RCC_CFGR_SW_MSI, RCC_CFGR_SW_HSI, or
  RCC_CFGR_SW_PLL
*/
void STM32_init_clock(unsigned long frequency);

/**
  @brief Sets up addition timers
*/
void init_delay_timer();

/**
  @brief Initialisation of the STM32L4R5 board internals (UART, SPI, Power,
  etc.)
*/
void STM32_init_internals(void);

/**
  @brief Initialisation of the STM32L5 board externals (GPIO, ADC, etc.)
*/
void STM32_init_peripherals(void);

void STM32_beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps);

void STM32_indicate_on();

void STM32_blink_flash();

void STM32_super_beep();

#endif /* STM32_INIT_H */
