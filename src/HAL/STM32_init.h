/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: Header file for the STM32L4R5 firmware
*/
#ifndef STM32_DRIVER_H
#define STM32_INIT_H

#include "HAL/mcu.h"

// System parameters
#define SEA_LEVEL_PRESSURE_AT_SITE 1013.25 // change on the day
#define MSL_ALT_TIMER_THRESHOLD_MS 60000 // 1 minute

// Pins
#define BUZZER PIN('B', 9)
#define GREEN_LED PIN('B', 8)

// Required structures
typedef enum FlightStages {LAUNCHPAD, ASCENT, APOGEE, DESCENT, LANDING} FlightStages;


typedef struct Date 
{
  unsigned int year;
  unsigned int month;
  unsigned int day;
} Date;


typedef struct Time
{
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
  @param frequency - Pass either RCC_CFGR_SW_MSI, RCC_CFGR_SW_HSI, or RCC_CFGR_SW_PLL
*/
void STM32_init_clock(unsigned long frequency);

/**
  @brief Sets up addition timers
*/
void init_delay_timer();

/**
  @brief Initialisation of the STM32L4R5 board internals (UART, SPI, Power, etc.)
*/
void STM32_init_internals(void);

/**
  @brief Initialisation of the STM32L5 board externals (GPIO, ADC, etc.)
*/
void STM32_init_peripherals(void);

void STM32_beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps);

void STM32_indicate_on();

#endif /* STM32_DRIVER_H */
