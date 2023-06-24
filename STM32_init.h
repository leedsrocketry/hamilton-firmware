/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: Header file for the STM32L4R5 firmware
*/
#ifndef STM32_DRIVER_H
#define STM32_INIT_H
#include "mcu.h"

// System parameters
#define SEA_LEVEL_PRESSURE_AT_SITE 1013.25 // change on the day
#define MSL_ALT_TIMER_THRESHOLD_MS 60000 // 1 minute

// Required structures
typedef enum FlightStages {LAUNCHPAD, ASCEND, APOGEE, DESCENT, LANDING} FlightStages;


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


// Pins
const uint16_t _vBatt   = PIN('A', 0);  
const uint16_t _vBatt1  = PIN('A', 0); 
const uint16_t _vBatt2  = PIN('A', 0); 
const uint16_t _vBatt3  = PIN('A', 0); 
const uint16_t _buzzer  = PIN('A', 0); 
const uint16_t _blueLED = PIN('B', 7);


// Functions
/**
  @brief TODO
*/
int init_STM32(void);


/**
  @brief TODO
*/
void init_internals(void);


/**
  @brief TODO
*/
void init_peripherals(void);


/**
  @brief TODO
  @param onDurationMs
  @param offDurationMs
  @param noOfBeeps
*/
void beep_buzzer(int onDurationMs, int offDurationMs, int noOfBeeps);


/**
  @brief TODO
*/
void indicate_on_buzzer();


/**
  @brief TODO
*/
void indicate_on_red_led();


/**
  @brief TODO
*/
void leds_on();


/**
  @brief TODO
*/
void leds_off();


/**
  @brief TODO
*/
float get_battery_capacity(uint8_t batteryNo);


#endif /* STM32_DRIVER_H */
