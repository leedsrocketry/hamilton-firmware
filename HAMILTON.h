/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
	Last modified on: 10 June 2023
  Description: Main header file for the STM32L4R5 firmware
*/

#define HAMILTON_H
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
int initHamilton(void);


/**
  @brief TODO
*/
void initInternals(void);


/**
  @brief TODO
*/
void initPeripherals(void);


/**
  @brief TODO
  @param onDurationMs
  @param offDurationMs
  @param noOfBeeps
*/
void beepBuzzer(int onDurationMs, int offDurationMs, int noOfBeeps);


/**
  @brief TODO
*/
void indicateOnBuzzer();


/**
  @brief TODO
*/
void indicateOnRedLed();


/**
  @brief TODO
*/
void ledsOn();


/**
  @brief TODO
*/
void ledsOff();


/**
  @brief TODO
*/
float getBatteryCapacity(uint8_t batteryNo);
