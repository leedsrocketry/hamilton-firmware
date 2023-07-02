/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
    Description: Driver file for the Barometer module MS561101BA03 (https://www.te.com/usa-en/product-MS560702BA03-50.html)
*/

#include "MS5611_driver.h"
#include "stdint.h"
#include "mcu.h"


// min OSR by default
static uint8_t pressAddr  = PRESSURE_OSR_256;
static uint8_t tempAddr   = TEMP_OSR_256;
static uint32_t convDelay = CONVERSION_OSR_256;


int8_t init_MS5611()
{
	return 0;
}


/* ---------------------------------- Private Functions ---------------------------------- */