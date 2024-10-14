/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Evan Madurai
    Created on: 15 December 2023
    Description: Driver file for the Barometer module MS561101BA03 (https://www.te.com/usa-en/product-MS560702BA03-50.html)
*/

#include "MS5611_driver.h"
#include "stdint.h"

// min OSR by default
// static uint8_t pressAddr  = PRESSURE_OSR_256;
// static uint8_t tempAddr   = TEMP_OSR_256;
// static uint32_t convDelay = CONVERSION_OSR_256;

PROM_data ms5611_prom_data;

// TODO: Determine if this is actually set anywhere...?
SPI_TypeDef* MS5611_SPI;

/**
  @brief Init MS5611 Barometer driver
  @param spi Selected SPI
  @warning Return code not implemented
  @return Error code
*/
uint8_t MS5611_init(SPI_TypeDef* spi)
{
    MS5611_SPI = spi;
    spi_enable_cs(MS5611_SPI, MS5611_CS);
    uint8_t cmd = MS5611_CMD_RESET;
    uint8_t code = spi_transmit(MS5611_SPI, cmd);
    spi_disable_cs(MS5611_SPI, MS5611_CS);
    if(code != 254)
    {
      return 1;
    }
    MS5611_read_PROM(MS5611_SPI);
    //M5611_data data;
    // MS5611_get_data(&data);
    // LOG("%d\r\n", data.temp);
	return 0;
}

/* ---------------------------------- Private Functions ---------------------------------- */

uint16_t PROM_values[8];

/**
  @brief Read PROM data from Barometer into ms5611_prom_data structure
  @warning Error code not implemented
  @return Error code
*/
uint8_t MS5611_read_PROM()
{
    // Take ptr to the PROM data struct
    int16_t *prom_ptr = (int16_t *)&ms5611_prom_data;
    for(int i = 0; i < 8; i++)
    {
        uint16_t result;
        spi_enable_cs(MS5611_SPI, MS5611_CS);
        uint8_t cmd = MS5611_CMD_READ_PROM(i);
        spi_transmit_receive(MS5611_SPI, &cmd, 1, 2, (uint32_t*)&result);
        spi_disable_cs(MS5611_SPI, MS5611_CS);
        
        // Fill struct using ptr arithmatic
        *(prom_ptr + i) = (int16_t)result;
        //LOG("PROM: %d\r\n", result);
        delay_ms(10);
    }
    return 0;
}

/**
  @brief Get pressure and temperature data off MS5611 sensor and place into MS5611_data struct
  @param data Structure for Barometer data to be placed in
  @warning Error code not implemented
  @return Error code
*/
uint32_t MS5611_get_data(M5611_data* data)
{
    uint8_t cmd;
    // check if the device has a register that checks if the conversion is complete?
    spi_enable_cs(MS5611_SPI, MS5611_CS);
    cmd = MS5611_CMD_CONVERT_D2;
    spi_transmit_receive(MS5611_SPI, &cmd, 1, 0, NULL);
    spi_disable_cs(MS5611_SPI, MS5611_CS);

    delay_microseconds(600);

    uint32_t D2 = 0;
    spi_enable_cs(MS5611_SPI, MS5611_CS);
    cmd = MS5611_CMD_READ_ADC;
    spi_transmit_receive(MS5611_SPI, &cmd, 1, 3, &D2);
    spi_disable_cs(MS5611_SPI, MS5611_CS);

    spi_enable_cs(MS5611_SPI, MS5611_CS);
    cmd = MS5611_CMD_CONVERT_D1;
    spi_transmit_receive(MS5611_SPI, &cmd, 1, 0, NULL);
    spi_disable_cs(MS5611_SPI, MS5611_CS);

    delay_microseconds(600);

    uint32_t D1;
    spi_enable_cs(MS5611_SPI, MS5611_CS);
    cmd = MS5611_CMD_READ_ADC;
    spi_transmit_receive(MS5611_SPI, &cmd, 1, 3, &D1);
    spi_disable_cs(MS5611_SPI, MS5611_CS);

    calculate_pressure(D1, D2, data);

    return 0;
}

int32_t calculate_pressure(int32_t D1, int32_t D2, M5611_data* data)
{
    int32_t dT = ((int32_t)D2) - ((int32_t)ms5611_prom_data.T_REF << 8);
    int32_t TEMP = 2000 + dT * ms5611_prom_data.TEMPSENS / (2<<23);

    int64_t OFF = ((int64_t)ms5611_prom_data.OFF << 16) + ((int64_t)ms5611_prom_data.TCO * dT >> 7);
    int64_t SENS = ((int64_t)ms5611_prom_data.SENS << 15) + ((int64_t)ms5611_prom_data.TCS * dT >> 8);

    // Second order temperature compensation
    int64_t T2, OFF2, SENS2;
    if (TEMP < 2000) {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * ((TEMP - 2000)*(TEMP - 2000)) >> 1;
        SENS2 = 5 * ((TEMP - 2000)*(TEMP - 2000)) >> 2;
        if (TEMP < -1500) {
            OFF2 = OFF2 + (7 * ((TEMP + 1500)*(TEMP + 1500)));
            SENS2 = SENS2 + (11 * ((TEMP + 1500)*(TEMP + 1500)) >> 1);
        }
    } else {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2


    int32_t PRESSURE = (int32_t)(((int64_t)D1 * SENS >> 21) - OFF) >> 15;    data->temp = TEMP;
    data->pressure = PRESSURE;
}
