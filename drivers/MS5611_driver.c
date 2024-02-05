/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Evan Madurai
    Created on: 15 December 2023
    Description: Driver file for the Barometer module MS561101BA03 (https://www.te.com/usa-en/product-MS560702BA03-50.html)
*/

#include "MS5611_driver.h"
#include "stdint.h"
#include "mcu.h"

// min OSR by default
//static uint8_t pressAddr  = PRESSURE_OSR_256;
//static uint8_t tempAddr   = TEMP_OSR_256;
//static uint32_t convDelay = CONVERSION_OSR_256;

typedef struct PROM_data
{
    uint16_t blank;         // "factory data and the setup" ???
    uint16_t SENS;       // C1 - Pressure Sensitivity
    uint16_t OFF;           // C2 - Pressure Offset
    uint16_t TCS;           // C3 - Temperature coefficient of pressure sensitivity
    uint16_t TCO;           // C4 - Temperature coefficient of pressure offset
    uint16_t T_REF;             // C5 - Reference temperature
    uint16_t TEMPSENS;      // C6 - Temperature coefficient of the temperature 
    uint16_t SC_CRC         // Serial code and CRC
} PROM_data;

PROM_data ms5611_prom_data;

typedef struct M5611_data
{
    int32_t temp;
    int64_t pressure;
} M5611_data;

uint8_t MS5611_init()
{
    set_cs(MS5611_CS);
    //spi_enable_cs(SPI1);
    uint8_t init = spi_transmit(SPI1, MS5611_CMD_RESET);
    printf("MS5611 init\r\n");
    //spi_disable_cs(SPI1);
    unset_cs();

    MS5611_read_PROM(SPI1);
    M5611_data data;
    MS5611_get_data(&data);
    printf("Temp: %u\r\n", data.temp);
	return 0;
}

int32_t MS5611_get_data_test()
{
    spi_enable_cs(SPI1, MS5611_CS);
    uint8_t init = spi_transmit(SPI1, MS5611_CMD_RESET);
    printf("Init: %u\r\n", init); // 254 if works correctly
    spi_disable_cs(SPI1, MS5611_CS);

    MS5611_read_PROM(SPI1);
    M5611_data data;
    MS5611_get_data(&data);
    printf("Temp: %u\r\n", data.temp);
	return 0;
}


/* ---------------------------------- Private Functions ---------------------------------- */

uint16_t PROM_values[8];

uint8_t MS5611_read_PROM()
{
    for(int i = 0; i < 8; i++)
    {
        spi_enable_cs(SPI1, MS5611_CS);
        uint16_t result = spi_transmit_receive(SPI1, MS5611_CS, MS5611_CMD_READ_PROM(i), 1, 2);
        PROM_values[i] = result;
        spi_disable_cs(SPI1, MS5611_CS);
    }

    ms5611_prom_data.blank = PROM_values[0];
    ms5611_prom_data.SENS = PROM_values[1];
    ms5611_prom_data.OFF = PROM_values[2];
    ms5611_prom_data.TCS = PROM_values[3];
    ms5611_prom_data.TCO = PROM_values[4];
    ms5611_prom_data.T_REF = PROM_values[5];
    ms5611_prom_data.TEMPSENS = PROM_values[6];
    ms5611_prom_data.SC_CRC = PROM_values[7];
    return 0;
}

int MS5611_get_data(M5611_data* data)
{
    spi_transmit_receive(SPI1, MS5611_CS, MS5611_CMD_CONVERT_D2, 1, 1);
    delay(1);
    int32_t D2 = spi_transmit_receive(SPI1, MS5611_CS, MS5611_CMD_READ_ADC, 1, 3);
    int32_t dT = (D2) - ((int32_t)ms5611_prom_data.T_REF << 8);
    int32_t TEMP = 2000 + dT * ms5611_prom_data.TEMPSENS / (2<<23);

    spi_transmit_receive(SPI1, MS5611_CS, MS5611_CMD_CONVERT_D1, 1, 1);
    delay(1);
    int32_t D1 = spi_transmit_receive(SPI1, MS5611_CS, MS5611_CMD_READ_ADC, 1, 3);
    int64_t OFF = (ms5611_prom_data.OFF << 16) + (ms5611_prom_data.TCO*dT)/(2<<7);
    int64_t SENS = (ms5611_prom_data.SENS<<15) + (ms5611_prom_data.TCS*dT)/(2<<8);
    int64_t PRESSURE = ((((int64_t) D1 * SENS) >> 21) - OFF) >> 15;
    data->temp = TEMP;
    data->pressure = PRESSURE;

    return 0;
}
