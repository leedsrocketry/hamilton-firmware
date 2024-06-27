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

uint8_t MS5611_init(SPI_TypeDef* spi)
{
    MS5611_SPI = spi;
    spi_enable_cs(MS5611_SPI, MS5611_CS);
    uint8_t cmd = MS5611_CMD_RESET;
    spi_transmit(MS5611_SPI, cmd);
    spi_disable_cs(MS5611_SPI, MS5611_CS);
    MS5611_read_PROM(MS5611_SPI);
    M5611_data data;
    MS5611_get_data(&data);
	return 0;
}

/* ---------------------------------- Private Functions ---------------------------------- */

uint16_t PROM_values[8];

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
    }
    return 0;
}

int MS5611_get_data(M5611_data* data)
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

    int32_t dT = ((int32_t)D2) - ((int32_t)ms5611_prom_data.T_REF << 8);
    int32_t TEMP = 2000 + dT * ms5611_prom_data.TEMPSENS / (2<<23);

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

    int64_t OFF = (int64_t)((ms5611_prom_data.OFF * pow(2,16)) + (ms5611_prom_data.TCO*dT)/pow(2,7));
    int64_t SENS = (int64_t)((ms5611_prom_data.SENS * pow(2,15)) + (ms5611_prom_data.TCS*dT)/pow(2,8));
    int32_t PRESSURE = (int32_t)(((int32_t)D1 * (int32_t)SENS / pow(2,21) - (int32_t)OFF) / pow(2,15)); // Should SENS really be cast here?
    data->temp = TEMP;
    data->pressure = PRESSURE;

    return 0;
}
