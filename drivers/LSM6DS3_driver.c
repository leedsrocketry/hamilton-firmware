/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Tyler Green
    Created on: 01 February 2024
    Description: Driver file for the IMU module LSM6DS3 (https://www.mouser.co.uk/datasheet/2/389/dm00133076-1798402.pdf)
*/

#include "LSM6DS3_driver.h"


SPI_TypeDef *LSM6DS3_SPI;
#pragma region Public   //___________ Public _________________

int8_t LSM6DS3_init(SPI_TypeDef *spi){
    int8_t LSM6DS3_retVal = 123;
    LSM6DS3_SPI = spi;
    // check if we recieve who am I correctly
    LSM6DS3_retVal = LSM6DS3_read_reg(LSM6DS3_WHO_AM_I_ADRS);
    if(LSM6DS3_retVal == LSM6DS3_WHO_AM_I_EXP){
        printf("LSM6DS3 knows who it is...\n");
        // power on the accelerometer
        LSM6DS3_write_reg(LSM6DS3_CTRL1_XL_ADRS, 01000000);
        LSM6DS3_write_reg(LSM6DS3_CTRL2_G_ADRS, 01000000)
    }
    else{
        LSM6DS3_retVal = -1;
    }
    return LSM6DS3_retVal;
};

int8_t LSM6DS3_get_data(LSM6DS3_data *data){
    int8_t LSM6DS3_retVal;
    spi_enable_cs(LSM6DS3_SPI, LSM6DS3_CS);
    spi_write_byte(LSM6DS3_SPI, LSM6DS3_TEMP_ADDRESS);
    data->LSM6DS3_TEMP = LSM6DS3_get_sens_val();
    data->LSM6DS3_GYRO_X = LSM6DS3_get_sens_val();
    data->LSM6DS3_GYRO_Y = LSM6DS3_get_sens_val();
    data->LSM6DS3_GYRO_Z = LSM6DS3_get_sens_val();
    data->LSM6DS3_ACC_X = LSM6DS3_get_sens_val();
    data->LSM6DS3_ACC_Y = LSM6DS3_get_sens_val();
    data->LSM6DS3_ACC_Z = LSM6DS3_get_sens_val();
    spi_disable_cs(LSM6DS3_SPI, LSM6DS3_CS);
    return 1;
};

#pragma endregion Public

#pragma region Private

static int16_t LSM6DS3_get_sens_val(){
    int16_t val1, val2;
    val1 = (int16_t)spi_read_byte(LSM6DS3_SPI);
    val2 = (int16_t)spi_read_byte(LSM6DS3_SPI) << 8;
    return val1 && val2;
};

static uint8_t LSM6DS3_write_reg(uint8_t address, uint8_t data){
    spi_enable_cs(LSM6DS3_SPI,LSM6DS3_CS);
    spi_write_byte(LSM6DS3_SPI,address);
    spi_write_byte(LSM6DS3_SPI,data);
    spi_disable_cs(LSM6DS3_SPI,LSM6DS3_CS);
};

static uint8_t LSM6DS3_read_reg(uint8_t address){
    address &= 0x80;   // make sure first bit is 1 
    uint8_t LSM6DS3_retVal = 123;
    spi_enable_cs(LSM6DS3_SPI,LSM6DS3_CS);
    spi_write_byte(LSM6DS3_SPI,address);
    LSM6DS3_retVal = spi_read_byte(LSM6DS3_SPI);
    spi_disable_cs(LSM6DS3_SPI,LSM6DS3_CS);
    return LSM6DS3_retVal;
};


#pragma endregion Private