/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
    Description: Driver file for the Pressure/Temp/humidity module BME280 
*/

#include "BME280_driver.h"
#include "stdint.h"s
#include "mcu.h"


int8_t BME280_init(BME280_dev *dev) {
    int8_t retVal;
    uint8_t chipId = 0;

    // Read the chip-id of bme280 sensor
    retVal = BME280_get_regs(BME280_REG_CHIP_ID, &chipId, 1, dev);

    // Check for chip id validity
    if (retVal == BME280_OK)
    {
        if (chipId == BME280_CHIP_ID)
        {
            dev->chipId = chipId;
            retVal = BME280_soft_reset(dev);      // Reset the sensor

            if (retVal == BME280_OK)
            {
                retVal = get_calib_data(dev);     // Get calibration data
            }
        } else {
            retVal = BME280_E_DEV_NOT_FOUND;
        }
    }

    return retVal;
};


int8_t BME280_soft_reset(BME280_dev *dev)
{
    int8_t retVal;
    uint8_t regAddr = BME280_REG_RESET;
    uint8_t statusReg = 0;
    uint8_t tryRun = 5;
    uint8_t softRstCmd = BME280_SOFT_RESET_COMMAND; // 0xB6

    // Write the soft reset command in the sensor
    retVal = BME280_set_regs(&regAddr, &softRstCmd, 1, dev);

    if (retVal == BME280_OK)
    {
        do
        {
            // As per data sheet - Table 1, startup time is 2 ms
            delay_microseconds(BME280_STARTUP_DELAY);
            retVal = BME280_get_regs(BME280_REG_STATUS, &statusReg, 1, dev);

        } while ((retVal == BME280_OK) && (tryRun--) && (statusReg & BME280_STATUS_IM_UPDATE));

        if (statusReg & BME280_STATUS_IM_UPDATE)
        {
            retVal = BME280_E_NVM_COPY_FAILED;
        }
    }

    return retVal;
}


int8_t BME280_get_regs(uint8_t regAddr, uint8_t *regData, uint32_t len, BME280_dev *dev)
{
    int8_t retVal;
    retVal = null_ptr_check(dev); // Check for null pointer in the device structure

    if ((retVal == BME280_OK) && (regData != NULL))
    {        
        regAddr = regAddr | 0x80;                           // SPI 
        dev->intfRslt = dev->read(regAddr, regData, len);   // Read the data

        // Check for communication error
        if (dev->intfRslt != BME280_INTF_RET_SUCCESS)
        {
            retVal = BME280_E_COMM_FAIL;
        }
    } else {
        retVal = BME280_E_NULL_PTR;
    }

    return retVal;
}


int8_t BME280_set_regs(uint8_t *regAddr, const uint8_t *regData, uint32_t len, BME280_dev *dev)
{
    int8_t retVal;
    uint8_t tempBuff[20]; // Typically not to write more than 10 registers
    uint32_t tempLen;
    uint32_t regAddrCnt;

    if (len > 10) // max allowed length is 10
    {
        len = 10;
    }

    // Check for null pointer in the device structure 
    retVal = null_ptr_check(dev);

    // Check for arguments validity 
    if ((retVal == BME280_OK) && (regAddr != NULL) && (regData != NULL))
    {
        if (len != 0)
        {
            tempBuff[0] = regData[0];

            // SPI
            for (regAddrCnt = 0; regAddrCnt < len; regAddrCnt++)
            {
                regAddr[regAddrCnt] = regAddr[regAddrCnt] & 0x7F;
            }

            // Burst write mode
            if (len > 1)
            {
                // Interleave register address w.r.t data for burst write
                interleave_reg_addr(regAddr, tempBuff, regData, len);
                tempLen = ((len * 2) - 1);
            } else { 
                tempLen = len;
            }

            dev->intfRslt = dev->write(reg_addr[0], tempBuff, tempLen, dev->intfRslt);

            // Check for communication error 
            if (dev->intfRslt != BME280_INTF_RET_SUCCESS)
            {
                retVal = BME280_E_COMM_FAIL;
            }
        } else {
            retVal = BME280_E_INVALID_LEN;
        }
    } else {
        retVal = BME280_E_NULL_PTR;
    }

    return retVal;
}


int8_t BME280_get_sensor_data(uint8_t sensorComp, BME280_data *compData, BME280_dev *dev)
{
    int8_t retVal;

    // Array to store the pressure, temperature and humidity data 
    uint8_t regData[BME280_LEN_P_T_H_DATA] = { 0 };
    BME280_uncompData uncompData = { 0 };

    if (compData != NULL)
    {
        /* Read the pressure and temperature data from the sensor */
        retVal = BME280_get_regs(BME280_REG_DATA, regData, BME280_LEN_P_T_H_DATA, dev);

        if (retVal == BME280_OK)
        {
            // Parse the read data from the sensor 
            parse_sensor_data(regData, &uncompData);

            // Compensate the pressure and/or temperature and/or humidity data from the sensor
            retVal = BME280_compensate_data(sensorComp, &uncompData, compData, &dev->calibData);
        }
    }
    else
    {
        retVal = BME280_E_NULL_PTR;
    }

    return retVal;
}


int8_t BME280_compensate_data(uint8_t sensorComp, const BME280_uncompData *uncompData,
                              BME280_data *compData, BME280_calibData *calibData)
{
    int8_t retVal = BME280_OK;

    if ((uncompData != NULL) && (compData != NULL) && (compData != NULL))
    {
        // Initialize to zero
        compData->temperature = 0;
        compData->pressure = 0;
        compData->humidity = 0;

        // If pressure or temperature component is selected 
        if (sensorComp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {
            // Compensate the temperature data 
            compData->temperature = compensate_temperature(uncompData, calibData);
        }

        if (sensorComp & BME280_PRESS)
        {
            // Compensate the pressure data
            compData->pressure = compensate_pressure(uncompData, calibData);
        }

        if (sensor_comp & BME280_HUM)
        {
            // Compensate the humidity data
            compData->humidity = compensate_humidity(uncompData, calibData);
        }
    }
    else
    {
        retVal = BME280_E_NULL_PTR;
    }

    return retVal;
}


/* ---------------------------------- Private Functions ---------------------------------- */
/**
 @brief This private function is used to validate the device structure pointer for
 null conditions.
*/
static int8_t null_ptr_check(BME280_dev *dev)
{
    int8_t retVal;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL))
    {
        // Device structure pointer is not valid 
        retVal = BME280_E_NULL_PTR;
    }
    else
    {
        // Device structure is fine 
        retVal = BME280_OK;
    }

    return retVal;
}


/**
 @brief This private function reads the calibration data from the sensor, parse
 it and store in the device structure.
*/
static int8_t get_calib_data(BME280_dev *dev)
{
    int8_t retVal;
    uint8_t regAddr = BME280_REG_TEMP_PRESS_CALIB_DATA;

    /* Array to store calibration data */
    uint8_t calibData[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

    /* Read the calibration data from the sensor */
    retVal = BME280_get_regs(regAddr, calibData, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

    if (retVal == BME280_OK)
    {
        // Parse temperature and pressure calibration data and store it in device structure
        parse_temp_press_calib_data(calibData, dev);
        regAddr = BME280_REG_HUMIDITY_CALIB_DATA;

        // Read the humidity calibration data from the sensor
        retVal = BME280_get_regs(regAddr, calibData, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

        if (retVal == BME280_OK)
        {
            // Parse humidity calibration data and store in device structure
            parse_humidity_calib_data(calibData, dev);
        }
    }

    return retVal;
}


/**
 @brief This private function is used to parse the temperature and
 pressure calibration data and store it in device structure.
*/
static void parse_temp_press_calib_data(const uint8_t *regData, BME280_dev *dev)
{
    BME280_calibData *calibData = &dev->calibData;

    calibData->dig_t1 = BME280_CONCAT_BYTES(regData[1], regData[0]);
    calibData->dig_t2 = (int16_t)BME280_CONCAT_BYTES(regData[3], regData[2]);
    calibData->dig_t3 = (int16_t)BME280_CONCAT_BYTES(regData[5], regData[4]);
    calibData->dig_p1 = BME280_CONCAT_BYTES(regData[7], regData[6]);
    calibData->dig_p2 = (int16_t)BME280_CONCAT_BYTES(regData[9], regData[8]);
    calibData->dig_p3 = (int16_t)BME280_CONCAT_BYTES(regData[11],regData[10]);
    calibData->dig_p4 = (int16_t)BME280_CONCAT_BYTES(regData[13],regData[12]);
    calibData->dig_p5 = (int16_t)BME280_CONCAT_BYTES(regData[15],regData[14]);
    calibData->dig_p6 = (int16_t)BME280_CONCAT_BYTES(regData[17],regData[16]);
    calibData->dig_p7 = (int16_t)BME280_CONCAT_BYTES(regData[19],regData[18]);
    calibData->dig_p8 = (int16_t)BME280_CONCAT_BYTES(regData[21],regData[20]);
    calibData->dig_p9 = (int16_t)BME280_CONCAT_BYTES(regData[23],regData[22]);
    calibData->dig_h1 = regData[25];
}


/**
 @brief This private function is used to parse the humidity calibration data
 and store it in device structure.
*/
static void parse_humidity_calib_data(const uint8_t *regData, BME280_dev *dev)
{
    struct bme280_calib_data *calibData = &dev->calibData;
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calibData->dig_h2 = (int16_t)BME280_CONCAT_BYTES(regData[1], regData[0]);
    calibData->dig_h3 = regData[2];
    dig_h4_msb = (int16_t)(int8_t)regData[3] * 16;
    dig_h4_lsb = (int16_t)(regData[4] & 0x0F);
    calibData->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)regData[5] * 16;
    dig_h5_lsb = (int16_t)(regData[4] >> 4);
    calibData->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calibData->dig_h6 = (int8_t)regData[6];
}


/**
    @brief This private function interleaves the register address between the
    register data buffer for burst write operation.
*/
static void interleave_reg_addr(const uint8_t *regAddr, uint8_t *tempBuff, const uint8_t *regData, uint32_t len)
{
    uint32_t index;

    for (index = 1; index < len; index++)
    {
        tempBuff[(index * 2) - 1] = regAddr[index];
        tempBuff[index * 2] = regData[index];
    }
}


/**
    @brief This private function is used to compensate the raw temperature data and
    return the compensated temperature data in integer data type.
*/
static int32_t compensate_temperature(const BME280_uncompData *uncompData, BME280_calibData *calibData)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperatureMin = -4000;
    int32_t temperatureMax = 8500;

    var1 = (int32_t)((uncompData->temperature / 8) - ((int32_t)calibData->dig_t1 * 2));
    var1 = (var1 * ((int32_t)calibData->dig_t2)) / 2048;
    var2 = (int32_t)((uncompData->temperature / 16) - ((int32_t)calibData->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calibData->dig_t3)) / 16384;
    calibData->t_fine = var1 + var2;
    temperature = (calibData->t_fine * 5 + 128) / 256;

    if (temperature < temperatureMin)
    {
        temperature = temperatureMin;
    } 
    else if (temperature > temperatureMax)
    {
        temperature = temperatureMax;
    }

    return temperature;
}


/**
    @brief This private function is used to compensate the raw pressure data and
    return the compensated pressure data in integer data type with high accuracy.
*/
static uint32_t compensate_pressure(const BME280_uncompData *uncompData, const BME280_calibData *calibData)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressureMin = 3000000;
    uint32_t pressureMax = 11000000;

    var1 = ((int64_t)calibData->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibData->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calibData->dig_p5) * 131072);
    var2 = var2 + (((int64_t)calibData->dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calibData->dig_p3) / 256) + ((var1 * ((int64_t)calibData->dig_p2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calibData->dig_p1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - uncompData->pressure;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)calibData->dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)calibData->dig_p8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calibData->dig_p7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressureMin)
        {
            pressure = pressureMin;
        }
        else if (pressure > pressureMax)
        {
            pressure = pressureMax;
        }
    } else {
        pressure = pressureMin;
    }

    return pressure;
}


/**
    @brief This internal API is used to compensate the raw humidity data and
    return the compensated humidity data in integer data type.
*/
static uint32_t compensate_humidity(const BME280_uncompData *uncompData,
                                    const BME280_calibData *calibData)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidityMax = 102400;

    var1 = calibData->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncompData->humidity * 16384);
    var3 = (int32_t)(((int32_t)calibData->dig_h4) * 1048576);
    var4 = ((int32_t)calibData->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calibData->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calibData->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calibData->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calibData->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidityMax)
    {
        humidity = humidityMax;
    }

    return humidity;
}
