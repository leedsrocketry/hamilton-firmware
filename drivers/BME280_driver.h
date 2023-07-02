/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 10 June 2023
  Description: header file for the Pressure/Temp/humidity module BME280 (https://www.mouser.co.uk/datasheet/2/783/bst_bme280_ds002-2238172.pdf)
*/
#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H
#include "stdint.h"

#pragma region Macros
/** @name BME280 chip status */
#define BME280_OK                                 INT8_C(0)
#define BME280_E_NULL_PTR                         INT8_C(-1)
#define BME280_E_COMM_FAIL                        INT8_C(-2)
#define BME280_E_INVALID_LEN                      INT8_C(-3)
#define BME280_E_DEV_NOT_FOUND                    INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL                  INT8_C(-5)
#define BME280_E_NVM_COPY_FAILED                  INT8_C(-6)
#define BME280_W_INVALID_OSR_MACRO                INT8_C(1)

/** @name BME280 chip identifier */
#define BME280_CHIP_ID                            UINT8_C(0x60)

/** @name Register Address */
#define BME280_REG_CHIP_ID                        UINT8_C(0xD0)
#define BME280_REG_RESET                          UINT8_C(0xE0)
#define BME280_REG_TEMP_PRESS_CALIB_DATA          UINT8_C(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA            UINT8_C(0xE1)
#define BME280_REG_CTRL_HUM                       UINT8_C(0xF2)
#define BME280_REG_STATUS                         UINT8_C(0xF3)
#define BME280_REG_PWR_CTRL                       UINT8_C(0xF4)
#define BME280_REG_CTRL_MEAS                      UINT8_C(0xF4)
#define BME280_REG_CONFIG                         UINT8_C(0xF5)
#define BME280_REG_DATA                           UINT8_C(0xF7)

/** @name Commands */
#define BME280_SOFT_RESET_COMMAND                 UINT8_C(0xB6)
#define BME280_STATUS_IM_UPDATE                   UINT8_C(0x01)
#define BME280_STATUS_MEAS_DONE                   UINT8_C(0x08)

/** @name Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET                        UINT16_C(1250)
#define BME280_MEAS_DUR                           UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET               UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR                UINT16_C(1000)
#define BME280_STARTUP_DELAY                      UINT16_C(2000)

/** @name Macros related to size */
#define BME280_LEN_TEMP_PRESS_CALIB_DATA          UINT8_C(26)
#define BME280_LEN_HUMIDITY_CALIB_DATA            UINT8_C(7)
#define BME280_LEN_P_T_H_DATA                     UINT8_C(8)

/** @name Sensor component selection macros. Internal for API implementation. */
#define BME280_PRESS                              UINT8_C(1)
#define BME280_TEMP                               UINT8_C(1 << 1)
#define BME280_HUM                                UINT8_C(1 << 2)
#define BME280_ALL                                UINT8_C(0x07)

/** @name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/*!
 * BME280_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BME280_INTF_RET_TYPE
#define BME280_INTF_RET_TYPE                      int8_t
#endif
#pragma endregion Macros


#pragma region Pointers
/**
 @brief Bus communication function pointer which should be mapped to the specific read functions
 @param regAddr Register address from which data is read
 @param regData Pointer to data buffer where read data is stored
 @param len Number of bytes of data to be read
*/
typedef BME280_INTF_RET_TYPE (*BME280_read_fptr_t)(uint8_t regAddr, uint8_t *regData, uint32_t len);

/*!
 @brief Bus communication function pointer which should be mapped to the specific write functions
 @param reg_addr Register address to which the data is written
 @param reg_data Pointer to data buffer in which data to be written is stored
 @param len Number of bytes of data to be written
*/
typedef BME280_INTF_RET_TYPE (*BME280_write_fptr_t)(uint8_t regAddr, const uint8_t *regData, uint32_t len);


#pragma region Structs
/**
 * @brief BME280 device structure
 */
typedef struct BME280_dev
{
    // Chip Id 
    uint8_t chipId;

    // Variable to store result of read/write function 
    BME280_INTF_RET_TYPE intfRslt;

    // Read function pointer 
    BME280_read_fptr_t read;

    // Write function pointer 
    BME280_write_fptr_t write;

    // Trim data 
    BME280_calibData calib;
} BME280_dev;


/**
  @brief Sensor structure that holds compensated temp, pres and humidity data
*/
typedef struct BME280_data
{
  uint32_t pressure;    // Compensated pressure
  int32_t temperature;  // Compensated temperature
  uint32_t humidity;    // Compensated humidity
}; BME280_data;


/**
  @brief Sensor structure that holds uncompensated temp, pres and humidity data
*/
typedef struct BME280_uncompData
{
  uint32_t pressure;    // Un-compensated pressure
  uint32_t temperature; // Un-compensated temperature
  uint32_t humidity;    // Un-compensated humidity
} BME280_uncompData;


/**
  @brief Calibration data
*/
typedef struct BME280_calibData
{
  // Calibration coefficient for the temperature sensor 
  uint16_t dig_t1;  
  int16_t dig_t2;
  int16_t dig_t3;

  // Calibration coefficient for the pressure sensor 
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;

  // Calibration coefficient for the humidity sensor 
  uint8_t dig_h1;
  int16_t dig_h2;
  uint8_t dig_h3;
  int16_t dig_h4;
  int16_t dig_h5;
  int8_t dig_h6;

  // Variable to store the intermediate temperature coefficient 
  int32_t t_fine;
} BME280_calibData;
#pragma endregion Structs


#pragma region Functions
/**
  @brief Reads the chip-id of the sensor which is the first step to
 * verify the sensor and also calibrates the sensor
  @param dev Structure instance of BME280_dev
  @return Result of execution status
  @retval   0 -> Success
  @retval > 0 -> Warning    
  @retval < 0 -> Fail
*/
int8_t BME280_init(BME280_dev *dev);


/**
  @brief Soft-resets the sensor
  @param dev Structure instance of BME280_dev
  @return Result of execution status (same return as BME280_init) 
*/
int8_t BME280_soft_reset(BME280_dev *dev);


/**
  @brief Reads the data from the given register address of sensor
  @param regAddr Register address from where the data to be read
  @param regData Pointer to data buffer to store the read data
  @param len Number of bytes of data to be read
  @param dev Structure instance of BME280_dev
  @return Result of execution status (same return as BME280_init)
*/
int8_t BME280_get_regs(uint8_t regAddr, uint8_t *regData, uint32_t len, BME280_dev *dev);


/**
  @brief Writes the given data to the register address of the sensor
  @param regAddr Register addresses to where the data is to be written
  @param regData Pointer to data buffer which is to be written in the regAddr of sensor.
  @param len Number of bytes of data to write
  @param dev Structure instance of BME280_dev
  @return Result of execution status (same return as BME280_init)
 */
int8_t BME280_set_regs(uint8_t *regAddr, const uint8_t *regData, uint32_t len, BME280_dev *dev);


/**
  @brief Reads the pres, temp and humidity data from the sensor, compensates the data and store 
  it in the BME280_data structure instance passed by the user.
  @param sensorComp : Variable which selects which data to be read
  @verbatim
  sensorComp |   Macros
  -----------|-----------------
    1        |  BME280_PRESS
    2        |  BME280_TEMP
    4        |  BME280_HUM
    7        |  BME280_ALL
  @endverbatim
  @param compData : Structure instance of BME280_data.
  @param dev      : Structure instance of BME280_dev.

  @return Result of execution status
*/
int8_t BME280_get_sensor_data(uint8_t sensorComp, BME280_data *compData, BME280_dev *dev);


/**
  @brief Compensate the pres and/or temp and/or humidity data according to the selected component
  @param sensorComp : Used to select pressure and/or temp and/or humidity
  @param uncompData : Contains the uncompensated pres, temp and humidity data
  @param compData   : Contains the compensated pres and/or temp and/or humidity data
  @param calibData  : Pointer to BME280_calibData
  @return Result of execution status.
*/
int8_t BME280_compensate_data(uint8_t sensorComp, const BME280_uncompData *uncompData,
                              BME280_data *compData, BME280_calibData *calibData);
#pragma endregion Functions

#endif /* BME280_DRIVER_H */
