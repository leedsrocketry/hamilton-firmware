/*
        Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
    Description: Driver file for the Pressure/Temp/humidity module BME280
    Note: The current state of this driver (2024-07-29) has never been tested.
*/

#include "BME280_driver.h"

#pragma region Private
/* ---------------------------------- Private Functions
 * ---------------------------------- */
/**
 @brief Used to validate the device structure pointer for null conditions.
 @param dev Device struct
*/
static int8_t BME280_null_ptr_check(BME280_dev *dev) {
  int8_t ret_val;

  if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL)) {
    // Device structure pointer is not valid
    ret_val = BME280_E_NULL_PTR;
  } else {
    // Device structure is fine
    ret_val = 1;
  }

  return ret_val;
}

/**
 @brief This private function is used to parse the temperature and pressure
 calibration data and store it in device structure.
*/
static void BME280_parse_temp_press_calib_data(const uint8_t *reg_data, BME280_dev *dev) {
  BME280_calib_data *calib_data = &dev->calib_data;

  calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
  calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
  calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
  calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
  calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
  calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
  calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
  calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
  calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
  calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
  calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
  calib_data->dig_h1 = reg_data[25];
}

/**
 @brief This private function is used to parse the humidity calibration data and
 store it in device structure.
*/
static void BME280_parse_humidity_calib_data(const uint8_t *reg_data, BME280_dev *dev) {
  BME280_calib_data *calib_data = &dev->calib_data;
  int16_t dig_h4_lsb;
  int16_t dig_h4_msb;
  int16_t dig_h5_lsb;
  int16_t dig_h5_msb;

  calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  calib_data->dig_h3 = reg_data[2];
  dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
  dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
  calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
  dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
  dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
  calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
  calib_data->dig_h6 = (int8_t)reg_data[6];
}

/**
 @brief This private function reads the calibration data from the sensor, parse
 it and store in the device structure.
*/
static int8_t BME280_get_calib_data(BME280_dev *dev) {
  int8_t ret_val;
  uint8_t reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;

  /* Array to store calibration data */
  uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = {0};

  /* Read the calibration data from the sensor */
  ret_val = BME280_get_regs(reg_addr, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

  if (ret_val == 1) {
    // Parse temperature and pressure calibration data and store it in device
    // structure
    BME280_parse_temp_press_calib_data(calib_data, dev);
    reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;

    // Read the humidity calibration data from the sensor
    ret_val = BME280_get_regs(reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

    if (ret_val == 1) {
      // Parse humidity calibration data and store in device structure
      BME280_parse_humidity_calib_data(calib_data, dev);
    }
  }

  return ret_val;
}

/**
    @brief This private function interleaves the register address between the
   register data buffer for burst write operation.
*/
static void BME280_interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data,
                                       uint32_t len) {
  uint32_t index;

  for (index = 1; index < len; index++) {
    temp_buff[(index * 2) - 1] = reg_addr[index];
    temp_buff[index * 2] = reg_data[index];
  }
}

/**
    @brief This private function is used to compensate the raw temperature data
   and return the compensated temperature data in integer data type.
*/
static int16_t BME280_compensate_temperature(const BME280_uncomp_data *uncomp_data, BME280_calib_data *calib_data) {
  int16_t var1;
  int16_t var2;
  int16_t temperature;
  int16_t temperature_min = -4000;
  int16_t temperature_max = 8500;

  var1 = (int16_t)((uncomp_data->temperature / 8) - ((int16_t)calib_data->dig_t1 * 2));
  var1 = (int16_t)((var1 * ((int16_t)calib_data->dig_t2)) / 2048);
  var2 = (int16_t)((uncomp_data->temperature / 16) - ((int16_t)calib_data->dig_t1));
  var2 = (int16_t)((((var2 * var2) / 4096) * ((int16_t)calib_data->dig_t3)) / 16384);
  calib_data->t_fine = var1 + var2;
  temperature = (int16_t)((calib_data->t_fine * 5 + 128) / 256);

  if (temperature < temperature_min) {
    temperature = temperature_min;
  } else if (temperature > temperature_max) {
    temperature = temperature_max;
  }
  return temperature;
}

/**
    @brief This private function is used to compensate the raw pressure data and
   return the compensated pressure data in integer data type with high accuracy.
*/
static uint32_t BME280_compensate_pressure(const BME280_uncomp_data *uncomp_data, const BME280_calib_data *calib_data) {
  int64_t var1;
  int64_t var2;
  int64_t var3;
  int64_t var4;
  uint32_t pressure;
  uint32_t pressure_min = 3000000;
  uint32_t pressure_max = 11000000;

  var1 = ((int64_t)calib_data->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
  var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
  var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
  var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) + ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
  var3 = ((int64_t)1) * 140737488355328;
  var1 = (var3 + var1) * ((int64_t)calib_data->dig_p1) / 8589934592;

  /* To avoid divide by zero exception */
  if (var1 != 0) {
    var4 = 1048576 - uncomp_data->pressure;
    var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data->dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
    var2 = (((int64_t)calib_data->dig_p8) * var4) / 524288;
    var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
    pressure = (uint32_t)(((var4 / 2) * 100) / 128);

    if (pressure < pressure_min) {
      pressure = pressure_min;
    } else if (pressure > pressure_max) {
      pressure = pressure_max;
    }
  } else {
    pressure = pressure_min;
  }
  return pressure;
}

/**
    @brief This internal API is used to compensate the raw humidity data and
    return the compensated humidity data in integer data type.
*/
static uint32_t BME280_compensate_humidity(const BME280_uncomp_data *uncomp_data, const BME280_calib_data *calib_data) {
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  int32_t var5;
  uint32_t humidity;
  uint32_t humidity_max = 102400;

  var1 = calib_data->t_fine - ((int32_t)76800);
  var2 = (int32_t)(uncomp_data->humidity * 16384);
  var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
  var4 = ((int32_t)calib_data->dig_h5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
  var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  humidity = (uint32_t)(var5 / 4096);

  if (humidity > humidity_max) {
    humidity = humidity_max;
  }

  return humidity;
}
#pragma endregion Private

#pragma region Public
int8_t BME280_init(BME280_dev *dev, SPI_TypeDef *spi, uint8_t cs) {
  int8_t ret_val;
  uint8_t chip_ID = 0;

  dev->SPI = spi;
  dev->CS = cs;

  // Read the chip-id of bme280 sensor
  ret_val = BME280_get_regs(BME280_REG_CHIP_ID, &chip_ID, 1, dev);

  // Check for chip id validity
  if (chip_ID == BME280_CHIP_ID) {
    LOG("BME280 chip ID: %d\r\n", chip_ID);
    dev->chip_ID = chip_ID;
    ret_val = BME280_soft_reset(dev);  // Reset the sensor
    if (ret_val == 1) {
      ret_val = BME280_get_calib_data(dev);  // Get calibration data
    }
  } else {
    LOG("BME280 wrong device ID: %d\r\n", chip_ID);
    ret_val = BME280_E_DEV_NOT_FOUND;
  }

  return ret_val;
};

int8_t BME280_soft_reset(BME280_dev *dev) {
  int8_t ret_val;
  uint8_t reg_addr = BME280_REG_RESET;
  uint8_t status_reg = 0;
  uint8_t try_run = 5;
  uint8_t soft_rst_cmd = BME280_CMD_SOFT_RESET;  // 0xB6

  // Write the soft reset command in the sensor
  ret_val = BME280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

  if (ret_val == 1) {
    do {
      // As per data sheet - Table 1, startup time is 2 ms
      delay_microseconds(BME280_STARTUP_DELAY);
      ret_val = BME280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

    } while ((ret_val == 1) && (try_run--) && (status_reg & BME280_CMD_STATUS_IM_UPDATE));

    if (status_reg & BME280_CMD_STATUS_IM_UPDATE) {
      ret_val = BME280_E_NVM_COPY_FAILED;
    }
  }

  return ret_val;
}

int8_t BME280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, BME280_dev *dev) {
  int8_t ret_val;
  ret_val = BME280_null_ptr_check(dev);  // Check for null pointer in the device structure

  if ((ret_val == 1) && (reg_data != NULL)) {
    reg_addr = reg_addr | 0x80;
    spi_enable_cs(dev->SPI, dev->CS);
    spi_transmit_receive(dev->SPI, &reg_addr, (uint8_t)len, 1, reg_data);
    dev->intf_rslt = reg_data;
    spi_disable_cs(dev->SPI, dev->CS);

    // Check for communication error
    if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
      ret_val = BME280_E_COMM_FAIL;
    }
  }
  return ret_val;
}

int8_t BME280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint16_t len, BME280_dev *dev) {
  int8_t ret_val;
  uint8_t temp_buff[20];  // Typically not to write more than 10 registers
  uint8_t temp_len;
  uint32_t reg_addrCnt;

  if (len > 10)  // max allowed length is 10
    len = 10;

  // Check for null pointer in the device structure
  ret_val = BME280_null_ptr_check(dev);

  // Check for arguments validity
  if ((ret_val == 1) && (reg_addr != NULL) && (reg_data != NULL)) {
    if (len != 0) {
      temp_buff[0] = reg_data[0];

      // SPI
      for (reg_addrCnt = 0; reg_addrCnt < len; reg_addrCnt++) {
        reg_addr[reg_addrCnt] = reg_addr[reg_addrCnt] & 0x7F;
      }

      // Burst write mode
      if (len > 1) {
        // Interleave register address w.r.t data for burst write
        BME280_interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
        temp_len = (uint8_t)((len * 2) - 1);
      } else {
        temp_len = (uint8_t)len;
      }

      spi_enable_cs(dev->SPI, dev->CS);
      spi_transmit_receive(dev->SPI, temp_buff, temp_len, 0, NULL);
      spi_disable_cs(dev->SPI, dev->CS);
    }
  }
  return ret_val;
}

int8_t BME280_get_data(uint8_t sensor_comp, BME280_data *comp_data, BME280_dev *dev) {
  int8_t ret_val;

  // Array to store the pressure, temperature and humidity data
  uint8_t reg_data[BME280_LEN_P_T_H_DATA] = {0};
  BME280_uncomp_data uncomp_data = {0};

  if (comp_data != NULL) {
    /* Read the pressure and temperature data from the sensor */
    ret_val = BME280_get_regs(BME280_REG_DATA, reg_data, BME280_LEN_P_T_H_DATA, dev);

    if (ret_val == 1) {
      // Compensate the pressure and/or temperature and/or humidity data from
      // the sensor
      ret_val = BME280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
    }
  }
  return ret_val;
}

int8_t BME280_compensate_data(uint8_t sensor_comp, const BME280_uncomp_data *uncomp_data, BME280_data *comp_data,
                              BME280_calib_data *calib_data) {
  int8_t ret_val = 1;

  if ((uncomp_data != NULL) && (comp_data != NULL) && (comp_data != NULL)) {
    // Initialize to zero
    comp_data->temperature = 0;
    comp_data->pressure = 0;
    comp_data->humidity = 0;

    // If pressure or temperature component is selected
    if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) {
      // Compensate the temperature data
      comp_data->temperature = BME280_compensate_temperature(uncomp_data, calib_data);
    }

    if (sensor_comp & BME280_PRESS) {
      // Compensate the pressure data
      comp_data->pressure = BME280_compensate_pressure(uncomp_data, calib_data);
    }

    if (sensor_comp & BME280_HUM) {
      // Compensate the humidity data
      comp_data->humidity = BME280_compensate_humidity(uncomp_data, calib_data);
    }
  } else {
    ret_val = BME280_E_NULL_PTR;
  }

  return ret_val;
}
#pragma endregion Public
