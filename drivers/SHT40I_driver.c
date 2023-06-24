/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
    Description: Driver file for the Temp/humidity module SHT40I-AD1B-R2 (https://uk.farnell.com/sensirion/sht40-ad1b-r2/humidity-temperature-sensor-i2c/dp/3586476?gclid=CjwKCAjw4ZWkBhA4EiwAVJXwqQtCjSk4-cUQJt99RfvJhjQVMMUUEyUq-ltkNxoET4AltHRyoIrJfRoCohAQAvD_BwE&mckv=s_dc|pcrid|602911506946|kword|sht40%20ad1b%20r2|match|p|plid||slid||product||pgrid|140308037249|ptaid|kwd-1030226716710|&CMP=KNC-GUK-GEN-SKU-MDC-SEMIS)
*/

#include "SHT40I_driver.h"
#include "stdint.h"
#include "mcu.h"

// TODO: get pin number
#define PIN 0

/**
    @brief Heater commands for the SHT40-AD1B-R2 sensor
    @warning Do not use heater for extended periods of time. The heater is designed for a maximal duty cycle of less than 5% when it is periodically heated
*/
typedef enum Header {
    _200mWFor1s	  = 0x39,		// 200mW @ 3.3V for 1s
    _200mWFor0p1s = 0x32,		// 200mW @ 3.3V for 0.1s
    _110mWFor1s   = 0x2F,		// 110mW @ 3.3V for 1s
    _110mWFor0p1s = 0x24,		// 110mW @ 3.3V for 0.1s
    _20mWFor1s	  = 0x1E,		// 20mW  @ 3.3V for 1s
    _20mWFor0p1s  = 0x15,		// 20mW  @ 3.3V for 0.1s
} Header;

typedef enum Precision {
    High 	 	= 0xFD,		///< measure T & RH with High precision (High repeatability)
    Medium  	= 0xF6,		///< measure T & RH with medium precision (medium repeatability)
    Low 		= 0xE0,		///< measure T & RH with lowest precision (low repeatability)
} Precision;


/**
    @brief Humidity sensor SHT40-AD1B-R2 initialization
*/
void init() {
    Command reset = SoftReset;
    gpio_write(PIN, reset);
    delayMicroseconds(2);
    return 1;
};


/**
    @brief Get the temperature without using the heater. The measurement takes about 10ms
    @param precision Precision for the conversion
    @return Temperature sensor temperature
*/
static float getTemperature(Precision precision) {
    if (precision != NULL) {
        return getSensorData(precision).temperature;
    } else {
        return 0;
    }
}


/**
    @brief Get the temperature without using the heater. The measurement takes about 10ms
    @param precision Precision for the conversion
    @return Temperature sensor humidity
*/
static float getHumidity(Precision precision){
    if (precision != NULL) {
        return getSensorData(precision).humidity;
    } else {
        return 0;
    }
}


/**
    @brief Enable the heater module on the sensor
    @warning The heater is designed for a maximal duty cycle of less than 5% when it is periodically heated
    @param level Power and duration command
*/
static void enableHeater(Header level) {
    gpio_write(PIN, enumValue(level));
}


/**
 * @brief Sensor data
 */
typedef struct SensorData {
    float humidity;				// Humidity sensor value
    float temperature;	        // Temperature sensor value
} SensorData;


/**
    @brief Get a measurement without using the heater. The measurement takes about 10ms
    @param precision Precision for the conversion
    @return SensorData struct containing the humidity value in % and the sensor temperature
 */
static SensorData getSensorData(Precision precision) {
    // Get sensor data
    gpio_write(PIN, enumValue(precision));
    delayMicroseconds(10);	// Delay until the measurement is done, there is no other option
    bool _rawData = gpio_read(PIN);

    // TODO Check data readout from _rawData to rawData
    SensorDataRaw rawData;

    // Filter data
    SensorData data;
    data.temperature = -45 + 175 * ((float)((uint16_t)(rawData.th) << 8 | rawData.tl)) / 0xFFFF;
    data.humidity = -6 + 125 * ((float)((uint16_t)(rawData.rhh) << 8 | rawData.rhl)) / 0xFFFF;
    if (data.humidity > 100) data.humidity = 100;
    else if(data.humidity < 0) data.humidity = 0;

    // Return sensor readings
    return data;
}

/**
    @brief Raw sensor data
*/
typedef struct SensorDataRaw {
    uint8_t th;			// Temperature high byte
    uint8_t tl;			// Temperature low byte
    uint8_t tcrc8;		// Temperature crc8 checksum
    uint8_t rhh;		// Relative humidity high byte
    uint8_t rhl;		// Relative humidity low byte
    uint8_t rhcrc8;		// Relative humidity crc8 checksum
} SensorDataRaw;


/**
 * @brief Commands for the SHT40-AD1B-R2 sensor
 */
typedef enum Command {
    ReadSerial = 0x89,		// Every single sensor has a unique serial number
    SoftReset  = 0x94,		// Soft reset
} Command;


static inline uint8_t DeviceAddress = 0x88;		///< I2C device address