/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 10 June 2023
    Description: Driver file for the Pressure/Temp/humidity module BME280 
*/

#include "BME280_driver.h"
#include "stdint.h"s
#include "mcu.h"

// TODO: get pin number
#define pin 0x44  // ADDRESS


void init_BME280() {
    // soft reset
    Command reset = SoftReset;
    gpio_write(PIN, reset);
    delay_microseconds(2);

    // check the serial connection to the sensor
    uint32_t serial;
    int16_t ret;
    uint16_t serial_words[SENSIRION_NUM_WORDS(*serial)];
};


static float get_temperature(Precision precision) {
    if (precision != NULL) {
        return get_sensor_data(precision).temperature;
    } else {
        return 0;
    }
}


static float get_humidity(Precision precision){
    if (precision != NULL) {
        return get_sensor_data(precision).humidity;
    } else {
        return 0;
    }
}


static void enable_heater(Heater level) {
    gpio_write(PIN, enumValue(level));
}


/**
    @brief Get a measurement without using the heater. The measurement takes about 10ms
    @param precision Precision for the conversion
    @return SensorData struct containing the humidity value in % and the sensor temperature
 */
static SensorData get_sensor_data(Precision precision) {
    // Get sensor data
    gpio_write(PIN, enumValue(precision));
    delay_microseconds(10);	// Delay until the measurement is done, there is no other option
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
