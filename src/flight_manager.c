/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 26 July 2024
  Description: System for managing flight
*/

#include "flight_manager.h"

M5611_data _M5611_data;
ADXL375_data _ADXL375_data;
LSM6DS3_data _LSM6DS3_data;
BME280_data _BME280_data;
GNSS_Data _GNSS_data;

FlightStage flightStage = LAUNCHPAD;

FlightStage get_flight_stage()
{
    return flightStage;
}

void set_flight_stage(FlightStage fs)
{
    flightStage = fs;
}

// void setup_flight_computer()
// {
//     STM32_init();
//     STM32_indicate_on();
//     init_flash();
//     initalise_drivers();
// }

void initalise_drivers()
{
    _BME280_data.temperature = 0;
    _BME280_data.pressure = 0;
    _BME280_data.humidity = 0;
    _GNSS_data.latitude = 0;
    _GNSS_data.longitude = 0;
    _GNSS_data.altitude = 0;
    _GNSS_data.velocity = 0;

    // Sensor initialisation
    MS5611_init(SPI1);                  // Barometer
    ADXL375_init(SPI1);                 // Accelerometer
    LSM6DS3_init(SPI1, &_LSM6DS3_data); // IMU
}

void run_flight()
{
    // STM32 setup
    STM32_init();

    LOG("================ PROGRAM START ================\r\n");
    STM32_indicate_on();

    LOG("============ INITIALISE NAND FLASH ============\r\n");
    // init_flash();

    LOG("============== INITIALISE DRIVERS =============\r\n");
    initalise_drivers();

    // Buffer
    FrameArray frame;                         // initialise the frameArray that keeps updating
    uint8_t dataArray[128];                   // dummy array to store the frame data
    _memset(dataArray, 0, sizeof(dataArray)); // set the necessary memory and set values to 0
    frame = unzip(&dataArray);                // convert from normal array into FrameArray
    dataBuffer frame_buffer;                  // contains FrameArrays
    init_buffer(&frame_buffer);               // initialise the buffer

    // Additional variables
    int _data[WINDOW_SIZE];
    int previous_pressure = 999999999;
    int current_pressure = 999999999;
    int current_velocity = 0;
    int apogee_incr = 3;
    float accel_vector[4] = {0, 0, 0, 0};
    bool toggle_LED = true;

    LOG("============== ADD TESTS HERE ==============\r\n");
    // NAND_flash_read();
    // NAND_flash_erase();
    // run_BME280_routine();

    LOG("============= ENTER MAIN PROCEDURE ============\r\n");
    uint32_t newTime = get_time_us();
    uint32_t oldTime = get_time_us();
    uint32_t dt = 0;
    delay_microseconds(1000 * 1000); // One second delay before launch for sensors to stabilise

    for (;;)
    {
        flightStage = get_flight_stage();
        switch (flightStage)
        {
        case LAUNCHPAD:
            newTime = get_time_us(); // Get current time
            if ((newTime - oldTime) > (1000000 / PADREADFREQ))
            {
                oldTime = newTime;                 // old time = new time
                gpio_write(GREEN_LED, toggle_LED); // Flick LED
                toggle_LED = !toggle_LED;

                // Get the sensor readings
                update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
                get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                                _BME280_data, _GNSS_data);

                // Update buffer and window
                update_buffer(&frame, &frame_buffer);
                if (frame_buffer.count > WINDOW_SIZE * 2)
                {
                    // Get the window barometer median
                    for (int i = 0; i < WINDOW_SIZE; i++)
                    {
                        _data[i] = frame_buffer.window[i].barometer.pressure;
                    }
                    current_pressure = get_median(_data, WINDOW_SIZE); // get pressure median

                    // Check if rocket is stationary based on acceleration
                    LSM6DS3_acc_read(&_LSM6DS3_data, &accel_vector); // This is clearly wrong

                    // Check for launch given pressure decrease
                    if ((frame_buffer.ground_ref - current_pressure) > LAUNCH_THRESHOLD && accel_vector[3] > 1.4)
                    {
                        set_flight_stage(ASCENT);
                        LOG("FLIGHT STAGE = ASCENT\r\n");

                        // Log data from the the last window
                        for (int i = 0; i < 10; i++)
                        {
                            log_frame(frame_buffer.frames[i]);
                        }
                    }
                }
            }
            break;

        case ASCENT:
            newTime = get_time_us(); // Get current time
            if ((newTime - oldTime) > (1000000 / ASCENTREADFREQ))
            {
                oldTime = newTime; // Old time = new time

                // Get the sensor readings
                update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
                get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                                _BME280_data, _GNSS_data);

                // Log data
                log_frame(frame);

                // Update buffer and window
                update_buffer(&frame, &frame_buffer);

                // Get window median readings
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    _data[i] = frame_buffer.window[i].barometer.pressure;
                }
                current_pressure = get_median(_data, WINDOW_SIZE); // get pressure median

                // Check for apogee given pressure increase
                if (current_pressure - previous_pressure > APOGEE_THRESHOLD)
                {
                    set_flight_stage(APOGEE);
                    LOG("FLIGHT STAGE = APOGEE\r\n");
                }
                else if (previous_pressure > current_pressure)
                { // Storing the minimum, (median), pressure value during ascent
                    previous_pressure = current_pressure;
                }
            }
            break;

        case APOGEE:
            newTime = get_time_us(); // Get current time
            if ((newTime - oldTime) > (1000000 / APOGEEREADFREQ))
            {
                oldTime = newTime; // Old time = new time

                // Get the sensor readings
                update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
                get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                                _BME280_data, _GNSS_data);

                // Log data
                log_frame(frame);

                // Update buffer and window
                update_buffer(&frame, &frame_buffer);

                // Run for a few cycles to record apogee when switch to descent
                if (apogee_incr == 0)
                    set_flight_stage(DESCENT);
                else
                    apogee_incr--;
            }
            break;

        case DESCENT:
            newTime = get_time_us(); // Get current time
            if ((newTime - oldTime) > (1000000 / DESCENTREADFREQ))
            {
                oldTime = newTime; // Old time = new time

                // Get the sensor readings
                update_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
                get_frame_array(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                                _BME280_data, _GNSS_data);

                // Log data
                log_frame(frame);

                // Update buffer and window
                update_buffer(&frame, &frame_buffer);

                // Get window median readings
                int _data[WINDOW_SIZE];
                LSM6DS3_data _data_imu[WINDOW_SIZE];

                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    _data_imu[i] = frame_buffer.window[i].imu;
                    _data[i] = frame_buffer.window[i].barometer.pressure;
                }
                current_pressure = get_median(_data, WINDOW_SIZE); // get pressure median

                // Check for landing
                if (LSM6DS3_gyro_standard_dev(_data_imu, WINDOW_SIZE, 1500) && (frame_buffer.ground_ref - current_pressure) < GROUND_THRESHOLD)
                {
                    set_flight_stage(LANDING);
                    LOG("FLIGHT STAGE = LANDING\r\n");
                }
            }
            break;

        case LANDING:
            STM32_beep_buzzer(200, 200, 1);
            break;
        }
    }
}