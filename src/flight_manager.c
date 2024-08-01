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

FlightStage get_flight_stage() { return flightStage; }

void set_flight_stage(FlightStage fs) { flightStage = fs; }

// void setup_flight_computer()
// {
//     STM32_init();
//     STM32_indicate_on();
//     init_flash();
//     initalise_drivers();
// }

void initalise_drivers() {
  _BME280_data.temperature = 0;
  _BME280_data.pressure = 0;
  _BME280_data.humidity = 0;
  _GNSS_data.latitude = 0;
  _GNSS_data.longitude = 0;
  _GNSS_data.altitude = 0;
  _GNSS_data.velocity = 0;

  // Sensor initialisation
  MS5611_init(SPI1);                   // Barometer
  ADXL375_init(SPI1);                  // Accelerometer
  LSM6DS3_init(SPI1, &_LSM6DS3_data);  // IMU
}

void handle_LAUNCHPAD(Frame* frame, FrameBuffer* fb)
{
  // READ
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
  LOG("build\r\n");

  // BUILD
  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);
  LOG("ANALYSE\r\n");
  // ANALYSE
  int32_t current_pressure = get_framebuffer_median(&fb, BUFFER_SIZE, MS5611_TEMP);
  // TODO: calculate launch based on pressure+accel+gyro(?)

  // ACT
  // if(launched)
  // {
  //   set_flight_stage(ASCENT);
  // }

  // STORE
  write_framebuffer(fb);
}

void run_flight() {

  // Buffer
  Frame frame;        // initialise the Frame that keeps updating
  uint8_t dataArray[128];  // dummy array to store the frame data
  _memset(dataArray, 0,
          sizeof(dataArray));  // set the necessary memory and set values to 0
  frame = unzip(&dataArray);   // convert from normal array into Frame
  FrameBuffer frame_buffer;     // contains FrameArrays
  init_buffer(&frame_buffer);  // initialise the buffer

  // Additional variables
  uint32_t _data[WINDOW_SIZE];
  uint32_t previous_pressure = 999999999;
  uint32_t current_pressure = 999999999;
  uint32_t current_velocity = 0;
  uint32_t apogee_incr = 3;
  float accel_vector[4] = {0, 0, 0, 0};
  bool toggle_LED = true;

  LOG("============= ENTER MAIN PROCEDURE ============\r\n");
  uint32_t newTime = get_time_us();
  uint32_t oldTime = get_time_us();
  uint32_t dt = 0;
  delay_microseconds(
      1000 * 1000);  // One second delay before launch for sensors to stabilise

  for (;;) {
    flightStage = get_flight_stage();
    switch (flightStage) {
      case LAUNCHPAD:
        newTime = get_time_us();  // Get current time

        if ((newTime - oldTime) > (1000000 / PADREADFREQ)) {
          oldTime = newTime;

          handle_LAUNCHPAD(&frame, &frame_buffer);
        }
        break;

      case ASCENT:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / ASCENTREADFREQ)) {
          oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          build_frame(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);

          // Update buffer and window
          update_frame_buffer(&frame, &frame_buffer);

          // Get window median readings
          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data[i] = frame_buffer.window[i].barometer.pressure;
          }
          current_pressure =
              get_median(_data, WINDOW_SIZE);  // get pressure median

          // Check for apogee given pressure increase
          if (current_pressure - previous_pressure > APOGEE_THRESHOLD) {
            set_flight_stage(APOGEE);
            LOG("FLIGHT STAGE = APOGEE\r\n");
          } else if (previous_pressure >
                     current_pressure) {  // Storing the minimum, (median),
                                          // pressure value during ascent
            previous_pressure = current_pressure;
          }
        }
        break;

      case APOGEE:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / APOGEEREADFREQ)) {
          oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          build_frame(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);

          // Update buffer and window
          update_frame_buffer(&frame, &frame_buffer);

          // Run for a few cycles to record apogee when switch to descent
          if (apogee_incr == 0)
            set_flight_stage(DESCENT);
          else
            apogee_incr--;
        }
        break;

      case DESCENT:
        newTime = get_time_us();  // Get current time
        if ((newTime - oldTime) > (1000000 / DESCENTREADFREQ)) {
          oldTime = newTime;  // Old time = new time

          // Get the sensor readings
          read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
          build_frame(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data,
                          _BME280_data, _GNSS_data);

          // Log data
          log_frame(frame);

          // Update buffer and window
          update_frame_buffer(&frame, &frame_buffer);

          // Get window median readings
          int _data[WINDOW_SIZE];
          LSM6DS3_data _data_imu[WINDOW_SIZE];

          for (int i = 0; i < WINDOW_SIZE; i++) {
            _data_imu[i] = frame_buffer.window[i].imu;
            _data[i] = frame_buffer.window[i].barometer.pressure;
          }
          current_pressure =
              get_median(_data, WINDOW_SIZE);  // get pressure median

          // Check for landing
          if (LSM6DS3_gyro_standard_dev(_data_imu, WINDOW_SIZE, 1500) &&
              (frame_buffer.ground_ref - current_pressure) < GROUND_THRESHOLD) {
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