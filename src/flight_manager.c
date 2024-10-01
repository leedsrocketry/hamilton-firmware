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
double previous_ascent_altitude = 4294967294;
uint32_t apogee_index = 100; //amount of data recorded at apogee

double apogee = 0;

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

  if(MS5611_init(SPI1))
  {
    LOG("ERROR INITIALISING MS5611 BAROMETER\r\n");
  }
  if(ADXL375_init(SPI1))
  {
    LOG("ERROR INITIALISING ADXL375 ACCEL\r\n");
  }
  if(LSM6DS3_init(SPI1, &_LSM6DS3_data))
  {
    LOG("ERROR INITIALISING LSM6DS3 IMU\r\n");
  }
}

void handle_LAUNCHPAD(Frame* frame, FrameBuffer* fb)
{
  LOG("PAD\r\n");
  // READ
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  // FORMAT AND SEND DATA TO HC12
  char sensors_data_buffer[150];
  format_sensor_data(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data, sensors_data_buffer, sizeof(sensors_data_buffer)); // Format the data into a string
  HC12_transmit(UART1, sensors_data_buffer); // Transmit the formatted string over UART

  // BUILD
  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  // ANALYSE
  double current_pressure = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_PRESSURE);
  double current_temperature = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_TEMP) / 100;
  // TODO: calculate launch based on pressure+accel+gyro(?)
  frame->altitude = barometric_equation(current_pressure, 273.15+current_temperature); // Need to convert to kelvin temp

  // ACT
  bool accel_launch_flag = false;

  // Unused for now, will be later
  //bool baro_launch_flag = false;
  //bool gyro_launch_flag = false;

  if(_ADXL375_data.y < ACCEL_LAUNCH_THRESHOLD)
  {
    accel_launch_flag = true;
  }

  // if(velo > BARO_LAUNCH_THRESHOLD)
  // {
  //   baro_launch_flag = true;
  // }

  if(accel_launch_flag == true) // Test variations in emu
  {
    set_flight_stage(ASCENT);
  }

  // STORE
  log_frame(*frame);
  //write_framebuffer(fb);
}

void handle_ASCENT(Frame* frame, FrameBuffer* fb)
{
  LOG("ASCENT\r\n");
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  double current_pressure = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_PRESSURE);
  double current_temperature = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_TEMP) / 100;
  frame->altitude = barometric_equation(current_pressure, 273.15+current_temperature); // Need to convert to kelvin temp

  if(frame->altitude > apogee)
  {
    apogee = frame->altitude;
  }

  bool altitude_apogee_flag = false;

  if((apogee-frame->altitude) > ALTITUDE_APOGEE_THRESHOLD)
  {
    altitude_apogee_flag = true;
  }

  // ACT
  if(altitude_apogee_flag == true)
  {
    set_flight_stage(APOGEE);
  }

  previous_ascent_altitude = frame->altitude;

  // STORE
  log_frame(*frame);
}

void handle_APOGEE(Frame* frame, FrameBuffer* fb)
{
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  // TODO: calculate apogee based on sensor data
  // Gather lots of additional data at this phase, determining apogee is important

  // ACT
  apogee_index--;
  if(apogee_index == 0)
  {
    set_flight_stage(DESCENT);
  }

  // STORE
  log_frame(*frame);
}

void handle_DESCENT(Frame* frame, FrameBuffer* fb)
{
  LOG("DESCENT\r\n");
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);
  //int32_t current_pressure = get_framebuffer_median(fb, BUFFER_SIZE, MS5611_PRESSURE);
  // ACT
  bool gyro_landed_flag = false;

  if(is_stationary(fb))
  {
    gyro_landed_flag = true;
  }

  if(gyro_landed_flag == true)
  {
    set_flight_stage(LANDING);
  }

  // STORE
  log_frame(*frame);
}

void handle_LANDING(Frame* frame, FrameBuffer* fb)
{
  LOG("LAND\r\n");
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);
  // ACT
  // Beep? Signal? radio? idk
  STM32_indicate_on();

  // STORE
  log_frame(*frame);
}

void run_flight() {

  // Setup FrameBuffer (contains last 50 frames of data)
  Frame frame;        // initialise the Frame that keeps updating
  uint8_t dataArray[128];  // dummy array to store the frame data
  _memset(dataArray, 0,
          sizeof(dataArray));  // set the necessary memory and set values to 0
  frame = unzip(dataArray);   // convert from normal array into Frame
  FrameBuffer frame_buffer;     // contains FrameArrays
  init_frame_buffer(&frame_buffer);  // initialise the buffer

  // Additional variables

  LOG("============= GATHER INITIAL DATA ============\r\n");
  for(uint32_t i = 0; i < 100; i++)
  {
    read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);
    build_frame(&frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
    update_frame_buffer(&frame, &frame_buffer);
  }

  LOG("============= ENTER MAIN PROCEDURE ============\r\n");
  uint32_t newTime = get_time_us();
  uint32_t oldTime = get_time_us();
  uint32_t dt = 0;
  delay_microseconds(
      1000 * 1000);  // One second delay before launch for sensors to stabilise

  for (;;) {
    flightStage = get_flight_stage();

    newTime = get_time_us();
    dt = newTime-oldTime;
    //LOG("dt: %d\r\n", dt);
    switch (flightStage) {
      case LAUNCHPAD:
        handle_LAUNCHPAD(&frame, &frame_buffer);
        // if ((dt) > (1000000 / PADREADFREQ)) {
        //   handle_LAUNCHPAD(&frame, &frame_buffer);
        // }
        oldTime=newTime;
        break;

      case ASCENT:
        if ((dt) > (1000000 / ASCENTREADFREQ)) {
          handle_ASCENT(&frame, &frame_buffer);
        }
        break;

      case APOGEE:
        if ((dt) > (1000000 / APOGEEREADFREQ)) {
          handle_APOGEE(&frame, &frame_buffer);
        }
        break;

      case DESCENT:
        if ((dt) > (1000000 / DESCENTREADFREQ)) {
          handle_DESCENT(&frame, &frame_buffer);
        }
        break;

      case LANDING:
        handle_LANDING(&frame, &frame_buffer);
        //STM32_beep_buzzer(200, 200, 1);
        break;
    }
  }
}
