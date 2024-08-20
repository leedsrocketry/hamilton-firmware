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

  M5611_data data;
  MS5611_get_data(&data);
  LOG("%d\r\n", data.temp);
}

void handle_LAUNCHPAD(Frame* frame, FrameBuffer* fb)
{
  //LOG("PAD\r\n");
  // READ
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  // BUILD
  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  // ANALYSE
  double current_pressure = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_PRESSURE);
  double current_temperature = (double)get_framebuffer_median(fb, BUFFER_SIZE, MS5611_TEMP) / 100;
  // TODO: calculate launch based on pressure+accel+gyro(?)
  frame->altitude = barometric_equation(current_pressure, 273.15+current_temperature); // Need to convert to kelvin temp
  double velo = get_vertical_velocity(fb);
  
  // ACT
  bool accel_launch_flag = false;
  bool baro_launch_flag = false;
  bool gyro_launch_flag = false;

  if(_ADXL375_data.x < ACCEL_LAUNCH_THRESHOLD)
  {
    accel_launch_flag = true;
  }

  // if(velo > BARO_LAUNCH_THRESHOLD)
  // {
  //   baro_launch_flag = true;
  // }

  if(baro_launch_flag == true)
  {
    set_flight_stage(ASCENT);
  }

  // STORE
  //write_framebuffer(fb);
}

void handle_ASCENT(Frame* frame, FrameBuffer* fb)
{
  LOG("ASCENT\r\n");
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  int32_t current_pressure = get_framebuffer_median(&fb, BUFFER_SIZE, MS5611_PRESSURE);
  // TODO: calculate apogee based on sensor data

  // ACT
  // if(launched)
  // {
  //   set_flight_stage(DESCENT);
  // }

  // STORE
  write_framebuffer(fb);
}

void handle_APOGEE(Frame* frame, FrameBuffer* fb)
{
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  int32_t current_pressure = get_framebuffer_median(&fb, BUFFER_SIZE, MS5611_PRESSURE);
  // TODO: calculate apogee based on sensor data
  // Gather lots of additional data at this phase, determining apogee is important

  // ACT
  // if(launched)
  // {
  //   set_flight_stage(DESCENT);
  // }

  // STORE
  write_framebuffer(fb);
}

void handle_DESCENT(Frame* frame, FrameBuffer* fb)
{
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  int32_t current_pressure = get_framebuffer_median(&fb, BUFFER_SIZE, MS5611_PRESSURE);
  // TODO: calculate landing based on sensor data

  // ACT
  // if(landed)
  // {
  //   set_flight_stage(LANDED);
  // }

  // STORE
  write_framebuffer(fb);
}

void handle_LANDING(Frame* frame, FrameBuffer* fb)
{
  read_sensors(&_M5611_data, &_ADXL375_data, &_LSM6DS3_data);

  build_frame(frame, _M5611_data, _ADXL375_data, _LSM6DS3_data, _BME280_data, _GNSS_data);
  update_frame_buffer(frame, fb);

  // ACT
  // Beep? Signal? radio? idk

  // STORE
  write_framebuffer(fb);
}

void run_flight() {

  // Setup FrameBuffer (contains last 50 frames of data)
  Frame frame;        // initialise the Frame that keeps updating
  uint8_t dataArray[128];  // dummy array to store the frame data
  _memset(dataArray, 0,
          sizeof(dataArray));  // set the necessary memory and set values to 0
  frame = unzip(&dataArray);   // convert from normal array into Frame
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
        //STM32_beep_buzzer(200, 200, 1);
        break;
    }
  }
}