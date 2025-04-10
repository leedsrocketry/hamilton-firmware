/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Your Name
  Created on: 29 Mar 2025
  Description: Description of the file
*/

#include "sensors.h"

double Lb = -0.0065;   // temperature lapse rate (K/m)
double hb = 0;         // reference height (sea level)
double R = 8.3144598;  // universal gas constant
double g = 9.80665;    // gravitational acceleration
double M = 0.0289644;  // molar mass of Earth's air
double Pb = 101325;    // reference pressure at sea level (in Pa)

void build_frame(Frame *frame, M5611_data _M5611_data, ADXL375_data _ADXL375_data, LSM6DS3_data _LSM6DS3_data,
                 BME280_data _BME280_data, GNSS_Data _GNSS_data) {
  // Add time stamp
  // uint32_t time = get_time_us();
  // frame->date.minute = (uint8_t)(time / (1000000 * 60)) % 60;  // minutes
  // frame->date.second = (uint8_t)(time / 1000000) % 60;         // seconds
  // frame->date.millisecond = (uint16_t)(time / 1000) % 1000;    // milli
  // seconds frame->date.microsecond = (uint16_t)time % 1000;             //
  // Mirco seconds

  // Add data to the frame
  frame->changeFlag = get_flight_stage();
  frame->accel = _ADXL375_data;
  frame->imu = _LSM6DS3_data;
  frame->barometer = _M5611_data;
  frame->GNSS = _GNSS_data;
  frame->bme = _BME280_data;

  // frame->time = get_time_us();
}

// void read_sensors(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
//                   LSM6DS3_data* _LSM6DS3_data) {
//   MS5611_get_data(_M5611_data);
//   ADXL375_get_data(_ADXL375_data, true);
//   LSM6DS3_gyro_read(SPI1, _LSM6DS3_data);
//   LSM6DS3_acc_read(SPI1, _LSM6DS3_data);

//   // Run calculations

//   //   LOG("Barometer: %ld, Temp: %ld, Accel: %d, %d, %d, Gyro: %ld, %ld,
//   //   %ld\r\n",
//   //       _M5611_data->pressure, _M5611_data->temp, _LSM6DS3_data->x_accel,
//   //       _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel,
//   //       _LSM6DS3_data->x_rate, _LSM6DS3_data->y_rate,
//   _LSM6DS3_data->z_rate);
// }

void read_sensors(Frame *frame, uint32_t dt) {
  (void)dt;  // Unused for now :(
  M5611_data _M5611_data;
  ADXL375_data _ADXL375_data;
  LSM6DS3_data _LSM6DS3_data;

  MS5611_get_data(&_M5611_data);
  ADXL375_get_data(&_ADXL375_data, true);
  LSM6DS3_gyro_read(SPI1, &_LSM6DS3_data);
  LSM6DS3_acc_read(SPI1, &_LSM6DS3_data);
  _LSM6DS3_data.x_rate += _LSM6DS3_data.x_offset;
  _LSM6DS3_data.y_rate += _LSM6DS3_data.y_offset;
  _LSM6DS3_data.z_rate += _LSM6DS3_data.z_offset;

  _LSM6DS3_data.x_rate = (int32_t)(_LSM6DS3_data.x_rate);
  _LSM6DS3_data.y_rate = (int32_t)(_LSM6DS3_data.y_rate);
  _LSM6DS3_data.z_rate = (int32_t)(_LSM6DS3_data.z_rate);

  // uint32_t time = get_time_us();
  // frame->date.minute = (time / (1000000 * 60)) % 60;  // minuts
  // frame->date.second = (time / 1000000) % 60;         // seconds
  // frame->date.millisecond = (time / 1000) % 1000;     // milli seconds
  // frame->date.microsecond = time % 1000;              // Mirco seconds
  // frame->changeFlag = flightStage;

  frame->accel = _ADXL375_data;
  frame->imu = _LSM6DS3_data;
  frame->barometer = _M5611_data;
}

void format_sensor_data(M5611_data *_M5611_data, ADXL375_data *_ADXL375_data, LSM6DS3_data *_LSM6DS3_data, char *buffer,
                        size_t buffer_size) {
  snprintf(buffer, buffer_size,
           "Barometer: %ld, Temp: %ld, ADXL Accel: %d, %d, %d, LSM Accel: %d, "
           "%d, %d, Gyro: %ld, %ld, %ld\r\n",
           _M5611_data->pressure, _M5611_data->temp, _ADXL375_data->x, _ADXL375_data->y, _ADXL375_data->z,
           _LSM6DS3_data->x_accel, _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel, _LSM6DS3_data->x_rate,
           _LSM6DS3_data->y_rate, _LSM6DS3_data->z_rate);
}

void print_sensor_line(Frame frame) {
  printf("[BAR: T=%5" PRId32 " P=%5" PRId32 "] [ACCEL: X=%5" PRId16 " Y=%5" PRId16 " Z=%5" PRId16 "] [IMU: X=%5" PRId32
         " Y=%5" PRId32 " Z=%5" PRId32 "]\r\n",
         frame.barometer.temp, frame.barometer.pressure, frame.accel.x, frame.accel.y, frame.accel.z, frame.imu.x_rate,
         frame.imu.y_rate, frame.imu.z_rate);
}

double barometric_equation(double pressure, double temp) {
  double h;             // height to be calculated
  double P = pressure;  // pressure at the point of interest (in Pa)
  double Tb = temp;     // temperature at the point of interest (in K)

  h = hb + (Tb / Lb) * (pow(P / Pb, (-R * Lb) / (g * M)) - 1);

  return h/10;
}

// void test_sensors()
// {
//   M5611_data TEST_M5611_data;
//   ADXL375_data TEST_ADXL375_data;
//   LSM6DS3_data TEST_LSM6DS3_data;

//   for (;;)
//   {
//     read_sensors(&TEST_M5611_data, &TEST_ADXL375_data, &TEST_LSM6DS3_data);
//     printf(
//         "DEBUG - Barometer: %ld, Temp: %ld, ADXL Accel: %d, %d, %d, LSM
//         Accel: "
//         "%d, %d, %d, Gyro: %ld, %ld, %ld\r\n",
//         TEST_M5611_data.pressure, TEST_M5611_data.temp, TEST_ADXL375_data.x,
//         TEST_ADXL375_data.y, TEST_ADXL375_data.z, TEST_LSM6DS3_data.x_accel,
//         TEST_LSM6DS3_data.y_accel, TEST_LSM6DS3_data.z_accel,
//         TEST_LSM6DS3_data.x_rate, TEST_LSM6DS3_data.y_rate,
//         TEST_LSM6DS3_data.z_rate);
//   }
// }

void calibrate_ADXL375() {
  ADXL375_data TEST_ADXL375_data;

  int32_t runs = 10000;

  int32_t cum_x = 0;
  int32_t cum_y = 0;
  int32_t cum_z = 0;

  for (int32_t i = 0; i < runs; i++) {
    ADXL375_get_data(&TEST_ADXL375_data, false);
    cum_x += TEST_ADXL375_data.x;
    cum_y += TEST_ADXL375_data.y;
    cum_z += TEST_ADXL375_data.z;
  }

  int32_t avg_x = cum_x / runs;
  int32_t avg_y = cum_y / runs;
  int32_t avg_z = cum_z / runs;

  printf("AVERAGE DATA FOR CALIBRATION: ");
  printf("%ld, %ld, %ld \r\n", avg_x, avg_y, avg_z);
  return;
}
