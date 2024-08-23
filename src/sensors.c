#include "sensors.h"

void build_frame(Frame* frame, M5611_data _M5611_data,
                     ADXL375_data _ADXL375_data, LSM6DS3_data _LSM6DS3_data,
                     BME280_data _BME280_data, GNSS_Data _GNSS_data) {
  // Add time stamp
  uint32_t time = get_time_us();
  frame->date.minute = (uint8_t)(time / (1000000 * 60)) % 60;  // minutes
  frame->date.second = (uint8_t)(time / 1000000) % 60;         // seconds
  frame->date.millisecond =
      (uint16_t)(time / 1000) % 1000;                     // milli seconds
  frame->date.microsecond = (uint16_t)time % 1000;  // Mirco seconds

  // Add data to the frame
  frame->changeFlag = get_flight_stage();
  frame->accel = _ADXL375_data;
  frame->imu = _LSM6DS3_data;
  frame->barometer = _M5611_data;
  frame->GNSS = _GNSS_data;
  frame->bme = _BME280_data;

  frame->time = get_time_us();
}



void read_sensors(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data) {
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  LSM6DS3_gyro_read(SPI1, _LSM6DS3_data);
  LSM6DS3_acc_read(SPI1, _LSM6DS3_data);

  // Run calculations


//   LOG("Barometer: %ld, Temp: %ld, Accel: %d, %d, %d, Gyro: %ld, %ld, %ld\r\n",
//       _M5611_data->pressure, _M5611_data->temp, _LSM6DS3_data->x_accel,
//       _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel, _LSM6DS3_data->x_rate,
//       _LSM6DS3_data->y_rate, _LSM6DS3_data->z_rate);
}

void format_sensor_data(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                        LSM6DS3_data* _LSM6DS3_data, char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, 
             "Barometer: %ld, Temp: %ld, ADXL Accel: %d, %d, %d, LSM Accel: %d, %d, %d, Gyro: %ld, %ld, %ld\r\n",
             _M5611_data->pressure, 
             _M5611_data->temp,
             _ADXL375_data->x, 
             _ADXL375_data->y, 
             _ADXL375_data->z,
             _LSM6DS3_data->x_accel, 
             _LSM6DS3_data->y_accel, 
             _LSM6DS3_data->z_accel,
             _LSM6DS3_data->x_rate, 
             _LSM6DS3_data->y_rate, 
             _LSM6DS3_data->z_rate);

double barometric_equation(double pressure, double temp)
{
    double h; // height to be calculated
    double P = pressure; // pressure at the point of interest (in Pa)
    double Tb = temp; // temperature at the point of interest (in K)

    h = hb + (Tb / Lb) * (pow(P / Pb, (-R * Lb) / (g * M)) - 1);

    return h;
}