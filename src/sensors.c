#include "sensors.h"

void build_frame(Frame* _frameArray, M5611_data _M5611_data,
                     ADXL375_data _ADXL375_data, LSM6DS3_data _LSM6DS3_data,
                     BME280_data _BME280_data, GNSS_Data _GNSS_data) {
  // Add time stamp
  uint32_t time = get_time_us();
  _frameArray->date.minute = (uint8_t)(time / (1000000 * 60)) % 60;  // minutes
  _frameArray->date.second = (uint8_t)(time / 1000000) % 60;         // seconds
  _frameArray->date.millisecond =
      (uint16_t)(time / 1000) % 1000;                     // milli seconds
  _frameArray->date.microsecond = (uint16_t)time % 1000;  // Mirco seconds

  // Add data to the frame
  _frameArray->changeFlag = get_flight_stage();
  _frameArray->accel = _ADXL375_data;
  _frameArray->imu = _LSM6DS3_data;
  _frameArray->barometer = _M5611_data;
  _frameArray->GNSS = _GNSS_data;
  _frameArray->bme = _BME280_data;
}

void read_sensors(M5611_data* _M5611_data, ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data) {
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  LSM6DS3_gyro_read(SPI1, _LSM6DS3_data);
  LSM6DS3_acc_read(SPI1, _LSM6DS3_data);

//   LOG("Barometer: %ld, Temp: %ld, Accel: %d, %d, %d, Gyro: %ld, %ld, %ld\r\n",
//       _M5611_data->pressure, _M5611_data->temp, _LSM6DS3_data->x_accel,
//       _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel, _LSM6DS3_data->x_rate,
//       _LSM6DS3_data->y_rate, _LSM6DS3_data->z_rate);
}

double barometric_equation(double pressure, double temp)
{
    double P = pressure; // pressure at the point of interest (in Pa)
    double Pb = 101325; // reference pressure at sea level (in Pa)
    double Tb = temp; // temperature at the point of interest (in K)
    double Lb = -0.0065; // temperature lapse rate (K/m)
    double h; // height to be calculated
    double hb = 0; // reference height (sea level)
    double R = 8.3144598; // universal gas constant
    double g = 9.80665; // gravitational acceleration
    double M = 0.0289644; // molar mass of Earth's air

    h = hb + (Tb / Lb) * (pow(P / Pb, (-R * Lb) / (g * M)) - 1);

    return h;
}