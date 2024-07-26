#include "sensors.h"

void get_frame_array(FrameArray* _frameArray, 
                    M5611_data _M5611_data, 
                    ADXL375_data _ADXL375_data, 
                    LSM6DS3_data _LSM6DS3_data,
                    BME280_data _BME280_data,
                    GNSS_Data _GNSS_data) {
  // Add time stamp
  uint32_t time = get_time_us();
  _frameArray->date.minute = (uint8_t)(time/(1000000*60))%60; //minuts
  _frameArray->date.second = (uint8_t)(time/1000000)%60; //seconds
  _frameArray->date.millisecond = (uint16_t)(time/1000)%1000; //milli seconds
  _frameArray->date.microsecond = (uint16_t)time%1000; //Mirco seconds
  
  // Add data to the frame
  _frameArray->changeFlag = get_flight_stage();
  _frameArray->accel = _ADXL375_data;
  _frameArray->imu =_LSM6DS3_data;
  _frameArray->barometer = _M5611_data;
  _frameArray->GNSS = _GNSS_data;
  _frameArray->bme = _BME280_data;
}

void update_sensors(M5611_data* _M5611_data, 
                    ADXL375_data* _ADXL375_data,
                    LSM6DS3_data* _LSM6DS3_data) {
  
  MS5611_get_data(_M5611_data);
  ADXL375_get_data(_ADXL375_data);
  LSM6DS3_gyro_read(SPI1, _LSM6DS3_data);
  LSM6DS3_acc_read(SPI1, _LSM6DS3_data);

  LOG("Barometer: %ld, Temp: %ld, Accel: %d, %d, %d, Gyro: %ld, %ld, %ld\r\n", 
          _M5611_data->pressure, _M5611_data->temp, 
          _LSM6DS3_data->x_accel, _LSM6DS3_data->y_accel, _LSM6DS3_data->z_accel, 
          _LSM6DS3_data->x_rate, _LSM6DS3_data->y_rate, _LSM6DS3_data->z_rate);
}