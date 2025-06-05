#include "NAND_flash_driver.h"

// Global variable definitions
uint32_t frameAddressPointer = 0;
uint8_t globalPinMode = GPIO_MODE_OUTPUT;

bool get_bit_arr(uint8_t *arr, int pos) {
  return (bool)(arr[pos/8] & (1 << (7-(pos%8))));
}

bool get_bit(uint8_t byte, int pos) {
  return (bool)(byte & (1 << (7-(pos%8))));
}

void print_byte(uint8_t myByte) {
  LOG("0b");
  for (int i = 7; i >= 0; i--) {
    LOG("%i", (myByte >> i) & 0b1);
  }
  LOG("\r\n");
}

void print_frame(uint8_t dataArray[]) {
  LOG("u");
  for (int i = 0; i < 128; i++) {
    LOG("%X,", dataArray[i]); // %02X adds a paddings of 0
  }
  LOG("\r\n");
}

void print_frameHex(uint8_t dataArray[]) {
  for (int i = 0; i < 128; i++) {
    if (dataArray[i] < 16) {
      LOG("0");
    }
    LOG("%i", dataArray[i]);
    LOG(", ");
  }
  LOG("\r\n");
}

void _memset(uint8_t *arr, uint8_t val, int num){
  for (int i = 0; i < num; i++) {
    arr[i] = val;
  }
}

void wait_for_ready_flag() {
  int count = 1000*10; // Try for 1 second before giving error
  while (gpio_read(RB) == LOW && count > 0) {
    delay_microseconds(1);
    count--;
  }
  if (count < 1) {
    //LOG("waitForReadyFlag: TIMEOUT\r\n");
  } 
}

void set_pin_modes() {
  gpio_set_mode(DATA0, globalPinMode);
  gpio_set_mode(DATA1, globalPinMode);
  gpio_set_mode(DATA2, globalPinMode);
  gpio_set_mode(DATA3, globalPinMode);
  gpio_set_mode(DATA4, globalPinMode);
  gpio_set_mode(DATA5, globalPinMode);
  gpio_set_mode(DATA6, globalPinMode);
  gpio_set_mode(DATA7, globalPinMode);
  delay_microseconds(DELAY_PINMODE);
}

void set_control_pins(uint8_t controlRegister) {  // CE# CLE ALE WE# RE# WP#
  gpio_write(CE, get_bit(controlRegister, 0));
  gpio_write(CLE, get_bit(controlRegister, 1));
  gpio_write(ALE, get_bit(controlRegister, 2));
  gpio_write(WE, get_bit(controlRegister, 3));
  gpio_write(RE, get_bit(controlRegister, 4));
  gpio_write(WP, get_bit(controlRegister, 5));
}

void set_data_pins(uint8_t Byte) {
  if (globalPinMode == GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_OUTPUT;
    set_pin_modes();
  }

  gpio_write(DATA0, get_bit(Byte, 7));
  gpio_write(DATA1, get_bit(Byte, 6));
  gpio_write(DATA2, get_bit(Byte, 5));
  gpio_write(DATA3, get_bit(Byte, 4));
  gpio_write(DATA4, get_bit(Byte, 3));
  gpio_write(DATA5, get_bit(Byte, 2));
  gpio_write(DATA6, get_bit(Byte, 1));
  gpio_write(DATA7, get_bit(Byte, 0));
}

void send_byte_to_flash(uint8_t cmd, uint8_t mode) {
  //delay_ms(DELAY); // include if needed
  set_control_pins(mode);
  set_data_pins(cmd);
  //delay_ms(DELAY);
  set_control_pins(mode | WE_HIGH); // lanch what is in the data bus in the memory
  //delay_ms(DELAY);
}

uint8_t receive_byte_from_flash() {  
  delay_microseconds(DELAY);
  set_control_pins(DATA_OUTPUT);
  delay_microseconds(DELAY);
  set_control_pins(DATA_OUTPUT & (~RE_HIGH));  // setting RE LOW
  delay_microseconds(DELAY);

  if (globalPinMode != GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_INPUT;
    set_pin_modes();
  }

  uint8_t data = (gpio_read(DATA7) << 7)
               | (gpio_read(DATA6) << 6)
               | (gpio_read(DATA5) << 5)
               | (gpio_read(DATA4) << 4)
               | (gpio_read(DATA3) << 3)
               | (gpio_read(DATA2) << 2)
               | (gpio_read(DATA1) << 1)
               | (gpio_read(DATA0) << 0);
  return data;
}

void send_addr_to_flash(uint32_t frameAddr, uint8_t byteAddr) {
  Address addr = {(frameAddr >> 11) & 0b0000111111111111,                      // block
                  (frameAddr >> 5) & 0b00111111,                                // page
                  ((frameAddr & 0b00011111) << 7) | (byteAddr & 0b01111111)}; // column 

  send_byte_to_flash((uint8_t)(addr.column & 0b0000000011111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.column & 0b0001111100000000) >> 8), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)(((addr.block & 0b0000000000000011) << 6)) | (addr.page & 0b00111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}

void send_block_addr_to_flash(uint32_t blockAddr) {
  send_byte_to_flash((uint8_t)(((blockAddr & 0b0000000000000011) << 6) | (0b00000000 & 0b00111111)), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}

uint8_t read_flash_status() {
  wait_for_ready_flag();
  send_byte_to_flash(0x70, COMMAND_INPUT);
  return receive_byte_from_flash();
}

uint64_t read_flash_ID() {
  uint64_t id = 0;

  wait_for_ready_flag();
  send_byte_to_flash(0x90, COMMAND_INPUT);
  send_byte_to_flash(0x00, ADDRESS_INPUT);

  for (int i = 0; i < 5; i++) {
    LOG("ID ");
    LOG("%i", i);
    LOG(": ");
    uint8_t byte = receive_byte_from_flash();
    id |= byte << (4-i);
    print_byte(byte);
  }
  return id;
}

void print_csv_header() {
  LOG("Date,");
  LOG("Time,");
  LOG("ChangeFlag,");
  LOG("ACC X,ACC Y,ACC Z,");
  LOG("IMU X_RATE,IMU Y_RATE,IMU Z_RATE,\
          IMU X_OFFSET,IMU Y_OFFSET,IMU Z_OFFSET,IMU ACC_X,IMU ACC_Y,IMU ACC_Z,");
  LOG("MS5611 Temperature,MS5611 Pressure,");
  LOG("GNSS Lat,GNSS Long,GNSS Alt,GNSS velocity,");
  LOG("BME280 Pressure,BME280 Temperature,BME280 Humidity,");
  LOG("\r\n");
}
/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_frame_csv(Frame frameFormat) {
  LOG("%i,", frameFormat.date.year); 
  LOG("%i:%i:%i:%i,", frameFormat.date.minute,
                         frameFormat.date.second,
                         frameFormat.date.millisecond,
                         frameFormat.date.microsecond);

  LOG("%i,", frameFormat.changeFlag);

  LOG("%i,%i,%i,", frameFormat.accel.x, frameFormat.accel.y, frameFormat.accel.z);

  LOG("%li,%li,%li,%li,%li,%li,%i,%i,%i,", frameFormat.imu.x_rate, 
                                        frameFormat.imu.y_rate, 
                                        frameFormat.imu.z_rate, 
                                        frameFormat.imu.x_offset, 
                                        frameFormat.imu.y_offset, 
                                        frameFormat.imu.z_offset, 
                                        frameFormat.imu.x_accel, 
                                        frameFormat.imu.y_accel, 
                                        frameFormat.imu.z_accel);

  LOG("%li,%li,", frameFormat.barometer.temp, frameFormat.barometer.pressure);
  LOG("%i,%i,%i,%i,", frameFormat.GNSS.latitude, frameFormat.GNSS.longitude, frameFormat.GNSS.altitude, frameFormat.GNSS.velocity);
  LOG("%li,%i,%li,", frameFormat.bme.pressure, frameFormat.bme.temperature, frameFormat.bme.humidity);
  LOG("\r\n");
}


void print_frame_array(Frame frameFormat) {
  LOG("Date: %i, %i:%i:%i:%i:\r\n", frameFormat.date.year, frameFormat.date.minute,
                                               frameFormat.date.second, frameFormat.date.millisecond, 
                                               frameFormat.date.microsecond );

  LOG("ChangeFlag: %u\r\n", frameFormat.changeFlag);

  LOG("Accel HG: \tX: %i,\tY: %i,\tZ: %i\t\r\n", frameFormat.accel.x, frameFormat.accel.y, frameFormat.accel.z);

  LOG("IMU: \tX Rate: %ld,\tY Rate: %ld,\tZ Rate: %ld,\tX Offset: %ld,\tY Offset: %ld, \
        \tZ Offset: %ld,\tX Accel: %d,\tY Accel: %d,\tZ Accel: %d,\r\n", frameFormat.imu.x_rate, 
                                                                        frameFormat.imu.y_rate, 
                                                                        frameFormat.imu.z_rate, 
                                                                        frameFormat.imu.x_offset, 
                                                                        frameFormat.imu.y_offset, 
                                                                        frameFormat.imu.z_offset, 
                                                                        frameFormat.imu.x_accel, 
                                                                        frameFormat.imu.y_accel, 
                                                                        frameFormat.imu.z_accel);

  LOG("Barometer: \ttemp: %ld, \tpressure: %ld\r\n", frameFormat.barometer.temp, frameFormat.barometer.pressure);
  LOG("GNSS: \tLat: %d,\tLong: %d,\tAlt: %d,\tVel: %d\r\n", frameFormat.GNSS.latitude, 
                                                               frameFormat.GNSS.longitude, 
                                                               frameFormat.GNSS.altitude, 
                                                               frameFormat.GNSS.velocity);
                                                               
  LOG("BME280: \tPressure: %ld,\tTemperature: %d,\tHumidity: %ld\r\n", frameFormat.bme.pressure, 
                                                                      frameFormat.bme.temperature, 
                                                                      frameFormat.bme.humidity);
}


void zip(Frame unzippedData, uint8_t *zippedData) {
  int i = -1;

  // Date and time
  zippedData[i++] = unzippedData.date.year;
  zippedData[i++] = unzippedData.date.minute;
  zippedData[i++] = unzippedData.date.second;
  zippedData[i++] = (uint8_t)((unzippedData.date.millisecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.date.millisecond & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.date.microsecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.date.microsecond & 0xFF);

  // Change flag for frame optimisation
  zippedData[i++] = (uint8_t)(unzippedData.changeFlag & 0xFF);

  // ADXL375
  zippedData[i++] = (uint8_t)((unzippedData.accel.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accel.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accel.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accel.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accel.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accel.z & 0xFF);

  // LSM6DS3
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_rate >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_rate >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_rate >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.x_rate & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_rate >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_rate >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_rate >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.y_rate & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_rate >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_rate >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_rate >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.z_rate & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.imu.x_offset >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_offset >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_offset >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.x_offset & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_offset >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_offset >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_offset >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.y_offset & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_offset >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_offset >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_offset >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.z_offset & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.imu.x_accel >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_accel >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.x_accel >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.x_accel & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_accel >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_accel >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.y_accel >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.y_accel & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_accel >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_accel >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.imu.z_accel >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.imu.z_accel & 0xFF);

  // MS5611
  zippedData[i++] = (uint8_t)((unzippedData.barometer.temp >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer.temp >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer.temp >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.barometer.temp & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer.pressure >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer.pressure >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer.pressure >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.barometer.pressure & 0xFF);

  // GNSS
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.latitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.latitude & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.longitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.longitude & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.altitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.altitude & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.velocity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.velocity & 0xFF);

  // BME280
  zippedData[i++] = (uint8_t)((unzippedData.bme.pressure >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.pressure >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.pressure >> 8) & 0xFF);
  zippedData[i++] = (uint8_t) (unzippedData.bme.pressure & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.temperature >> 8) & 0xFF);
  zippedData[i++] = (uint8_t) (unzippedData.bme.temperature & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t) (unzippedData.bme.humidity & 0xFF);

  zippedData[118] = unzippedData.hammingCode[0];
  zippedData[119] = unzippedData.hammingCode[1];
  zippedData[120] = unzippedData.hammingCode[2];
  zippedData[121] = unzippedData.hammingCode[3];
  zippedData[122] = unzippedData.hammingCode[4];
  zippedData[123] = unzippedData.hammingCode[5];
  zippedData[124] = unzippedData.hammingCode[6];
  zippedData[125] = unzippedData.hammingCode[7];

  zippedData[126] = (uint8_t)((unzippedData.CRC_Check >> 8) & 0xFF);
  zippedData[127] = (uint8_t)(unzippedData.CRC_Check & 0xFF);
}

Frame unzip(uint8_t *zippedData) {
  Frame _unzippedData;
  _unzippedData.changeFlag = 1;
  int i = -1;

  // Resolve errors to do with uninitialised data, but this is obviously not correct.
  _unzippedData.GNSS.altitude = 0;
  _unzippedData.GNSS.velocity = 0;
  _unzippedData.bme.temperature = 0;

  // Date and time
  _unzippedData.date.year = zippedData[i++];
  _unzippedData.date.minute = zippedData[i++];
  _unzippedData.date.second = zippedData[i++];
  _unzippedData.date.millisecond = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.date.millisecond |= zippedData[i++];
  _unzippedData.date.microsecond = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.date.microsecond |= zippedData[i++];

  // Change flag for frame optimisation
  _unzippedData.changeFlag |= zippedData[i++];

  // ADXL375
  _unzippedData.accel.x = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.accel.x |= zippedData[i++];
  _unzippedData.accel.y = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.accel.y |= zippedData[i++];
  _unzippedData.accel.z = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.accel.z |= zippedData[i++];

  // LSM6DS3
  _unzippedData.imu.x_rate = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.x_rate |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.x_rate |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.x_rate |= zippedData[i++];
  _unzippedData.imu.y_rate = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.y_rate |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.y_rate |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.y_rate |= zippedData[i++];
  _unzippedData.imu.z_rate = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.z_rate |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.z_rate |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.z_rate |= zippedData[i++];

  _unzippedData.imu.x_offset = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.x_offset |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.x_offset |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.x_offset |= zippedData[i++];
  _unzippedData.imu.y_offset = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.y_offset |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.y_offset |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.y_offset |= zippedData[i++];
  _unzippedData.imu.z_offset = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.z_offset |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.z_offset |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.z_offset |= zippedData[i++];

  _unzippedData.imu.x_accel = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.x_accel |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.x_accel |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.x_accel |= zippedData[i++];
  _unzippedData.imu.y_accel = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.y_accel |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.y_accel |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.y_accel |= zippedData[i++];
  _unzippedData.imu.z_accel = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.imu.z_accel |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.imu.z_accel |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.imu.z_accel |= zippedData[i++];

  // MS5611
  _unzippedData.barometer.temp = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.barometer.temp |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.barometer.temp |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.barometer.temp |= zippedData[i++];
  _unzippedData.barometer.pressure = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.barometer.pressure |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.barometer.pressure |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.barometer.pressure |= zippedData[i++];

  // GNSS
  _unzippedData.GNSS.latitude = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.GNSS.latitude |= zippedData[i++];
  _unzippedData.GNSS.longitude = (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.GNSS.longitude |= zippedData[i++];
  _unzippedData.GNSS.altitude |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.GNSS.altitude |= zippedData[i++];
  _unzippedData.GNSS.velocity |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.GNSS.velocity |= zippedData[i++];

  // BME280
  _unzippedData.bme.pressure = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.bme.pressure |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.bme.pressure |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.bme.pressure |= zippedData[i++];
  _unzippedData.bme.temperature |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.bme.temperature |= zippedData[i++];
  _unzippedData.bme.humidity = (zippedData[i++] << 24) & (0xFF << 24);
  _unzippedData.bme.humidity |= (zippedData[i++] << 16) & (0xFF << 16);
  _unzippedData.bme.humidity |= (zippedData[i++] << 8) & (0xFF << 8);
  _unzippedData.bme.humidity |= zippedData[i++];

  _unzippedData.hammingCode[0] = zippedData[118];
  _unzippedData.hammingCode[1] = zippedData[119];
  _unzippedData.hammingCode[2] = zippedData[120];
  _unzippedData.hammingCode[3] = zippedData[121];
  _unzippedData.hammingCode[4] = zippedData[122];
  _unzippedData.hammingCode[5] = zippedData[123];
  _unzippedData.hammingCode[6] = zippedData[124];
  _unzippedData.hammingCode[7] = zippedData[125];

  _unzippedData.CRC_Check = (zippedData[126] << 8) & (0xFF << 8);
  _unzippedData.CRC_Check |= zippedData[127]; 

  return _unzippedData;
}

void write_protection() {
  wait_for_ready_flag();
  set_control_pins(WRITE_PROTECT);  // Write Protection
}

void read_frame(uint32_t frameAddr, uint8_t *readFrameBytes, uint8_t _length) {
  wait_for_ready_flag();
  send_byte_to_flash(0x00, COMMAND_INPUT);
  send_addr_to_flash(frameAddr, 0);
  send_byte_to_flash(0x30, COMMAND_INPUT);
  wait_for_ready_flag();

  for (int byteAddr = 0; byteAddr < _length; byteAddr++) {
    readFrameBytes[byteAddr] = receive_byte_from_flash();  // read data byte
  }
}

void write_frame(uint32_t frameAddr, uint8_t *bytes) {
  wait_for_ready_flag();
  send_byte_to_flash(0x80, COMMAND_INPUT);
  send_addr_to_flash(frameAddr, 0);  // Address Input
  delay_microseconds(10);
  set_control_pins(DATA_INPUT);
  for (int byteAddr = 0; byteAddr < 128; byteAddr++) {
    gpio_write(WE, 0);
    set_data_pins(bytes[byteAddr]);
    gpio_write(WE, 1);
  }
  send_byte_to_flash(0x10, COMMAND_INPUT);
}

void erase_block(uint32_t blockAddr) {
  wait_for_ready_flag();
  send_byte_to_flash(0x60, COMMAND_INPUT); //0x60 is erase command.
  send_block_addr_to_flash(blockAddr);
  send_byte_to_flash(0xD0, COMMAND_INPUT); 
  wait_for_ready_flag();  // Blocking Function
}

void erase_all() {
  LOG("WARNING: ERASING ALL DATA (UNPLUG NAND FLASH TO ABORT)\r\n");

  // you have 10 seconds to unplug the nand flash
  for (int countDown = 10; countDown > 0; countDown--) {
    LOG("ERASING DATA IN: ");
    LOG("%i", countDown);
    LOG(" Seconds\r\n");
    delay_ms(1000);
  }

  for (uint32_t block = 0; block < 64*4096; block++) {
    erase_block(block);
    if (block%5000 == 0) {
      LOG("ERASING [");
      for (int i = 0; i < 50; i++) {
        if(i < block/(64*4096*0.01*2)){
          LOG("#");
        } else {
          LOG(" ");
        }
      }
      LOG("] - ");
      int percentage = (int)(block/(64*4096*0.01));
      LOG("%d", percentage);
      LOG("%%\r\n");
    }
  }
  LOG("ERASING COMPLETE \r\n");
}

uint16_t max(uint16_t x1, uint16_t x2) {
  return (x1 > x2) ? x1 : x2;
}

uint16_t min(uint16_t x1, uint16_t x2) {
  return (x1 < x2) ? x1 : x2;
}

uint16_t diff(uint16_t x1, uint16_t x2) {
  return (uint16_t)abs((int)((int)x1 - (int)x2));
}

uint32_t get_next_available_frame_addr() {
  uint16_t prevPointer = 4096;
  uint16_t pointer = prevPointer / 2;
  uint8_t _check = 0; 
  
  for (int i = 0; i < 11; i++) {
    read_frame(max(pointer - 1, 0) * 64 * 32, &_check, 1);  // Dosn't need to be the whole frame -------
    uint16_t difference = diff(pointer, prevPointer);
    prevPointer = pointer;

    // 0xFF is empty space
    // tree search
    if (_check == 0xFF) {  
      pointer -= difference / 2;
    } else {
      pointer += difference / 2;
    }
  }
  
  read_frame(max(pointer - 1, 0) * 64 * 32, &_check, 1);
  if (_check != 0xFF) {
    pointer += 1;
  }

  return max(pointer - 1, 0) * 64 * 32;  // pointer * no of pages in block * no of frames in a page
}

void init_flash() {
  gpio_set_mode(DATA0, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA1, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA2, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA3, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA4, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA5, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA6, GPIO_MODE_OUTPUT);
  gpio_set_mode(DATA7, GPIO_MODE_OUTPUT);

  gpio_set_mode(ALE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CLE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(RE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WP,  GPIO_MODE_OUTPUT);

  gpio_set_mode(RB,  GPIO_MODE_INPUT);
  
  frameAddressPointer = get_next_available_frame_addr();
  LOG("FRAME ADDRESS POINTER %li.\r\n", frameAddressPointer);

  LOG("%lld\r\n", read_flash_ID());

  if (read_flash_ID() != 0){
    LOG("Flash Working Correctly\r\n");
  }
}

uint16_t calculate_CRC(uint8_t* data, uint8_t length) {
  const uint16_t CRC_POLY = 0x1021;
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY : crc << 1;
    }
  }
  return crc;
}

void hash(uint8_t *_input, uint8_t *_output) {
  _memset(_output, 0, 120);

  for (int i = 0; i < 8*120; i++) {
    int j = ((i%120)*8) + (i/120);
    _output[i/8] |= get_bit_arr(_input, j) << (7-(i%8));
  }
}

bool is_power_of_two(int x) {
  return (x != 0) && ((x & (x - 1)) == 0);
}

void calculate_parity_bits(uint8_t *_input, uint8_t *_output) {
  uint8_t hashedData[120];
  _memset(hashedData, 0, 120);

  uint8_t condition[120];
  _memset(condition, 0, 120);
  for (int i = 0; i < 118; i++) {
    condition[i] = _input[i];
  }
  
  hash(condition, hashedData);

  // Initialise variables
  uint8_t _word[15];
  uint8_t parities = 0;
  uint8_t parity = 0;
  int k = 0;

  for (int _set = 0; _set < 8; _set++) {
    for (int i = 0; i < 15; i ++) {
      _word[i] = hashedData[(_set * 15) + i];
    }
    
    // Initialize parity bits to 0
    parities = 0;
    // Calculate parity bits
    for (int i = 0; i < 8; i++) {
      // Calculate bit position of this parity bit
      int bit_pos = 1 << i;
      
      // Calculate parity for this bit position
      parity = 0;
      k = 0;
      for (int j = 0; j < 128; j++) { // j from 0 - 128
        if (j + 1 != 1 && is_power_of_two(j + 1) == 0) {
          if (bit_pos & (j + 1)) {
            parity ^= (_word[k / 8] >> (k % 8)) & 1;
          }
          k++;
        }
      }
      parities |= parity << (i%8);
    }
    _output[_set] = parities;
  }
}

void encode_parity(Frame dataFrame, uint8_t *bytes) {
  zip(dataFrame, bytes);
  /*
  uint8_t parities[8];
  calculate_parity_bits(bytes, parities);
  for (int i = 0; i < 8; i++) {
    bytes[118+i] = parities[i];
  }
  */
  uint16_t CRC_Check = calculate_CRC(bytes, 126);
  bytes[126] = (uint8_t)((CRC_Check >> 8) & 0xFF);
  bytes[127] = (uint8_t)(CRC_Check & 0xFF);
}

void print_capacity_info() {
  uint32_t lastFrameUsed = get_next_available_frame_addr();
  LOG("Used: ");
  uint32_t usedInMB = (lastFrameUsed*128)/1000000;
  LOG("%i", (int)usedInMB);
  LOG(" MB (");
  LOG("%f", lastFrameUsed/(4096*64*32*0.01));
  LOG("%%) | ");
  LOG("Estimated Time Remaining (hh:mm:ss): ");
  uint32_t hours = ((4096*64*32) - lastFrameUsed)/(1000*60*60)%24;
  uint32_t minutes = ((4096*64*32) - lastFrameUsed)/(1000*60)%60;
  uint32_t seconds = (((4096*64*32) - lastFrameUsed)/(1000))%60;
  if (hours<10) LOG("0");
  LOG("%i", (int)hours);
  LOG(":");
  if (minutes<10) LOG("0");
  LOG("%i", (int)minutes);
  LOG(":");
  if (seconds<10) LOG("0");
  LOG("%i", (int)seconds);
  LOG("\r\n");
}

int log_frame(Frame _input) {
  // Frame to array of bytes; 8388607 is 8Gb end
  if (frameAddressPointer <= 8388607) {
    uint8_t encoded[128];
    _memset(encoded, 0, 128);
    encode_parity(_input, encoded);
    write_frame(frameAddressPointer, encoded);
    frameAddressPointer++;
  } else {
    LOG("Overflow Error\r\n");  // ERROR
    return STORAGE_FULL_ERROR;
  }
  // If the pointer is near the capacity of the storage
  // (95% full, 6.5 minutes left of recording)
  if (frameAddressPointer >= 8000000) {   
    return STORAGE_FULL_WARNING;
  }
  return SUCCESS;  // Successfully written
}

Frame recall_frame(uint32_t frameAddr) {
  uint8_t encoded[128];
  _memset(encoded, 0, 128);
  Frame _output;
  read_frame(frameAddr, encoded, 128);
  _output = unzip(encoded);  // Don't bother decoding parity bits
  return _output;
}

void read_all_raw() {
  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);
  
  bool skipBlank = true;

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      LOG("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;

    }else{
      //read as a uint8_t array
      read_frame(i, array, 128);
      print_frame(array);
    } 
  }
}

void read_all_frame() {
  Frame _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);
  
  bool skipBlank = true;

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      LOG("End of data in block.\r\n");
      i += 2048; // move to the next block
      i = i - (i%2048) - 1;

    } else {
      // rpint out as a frame
      _output = recall_frame(i);
        LOG("FN:%li\r\n", i);
      print_frame_array(_output);
    } 
  }
}

void read_all_csv() {
  Frame _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);
  
  bool skipBlank = true;
  print_csv_header();

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      LOG("End of data in block.\r\n");
      i += 2048; // move to the next block
      i = i - (i%2048) - 1;

    }else{
      // rpint out as a frame
      _output = recall_frame(i);
      // LOG("FN:%i\r\n", i);
      print_frame_csv(_output);
    } 
  }
}

void read_all() {
  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;
  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int data_empty = 0;

  uint8_t array[128];
  _memset(array, 0, 128);

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    read_frame(i, &_check, 1);
    if (_check == 0xFF && false){
      LOG("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;
    } else {
      // Read as a uint8_t array
      read_frame(i, array, 128);
      print_frame(array);
    }
  }
  
  LOG("----------------------------------------------\r\n");
  print_capacity_info();
  LOG("data_empty: ");
  LOG(data_empty + "\r\n");
  LOG("data_intact: ");
  LOG(data_intact + "\r\n");
  LOG("data_fixed: ");
  LOG(data_fixed + "\r\n");
  LOG("data_error: ");
  LOG(data_error + "\r\n");
  LOG("Percent Correct Data : ");
  LOG("%i", (data_intact + data_fixed)/(4096*64*(4096/128)));
  LOG("%%\r\n");
}

void NAND_flash_erase() {
  watchdog_pat();
  erase_all();
  while(1);
}

void NAND_flash_read() {
  LOG("==================== Reading NAND FLASH ====================\r\n");
  read_all_csv();
  print_capacity_info();
}
