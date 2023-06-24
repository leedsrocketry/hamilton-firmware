/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Thomas Groom, Alexnadra Posta
  Created on: 09 March 2023
  Description: NAND Flash driver
*/

#include "mcu.h"

#define NONE 0
#define SUCCESS 1
#define STORAGE_FULL_WARNING 2
#define STORAGE_FULL_ERROR 3

#define DATA_INTACT 1
#define DATA_FIXED 2
#define DATA_CORRUPTED 3
#define EMPTY 4

#define DELAY 1
#define DELAY_PINMODE 50

#define STANDBY 0b10001100 // CE# CLE ALE WE# RE# WP# X X
#define COMMAND_INPUT  0b01001100 // CE# CLE ALE WE# RE# WP# X X
#define ADDRESS_INPUT  0b00101100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_INPUT     0b00001100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_OUTPUT    0b00011100 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT  0b00001000 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT_OFF   0b00001100 // CE# CLE ALE WE# RE# WP# X X

#define WE_HIGH        0b00010000  // CE# CLE ALE WE# RE# WP# X X
#define RE_HIGH       0b00001000


typedef struct DateTime {
  uint8_t year;          // 0 - 128
  uint8_t month;         // 1 - 12
  uint8_t day;           // 1 - 32
  uint8_t hour;          // 0 - 23
  uint8_t minute;        // 0 - 59
  uint8_t second;        // 0 - 59
  uint16_t millisecond;  // 0 - 999
  uint16_t microsecond;  // 0 - 999
} DateTime;


typedef struct Vector3 {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} Vector3;


typedef struct GNSS_Data{
  uint16_t latitude;
  uint16_t longitude;
  uint32_t heading1;
  uint32_t velocity;
} GNSS_Data;


typedef struct FrameArray {
  DateTime date;
  uint16_t changeFlag;  //IS THIS NEEDED? CAN THIS BE DONE BETTER?
  Vector3 accelHighG;
  Vector3 accelLowG;
  Vector3 gyroscope;
  uint32_t barometer;
  uint16_t thermocouple[4];
  uint16_t humidity;
  uint32_t temp;
  uint16_t magneticFieldStrength;
  GNSS_Data GNSS;
  uint16_t ADC[2];
  uint8_t hammingCode[8];
  uint16_t CRC_Check;
  int successFlag; // Not used in zip
} FrameArray;


typedef struct Address {
    uint16_t block;  // 12 bits
    uint8_t page;    // 6 bits
    uint16_t column; // 13 bits (12 used)
} Address;


uint16_t data0 = PIN('D', 0); //2;
uint16_t data1 = PIN('D', 1); //3;
uint16_t data2 = PIN('D', 2); //4;
uint16_t data3 = PIN('D', 3); //5;
uint16_t data4 = PIN('D', 4); //11;
uint16_t data5 = PIN('D', 5); //12;
uint16_t data6 = PIN('D', 6); //8;
uint16_t data7 = PIN('D', 7); //9;

uint16_t WP  = PIN('D', 8); // 13;
uint16_t WE  = PIN('D', 9); // 14;
uint16_t ALE = PIN('E', 7); // 15;
uint16_t CLE = PIN('E', 8); // 16;
uint16_t CE  = PIN('E', 9); // 17;
uint16_t RE  = PIN('E', 11); // 18;
uint16_t RB  = PIN('E', 13); // 19;


uint32_t frameAddressPointer = 0;


uint8_t globalPinMode = GPIO_MODE_OUTPUT;


/**
  @brief TODO
  @param arr
  @param pos
  @return
*/
bool get_bit_arr(uint8_t *arr, int pos) {
  return (bool)(arr[pos/8] & (1 << (7-(pos%8))));
}


/**
  @brief TODO
  @param byte
  @param pos
  @return
*/
bool get_bit(uint8_t byte, int pos) {
  return (bool)(byte & (1 << (7-(pos%8))));
}


/**
  @brief TODO
  @param unzippedData
  @param zippedData
  @return
*/
void zip(FrameArray unzippedData, uint8_t *zippedData) {
  int i = -1;

  zippedData[i++] = unzippedData.date.year;
  zippedData[i++] = unzippedData.date.month;
  zippedData[i++] = unzippedData.date.day;
  zippedData[i++] = unzippedData.date.minute;
  zippedData[i++] = unzippedData.date.second;
  zippedData[i++] = (uint8_t)((unzippedData.date.millisecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.date.millisecond & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.date.microsecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.date.microsecond & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.changeFlag >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.changeFlag & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.accelHighG.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelHighG.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accelHighG.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelHighG.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accelHighG.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelHighG.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.accelLowG.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelLowG.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accelLowG.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelLowG.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accelLowG.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accelLowG.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.gyroscope.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.gyroscope.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.gyroscope.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.gyroscope.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.gyroscope.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.gyroscope.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.barometer >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.barometer >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.barometer & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.thermocouple[0] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.thermocouple[0] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.thermocouple[1] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.thermocouple[1] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.thermocouple[2] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.thermocouple[2] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.thermocouple[3] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.thermocouple[3] & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.humidity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.humidity & 0xFF);
  
  zippedData[i++] = (uint8_t)((unzippedData.temp >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.temp >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.temp >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.temp & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.magneticFieldStrength >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.magneticFieldStrength & 0xFF); 

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.latitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.latitude & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.longitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.longitude & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.heading1 >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.heading1 >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.heading1 >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.heading1 & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.velocity >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.velocity >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.velocity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.velocity & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.ADC[0] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.ADC[0] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.ADC[1] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.ADC[1] & 0xFF);

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

FrameArray unzip(uint8_t *zippedData) {
  FrameArray unzippedData;
  int i = -1;

  unzippedData.date.year = zippedData[i++];
  unzippedData.date.month = zippedData[i++];
  unzippedData.date.day = zippedData[i++];
  unzippedData.date.minute = zippedData[i++];
  unzippedData.date.second = zippedData[i++];
  unzippedData.date.millisecond = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.date.millisecond |= zippedData[i++];
  unzippedData.date.microsecond = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.date.microsecond |= zippedData[i++];

  unzippedData.changeFlag = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.changeFlag |= zippedData[i++];

  unzippedData.accelHighG.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelHighG.x |= zippedData[i++];
  unzippedData.accelHighG.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelHighG.y |= zippedData[i++];
  unzippedData.accelHighG.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelHighG.z |= zippedData[i++];

  unzippedData.accelLowG.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelLowG.x |= zippedData[i++];
  unzippedData.accelLowG.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelLowG.y |= zippedData[i++];
  unzippedData.accelLowG.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.accelLowG.z |= zippedData[i++];

  unzippedData.gyroscope.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.gyroscope.x |= zippedData[i++];
  unzippedData.gyroscope.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.gyroscope.y |= zippedData[i++];
  unzippedData.gyroscope.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.gyroscope.z |= zippedData[i++];

  unzippedData.barometer = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.barometer |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.barometer |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.barometer |= zippedData[i++];

  unzippedData.thermocouple[0] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.thermocouple[0] |= zippedData[i++];
  unzippedData.thermocouple[1] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.thermocouple[1] |= zippedData[i++];
  unzippedData.thermocouple[2] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.thermocouple[2] |= zippedData[i++];
  unzippedData.thermocouple[3] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.thermocouple[3] |= zippedData[i++];

  unzippedData.humidity = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.humidity |= zippedData[i++];
  
  unzippedData.temp = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.temp |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.temp |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.temp |= zippedData[i++];

  unzippedData.magneticFieldStrength = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.magneticFieldStrength |= zippedData[i++];

  unzippedData.GNSS.latitude = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.latitude |= zippedData[i++];
  unzippedData.GNSS.longitude = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.longitude |= zippedData[i++];

  unzippedData.GNSS.heading1 = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.GNSS.heading1 |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.GNSS.heading1 |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.heading1 |= zippedData[i++];

  unzippedData.GNSS.velocity = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.GNSS.velocity |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.GNSS.velocity |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.velocity |= zippedData[i++];

  unzippedData.ADC[0] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.ADC[0] |= zippedData[i++];
  unzippedData.ADC[1] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.ADC[1] |= zippedData[i++];

  unzippedData.hammingCode[0] = zippedData[118];
  unzippedData.hammingCode[1] = zippedData[119];
  unzippedData.hammingCode[2] = zippedData[120];
  unzippedData.hammingCode[3] = zippedData[121];
  unzippedData.hammingCode[4] = zippedData[122];
  unzippedData.hammingCode[5] = zippedData[123];
  unzippedData.hammingCode[6] = zippedData[124];
  unzippedData.hammingCode[7] = zippedData[125];

  unzippedData.CRC_Check = (zippedData[126] << 8) & (0xFF << 8);
  unzippedData.CRC_Check |= zippedData[127]; 

  return unzippedData;
}


/**
  @brief Prints a byte in binary format
  @param myByte: byte to be printed
*/
void print_byte(uint8_t myByte) {
  printf("0b");
  for (int i = 7; i >= 0; i--) {
    printf("%i", (myByte >> i) & 0b1);
  }
  printf("\r\n");
}


/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/
void print_frame(uint8_t dataArray[]) {
  printf("u");
  for (int i = 0; i < 128; i++) {
    /*if(dataArray[i] < 16) {
      printf("0");
    }*/
    printf("%02X", dataArray[i]);
  }
  //printfln("");
}


/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/	
void print_frameHex(uint8_t dataArray[]) {
  for (int i = 0; i < 128; i++) {
    if (dataArray[i] < 16) {
      printf("0");
    }
    printf("%i", dataArray[i]);
    printf(", ");
  }
  printf("\r\n");
}


/**
  @brief TODO
  @param arr: array to be filled
  @param val: value to fill array with
  @param num: number of elements to fill
*/
void _memset(uint8_t *arr, uint8_t val, int num){
  for (int i = 0; i < num; i++) {
    arr[i] = val;
  }
}


/**
  @brief TODO
  @param frameFormat
*/
void print_frame_array(FrameArray frameFormat) {
  uint8_t dataArray[128];
  _memset(dataArray, 0, 128);
  zip(frameFormat, dataArray);
  print_frame(dataArray);
}


/**
  @brief TODO 
*/
void wait_for_ready_flag() {
  int count = 1000*100; // Try for 1 second before giving error
  while (gpio_read(RB) == LOW && count > 0) {
    delay_nanoseconds(10);
    count--;
  }
  if (count < 1) {
    printf("waitForReadyFlag: TIMEOUT\r\n");
  } 
}


/**
  @brief TODO 
*/
void set_pin_modes() {
  gpio_set_mode(data0, globalPinMode);
  gpio_set_mode(data1, globalPinMode);
  gpio_set_mode(data2, globalPinMode);
  gpio_set_mode(data3, globalPinMode);
  gpio_set_mode(data4, globalPinMode);
  gpio_set_mode(data5, globalPinMode);
  gpio_set_mode(data6, globalPinMode);
  gpio_set_mode(data7, globalPinMode);
  delay_microseconds(DELAY_PINMODE);
}


/**
  @brief TODO 
*/
void set_control_pins(uint8_t controlRegister) {  // CE# CLE ALE WE# RE# WP#
  gpio_write(CE, get_bit(controlRegister, 0));
  gpio_write(CLE, get_bit(controlRegister, 1));
  gpio_write(ALE, get_bit(controlRegister, 2));
  gpio_write(WE, get_bit(controlRegister, 3));
  gpio_write(RE, get_bit(controlRegister, 4));
  gpio_write(WP, get_bit(controlRegister, 5));
}


/**
  @brief TODO 
*/
void set_data_pins(uint8_t Byte) {
  if (globalPinMode == GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_OUTPUT;
    set_pin_modes();
  }

  gpio_write(data0, get_bit(Byte, 7));
  gpio_write(data1, get_bit(Byte, 6));
  gpio_write(data2, get_bit(Byte, 5));
  gpio_write(data3, get_bit(Byte, 4));
  gpio_write(data4, get_bit(Byte, 3));
  gpio_write(data5, get_bit(Byte, 2));
  gpio_write(data6, get_bit(Byte, 1));
  gpio_write(data7, get_bit(Byte, 0));
}


/**
  @brief TODO
  @param cmd
  @param mode 
*/
void send_byte_to_flash(uint8_t cmd, uint8_t mode) {
  //delayNanoseconds(DELAY);
  set_control_pins(mode);
  set_data_pins(cmd);
  //delayNanoseconds(DELAY);
  set_control_pins(mode | WE_HIGH);
  //delayNanoseconds(DELAY);
}


/**
  @brief TODO
  @return 
*/
uint8_t receive_byte_from_flash() {
  delay_nanoseconds(DELAY);
  set_control_pins(DATA_OUTPUT);
  delay_nanoseconds(DELAY);
  set_control_pins(DATA_OUTPUT & (~RE_HIGH));  // setting RE LOW
  delay_nanoseconds(DELAY);

  if (globalPinMode != GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_INPUT;
    setPinModes();
  }

  uint8_t data = (gpio_read(data7) << 7)
               | (gpio_read(data6) << 6)
               | (gpio_read(data5) << 5)
               | (gpio_read(data4) << 4)
               | (gpio_read(data3) << 3)
               | (gpio_read(data2) << 2)
               | (gpio_read(data1) << 1)
               | (gpio_read(data0) << 0);
  return data;
}


/**
  @brief sends the 5-byte-address to the nand using the frame and byte address as input
  @note 8,388,608 frames each with 128 bytes. frameAddr has 23 valid bits. byteAddr has 7 valid bits
  @param frameAddr
  @param byteAddr 
*/
void send_addr_to_flash(uint32_t frameAddr, uint8_t byteAddr) {
  Address addr = {(frameAddr >> 11) & 0b0000111111111111,                             // block
                        (frameAddr >> 5) & 0b00111111,                                // page
                        ((frameAddr & 0b00011111) << 7) | (byteAddr & 0b01111111)};   // column 

  send_byte_to_flash((uint8_t)(addr.column & 0b0000000011111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.column & 0b0001111100000000) >> 8), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)(((addr.block & 0b0000000000000011) << 6)) | (addr.page & 0b00111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}


/**
  @brief TODO
  @param blockAddr 
*/
void send_block_addr_to_flash(uint32_t blockAddr) {
  send_byte_to_flash((uint8_t)(((blockAddr & 0b0000000000000011) << 6) | (0b00000000 & 0b00111111)), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}


/**
  @brief Read the status register from the nand flash
  @return 
*/
uint8_t read_flash_status() {
  wait_for_ready_flag();
  send_byte_to_flash(0x70, COMMAND_INPUT);
  return receive_byte_from_flash();
}


/**
  @brief Read the ID register from the nand flash
  @return 
*/
uint64_t read_flash_ID() {
  uint64_t id = 0;

  wait_for_ready_flag();
  send_byte_to_flash(0x90, COMMAND_INPUT);
  send_byte_to_flash(0x00, ADDRESS_INPUT);

  for (int i = 0; i < 5; i++) {
    printf("ID ");
    printf("%i", i);
    printf(": ");
    uint8_t byte = receive_byte_from_flash();
    id |= byte << (4-i);
    print_byte(byte);
  }

  return id;
}


/**
  @brief TODO
*/
void write_protection() {
  wait_for_ready_flag();
  set_control_pins(WRITE_PROTECT);  // Write Protection
}


/**
  @brief Code to read 1 frame from flash
*/
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


/**
  @brief Code to write 1 frame to the flash
*/
void write_frame(uint32_t frameAddr, uint8_t *bytes) {
  wait_for_ready_flag();
  send_byte_to_flash(0x80, COMMAND_INPUT);
  send_addr_to_flash(frameAddr, 0);  // Address Input
  delay(1);
  for (int byteAddr = 0; byteAddr < 128; byteAddr++) {
    send_byte_to_flash(bytes[byteAddr], DATA_INPUT);
  }
  
  send_byte_to_flash(0x10, COMMAND_INPUT);
}


/**
  @brief A blocking function which will erase a block on the flash
*/
void erase_block(uint32_t blockAddr) {
  wait_for_ready_flag();
  send_byte_to_flash(0x60, COMMAND_INPUT);
  send_block_addr_to_flash(blockAddr);
  send_byte_to_flash(0xD0, COMMAND_INPUT);
  wait_for_ready_flag();  // Blocking Function
}


/**
  @brief A blocking function which will erase a block on the flash
*/
void erase_all(){
  printf("WARNING: ERASING ALL DATA (UNPLUG NAND FLASH TO ABORT)\r\n");

  for (int countDown = 10; countDown > 0; countDown--) {
    printf("ERASING DATA IN: ");
    printf("%i", countDown);
    printf(" Seconds\r\n");
    delay(1);
  }

  for (uint32_t block = 0; block < 64*4096; block++) {
    erase_block(block);
    if (block%5000 == 0) {
      printf("ERASING [");
      for (int i = 0; i < 50; i++) {
        if(i < block/(64*4096*0.01*2)){
          printf("#");
        } else {
          printf(" ");
        }
      }
      printf("] - ");
      int percentage = (int)(block/(64*4096*0.01));
      printf("%d", percentage);
      printf("%%\r\n");
    }
  }
  printf("ERASING COMPLETE \r\n");
}


/**
  @brief TODO
*/
uint16_t max(uint16_t x1, uint16_t x2){
  return (x1 > x2) ? x1 : x2;
}


/**
  @brief TODO
*/
uint16_t min(uint16_t x1, uint16_t x2){
  return (x1 < x2) ? x1 : x2;
}


/**
  @brief TODO
*/
uint16_t diff(uint16_t x1, uint16_t x2) {
  return (uint16_t)abs((int)((int)x1 - (int)x2));
}

/**
  @brief Function which searches for next available block and returns the first frame address of that block
  @return
*/
uint32_t get_next_available_frame_addr() {
  uint16_t prevPointer = 4096;
  uint16_t pointer = prevPointer / 2;
  uint8_t _check = 0;
  
  for (int i = 0; i < 11; i++) {
    read_frame(max(pointer - 1, 0) * 64 * 32, &_check, 1);  // Dosn't need to be the whole frame -------
    uint16_t difference = diff(pointer, prevPointer);
    prevPointer = pointer;

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

/*
void test_routine() {
  uint8_t bytes[128];
  _memset(bytes, 0xFF, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    bytes[i] = i;
  }

  int wrong = 0;
  int total = 0;

  int numOfFramesToCheck = 1000;
  
  //unsigned long timeBegin = micros();

  for (int i = 0; i < numOfFramesToCheck; i++) {
    write_frame(i, bytes);  // Write frame 0
    total += 128;
  }

  unsigned long timeEnd = micros();

  for (int i = 0; i < numOfFramesToCheck; i++) {
    uint8_t bytesRead[128];
    read_frame(i, bytesRead, 128);

    for (int j = 0; j < 128; j++) {
      if (bytes_read[j] != j) {
        wrong++;
      }
    }
  }
  
  //unsigned long duration = timeEnd - timeBegin;
  printf("ms delay: ");
  printf(DELAY + "\n");
  printf("Incorrect Bytes: ");
  printf(wrong + "\n");
  printf("Total Bytes: ");
  printf(total + "\n");
  printf("Percent Incorrect: ");
  printf("%f", ((float)wrong * 100) / (float)total);
  printf(" %%\n");
  printf("Time took to write: ");
  printf(duration / 1000);
  printf(" ms\n");
  printf("Time waiting while nand flash was busy: ");
 // printf(count / 1000);
  printf(" ms\n");
  printf("Next available frame addr: ");
  printf(getNextAvailableFrameAddr() + "\n");
}
*/


/**
  @brief TODO
*/
void initialise_flash() {
  gpio_set_mode(data0, GPIO_MODE_OUTPUT);
  gpio_set_mode(data1, GPIO_MODE_OUTPUT);
  gpio_set_mode(data2, GPIO_MODE_OUTPUT);
  gpio_set_mode(data3, GPIO_MODE_OUTPUT);
  gpio_set_mode(data4, GPIO_MODE_OUTPUT);
  gpio_set_mode(data5, GPIO_MODE_OUTPUT);
  gpio_set_mode(data6, GPIO_MODE_OUTPUT);
  gpio_set_mode(data7, GPIO_MODE_OUTPUT);

  gpio_set_mode(ALE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CLE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(RE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WP,  GPIO_MODE_OUTPUT);

  gpio_set_mode(RB,  GPIO_MODE_INPUT);
  
  frameAddressPointer = get_next_available_frame_addr();

  if (read_flash_ID() != 0){
    printf("Flash Working Correctly\r\n");
  }
}

// --------------------------------

/**
  @brief Calculates CRC16-CCITT Checksum
  @return CRC16-CCITT Checksum
*/
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


/**
  @brief Hamming code hashing
*/
void hash(uint8_t *_input, uint8_t *_output) {
  _memset(_output, 0, 120);

  for (int i = 0; i < 8*120; i++) {
    int j = ((i%120)*8) + (i/120);
    _output[i/8] |= getBitArr(_input, j) << (7-(i%8));
  }
}


/**
  @brief TODO
  @return
*/
bool is_power_of_two(int x) {
    return (x != 0) && ((x & (x - 1)) == 0);
}


/**
  @brief Calculate parity bits for a given encoded data frame
*/
void calculate_parity_bits(uint8_t *_input, uint8_t *_output) {
  uint8_t hashedData[120];
  _memset(hashedData, 0, 120);

  uint8_t condition[120];
  _memset(condition, 0, 120);
  for (int i = 0; i < 118; i++) {
    condition[i] = _input[i];
  }
  
  hash(condition, hashedData);

  for (int _set = 0; _set < 8; _set++) {
    uint8_t _word[15];
    for (int i = 0; i < 15; i ++) {
      _word[i] = hashedData[(_set * 15) + i];
    }
    
    // Initialize parity bits to 0
    uint8_t parities = 0;
    // Calculate parity bits
    for (int i = 0; i < 8; i++) {
      // Calculate bit position of this parity bit
      int bit_pos = 1 << i;
      
      // Calculate parity for this bit position
      uint8_t parity = 0;
      int k = 0;
      for (int j = 0; j < 128; j++) { // j from 0 - 128
        if (j + 1 != 1 && isPowerOfTwo(j + 1) == 0) {
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


/**
  @brief TODO
  @return
*/
void encode_parity(FrameArray dataFrame, uint8_t *bytes) {
  zip(dataFrame, bytes);

  uint8_t parities[8];
  calculate_parity_bits(bytes, parities);
  for (int i = 0; i < 8; i++) {
    bytes[118+i] = parities[i];
  }
  uint16_t CRC_Check = calculateCRC(bytes, 126);
  bytes[126] = (uint8_t)((CRC_Check >> 8) & 0xFF);
  bytes[127] = (uint8_t)(CRC_Check & 0xFF);
}


/**
  @brief TODO
*/
void print_capacity_info() {
  uint32_t lastFrameUsed = get_next_available_frame_addr();
  printf("Used: ");
  uint32_t usedInMB = (lastFrameUsed*128)/1000000;
  printf("%i", (int)usedInMB);
  printf(" MB (");
  printf("%f", lastFrameUsed/(4096*64*32*0.01));
  printf("%%) | ");
  printf("Estimated Time Remaining (hh:mm:ss): ");
  uint32_t hours = ((4096*64*32) - lastFrameUsed)/(1000*60*60)%24;
  uint32_t minutes = ((4096*64*32) - lastFrameUsed)/(1000*60)%60;
  uint32_t seconds = (((4096*64*32) - lastFrameUsed)/(1000))%60;
  if (hours<10) printf("0");
  printf("%i", (int)hours);
  printf(":");
  if (minutes<10) printf("0");
  printf("%i", (int)minutes);
  printf(":");
  if (seconds<10) printf("0");
  printf("%i", (int)seconds);
  printf("\r\n");
}


/**
  @brief TODO
*/
int log_frame(FrameArray _input) {
  //printf("LOGFRAME addr ");
  //printfln(frameAddressPointer);

  // frameArray to array of bytes
  if (frameAddressPointer <= 8388607) {
    uint8_t encoded[128];
    _memset(encoded, 0, 128);
    encode_parity(_input, encoded);
    //printFrame(encoded);
    write_frame(frameAddressPointer, encoded);
    frameAddressPointer++;
  } else {
    printf("Overflow Error\r\n");  // ERROR
    return STORAGE_FULL_ERROR;
  }
  // If the pointer is near the capacity of the storage
  // (95% full, 6.5 minutes left of recording)
  if (frameAddressPointer >= 8000000) {   
    return STORAGE_FULL_WARNING;
  }
  return SUCCESS;  // Successfully written
}


/**
  @brief TODO
*/
FrameArray recall_frame(uint32_t frameAddr) {
  uint8_t encoded[128];
  _memset(encoded, 0, 128);
  FrameArray _output;
  
  // Attempts re-reading the data from the flash 10 times before it gives up
  //_output.successFlag = DATA_CORRUPTED;
  //int timeout = 10;  
  //while(_output.successFlag == DATA_CORRUPTED && timeout > 0){
  //  timeout--;
  read_frame(frameAddr, encoded, 128);
  //  if ((encoded[0] & encoded[1]) != 0xFF) {
  //    _output = decodeParity(encoded);  // CHANGE TO ALLOW FOR INT RETURN OF STATUS
  //  } else {
  //    _output = unzip(encoded);  // Don't bother decoding parity bits
  //    _output.successFlag = EMPTY;
  //  }
  //}
  _output = unzip(encoded);  // Don't bother decoding parity bits
  return _output;
}


/**
  @brief TODO
*/
void read_all (){
  FrameArray _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = getNextAvailableFrameAddr();
  
  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int data_empty = 0;

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    _output = recall_frame(i);
    /*int flag = _output.successFlag;
    if(flag == DATA_INTACT){
      data_intact += 1;
      printFrame(_output);
    } else if(flag == DATA_FIXED) {
      data_fixed += 1;
      printFrame(_output);
    } else if(flag == EMPTY) {
      data_empty += 1;
    } else {
      data_error += 1;
    }*/
    printFrameArray(_output);
  }
  
  printf("----------------------------------------------\r\n");
  printCapacityInfo();
  printf("data_empty: ");
  printf(data_empty + "\r\n");
  printf("data_intact: ");
  printf(data_intact + "\r\n");
  printf("data_fixed: ");
  printf(data_fixed + "\r\n");
  printf("data_error: ");
  printf(data_error + "\r\n");
  printf("Percent Correct Data : ");
  printf("%i", (data_intact + data_fixed)/(4096*64*(4096/128)));
  printf("%%\r\n");
}
