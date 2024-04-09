/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Thomas Groom, Alexnadra Posta
  Created on: 09 March 2023
  Description: NAND Flash driver
*/

#pragma once

#ifndef NAND_DRIVER_H
#define NAND_DRIVER_H

#include "mcu.h"

// Defines which are used when returning the status of a write to flash
#define NONE 0
#define SUCCESS 1
#define STORAGE_FULL_WARNING 2
#define STORAGE_FULL_ERROR 3

// Defines used in the status of frames which have been gone through error correction
#define DATA_INTACT 1
#define DATA_FIXED 2
#define DATA_CORRUPTED 3
#define EMPTY 4

// Defines a global delay mostly for debugging
#define DELAY 1
#define DELAY_PINMODE 50

// Manipulate control pins for commands
#define STANDBY           0b10001100 // CE# CLE ALE WE# RE# WP# X X
#define COMMAND_INPUT     0b01001100 // CE# CLE ALE WE# RE# WP# X X
#define ADDRESS_INPUT     0b00101100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_INPUT        0b00001100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_OUTPUT       0b00011100 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT     0b00001000 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT_OFF 0b00001100 // CE# CLE ALE WE# RE# WP# X X

// Defines used to toggle the WE or RE pins to latch a byte into the flash chip
#define WE_HIGH           0b00010000  // CE# CLE ALE WE# RE# WP# X X
#define RE_HIGH           0b00001000

// Structure used to break the address bits into block->page->column which have meanings defined by the flash chip
typedef struct Address {
    uint16_t block;  // 12 bits
    uint8_t page;    // 6 bits
    uint16_t column; // 13 bits (12 used)
} Address;

// Data Pins - pins used in data/control/address transmission (8 bit parrallel bus)
uint16_t data0 = PIN('D', 0); // 2;
uint16_t data1 = PIN('D', 1); // 3;
uint16_t data2 = PIN('D', 2); // 4;
uint16_t data3 = PIN('D', 3); // 5;
uint16_t data4 = PIN('D', 4); // 11;
uint16_t data5 = PIN('D', 5); // 12;
uint16_t data6 = PIN('D', 6); // 8;
uint16_t data7 = PIN('D', 7); // 9;

// Control Pins - pins used to control the state of the flash chip
uint16_t WP  = PIN('D', 8);  // 13, Write Protection;
uint16_t WE  = PIN('D', 9);  // 14, Write Enable;
uint16_t ALE = PIN('E', 7);  // 15, Address latch enable (where in the memory to store);
uint16_t CLE = PIN('E', 8);  // 16, Command latch enable (When on, you can sent command);
uint16_t CE  = PIN('E', 9);  // 17, Check Enable (in case we want to test separatly);
uint16_t RE  = PIN('E', 11); // 18, Read Enable;
uint16_t RB  = PIN('A', 8);  // 19, Ready/Busy;

// Stores the address of the next available frame (set of 128 bytes) (assumes all frames prior to this are full of valuable data)
// This variable is set by the get_next_available_frame_addr() function
uint32_t frameAddressPointer = 0;

// Set all pins as gpio outputs by default
uint8_t globalPinMode = GPIO_MODE_OUTPUT;

/**
  @brief: This function returns only the specified bit from an array of bytes
  @param arr: an array of bytes (uint8)
  @param pos: which bit in the array of bytes to access (msb: 0 to lsb: 8*length(arr)-1)
  @return: the value of the bit at position "pos" in the byte array "arr"
*/
static inline bool get_bit_arr(uint8_t *arr, int pos) {
  return (bool)(arr[pos/8] & (1 << (7-(pos%8))));
}

/**
  @brief This function returns the specific bit within a byte
  @param byte: the input byte
  @param pos: the position of the bit in question (msb: 0 to lsb: 7)
  @return: The value of the "pos" bit in the byte
*/
static inline bool get_bit(uint8_t byte, int pos) {
  return (bool)(byte & (1 << (7-(pos%8))));
}

/**
  @brief Converts a FrameArray struct to an array of bytes
  @param unzippedData: the frame array object to zip
  @param zippedData: a pass by reference to the byte array where the output is stored
  @return: None
*/
static inline void zip(FrameArray unzippedData, uint8_t *zippedData) {
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

/**
  @brief Converts a byte array to a FrameArray struct
  @param zippedData: a pass by reference to the byte array to be converted
  @return: the FrameArray
*/
static inline FrameArray unzip(uint8_t *zippedData) {
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
static inline void print_byte(uint8_t myByte) {
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
static inline void print_frame(uint8_t dataArray[]) {
  printf("u");
  for (int i = 0; i < 128; i++) {
    /*if(dataArray[i] < 16) {
      printf("0");
    }*/
    printf("%X,", dataArray[i]); // %02X adds a paddings of 0
  }
  printf("\r\n");
}

/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/	
static inline void print_frameHex(uint8_t dataArray[]) {
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
  @brief Allocates the space for the frame and fills it with 0
  @param arr: array to be filled
  @param val: value to fill array with
  @param num: number of elements to fill
  @note in C, you only define a pointer and won't allocate the bytes (as C++),
  you need to do it manually. We also need to fill the array with 0s
*/
static inline void _memset(uint8_t *arr, uint8_t val, int num){
  for (int i = 0; i < num; i++) {
    arr[i] = val;
  }
}

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_frame_array(FrameArray frameFormat) {
  /*
  uint8_t dataArray[128];
  _memset(dataArray, 0, 128);
  zip(frameFormat, dataArray);
  print_frame(dataArray);
  */

  printf("Date: %i/%i/%i, %i:%i:%i:%i:%i\r\n", frameFormat.date.day, frameFormat.date.month, frameFormat.date.year,
          frameFormat.date.hour, frameFormat.date.minute, frameFormat.date.second, frameFormat.date.millisecond, frameFormat.date.microsecond );
  //printf("ChangeFlag: %i\r\n", frameFormat.changeFlag);
  printf("Accel HG:\tX: %i,\tY: %i,\tZ: %i\t\r\n", frameFormat.accelHighG.x, frameFormat.accelHighG.y, frameFormat.accelHighG.z);
  //printf("Accel LG:\tX: %i,\tY: %i,\tZ: %i\r\n", frameFormat.accelLowG.x, frameFormat.accelLowG.y, frameFormat.accelLowG.z);
  //printf("Gyro: \t\tX: %i,\tY: %i,\tZ: %i\r\n", frameFormat.gyroscope.x, frameFormat.gyroscope.y, frameFormat.gyroscope.z);
  printf("Barometer: %i\r\n", frameFormat.barometer);
  //printf("Therocouples: \t1: %i,\t2: %i,\t3: %i,\t4: %i\r\n", frameFormat.thermocouple[0], frameFormat.thermocouple[1], frameFormat.thermocouple[2], frameFormat.thermocouple[3]);
  //printf("Humidity: %i\r\n", frameFormat.humidity);
  printf("Temp: %i\r\n", frameFormat.temp);
  //printf("Magnetic field: %i\r\n", frameFormat.magneticFieldStrength);
  //printf("GNSS:\tLat: %i,\tLong: %i,\tHead: %i,\tVel: %i\r\n", frameFormat.GNSS.latitude, frameFormat.GNSS.longitude, frameFormat.GNSS.heading1, frameFormat.GNSS.velocity);
  //printf("ADC: %i\r\n", frameFormat.ADC);
}

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_csv_header() {
  printf("DATE,");
  printf("TIME,");
  printf("ChangeFlag,");
  printf("AccelH X,AccelH Y,AccelH Z,");
  printf("AccelL X,AccelL Y,AccelL Z,");
  printf("GyroX,GyroY,GyroZ,");
  printf("Barometer,");
  printf("Thermocouple 1,Thermocouple 2,Thermocouple 3,Thermocouple 4,");
  printf("Humidity,");
  printf("Temp,");
  printf("MagneticField,");
  printf("GNSS 1,GNSS 2,GNSS 3,GNSS 4,");
  printf("ADC,");
  printf("\r\n");
}
/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_frame_csv(FrameArray frameFormat) {
  printf("%i/%i/%i, %i:%i:%i:%i:%i,", frameFormat.date.day, frameFormat.date.month, frameFormat.date.year,
          frameFormat.date.hour, frameFormat.date.minute, frameFormat.date.second, frameFormat.date.millisecond, frameFormat.date.microsecond );
  printf("%i,", frameFormat.changeFlag);
  printf("%i,%i,%i,", frameFormat.accelHighG.x, frameFormat.accelHighG.y, frameFormat.accelHighG.z);
  printf("%i,%i,%i,", frameFormat.accelLowG.x, frameFormat.accelLowG.y, frameFormat.accelLowG.z);
  printf("%i,%i,%i,", frameFormat.gyroscope.x, frameFormat.gyroscope.y, frameFormat.gyroscope.z);
  printf("%i,", frameFormat.barometer);
  printf("%i,%i,%i,%i,", frameFormat.thermocouple[0], frameFormat.thermocouple[1], frameFormat.thermocouple[2], frameFormat.thermocouple[3]);
  printf("%i,", frameFormat.humidity);
  printf("%i,", frameFormat.temp);
  printf("%i,", frameFormat.magneticFieldStrength);
  printf("%i,%i,%i,%i,", frameFormat.GNSS.latitude, frameFormat.GNSS.longitude, frameFormat.GNSS.heading1, frameFormat.GNSS.velocity);
  printf("%i,", frameFormat.ADC);
  printf("\r\n");
}

/**
  @brief Wait for the ready flag to be set
*/
static inline void wait_for_ready_flag() {
  int count = 1000*100; // Try for 1 second before giving error
  while (gpio_read(RB) == LOW && count > 0) {
    delay_ms(1);
    count--;
  }
  if (count < 1) {
    printf("waitForReadyFlag: TIMEOUT\r\n");
  } 
}

/**
  @brief Set all pins to the global pin mode (either GPIO_MODE_INPUT or GPIO_MODE_OUTPUT)
*/
static inline void set_pin_modes() {
  gpio_set_mode(data0, globalPinMode);
  gpio_set_mode(data1, globalPinMode);
  gpio_set_mode(data2, globalPinMode);
  gpio_set_mode(data3, globalPinMode);
  gpio_set_mode(data4, globalPinMode);
  gpio_set_mode(data5, globalPinMode);
  gpio_set_mode(data6, globalPinMode);
  gpio_set_mode(data7, globalPinMode);
  delay_ms(DELAY_PINMODE);
}

/**
  @brief Set the control pins based on the input byte (i.e. COMMAND_INPUT, DATA_INPUT)
*/
static inline void set_control_pins(uint8_t controlRegister) {  // CE# CLE ALE WE# RE# WP#
  gpio_write(CE, get_bit(controlRegister, 0));  // TODO: Why is this commented out?
  gpio_write(CLE, get_bit(controlRegister, 1));
  gpio_write(ALE, get_bit(controlRegister, 2));
  gpio_write(WE, get_bit(controlRegister, 3));
  gpio_write(RE, get_bit(controlRegister, 4));
  gpio_write(WP, get_bit(controlRegister, 5));
}

/**
  @brief Set the data pins to the desired input byte (can be data, control or address information)
*/
static inline void set_data_pins(uint8_t Byte) {
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
  @brief Write a single byte to the flash with control pins set to the "mode" byte and data pins set to the "cmd" byte (can be data, command or address byte)
  @param cmd: composed of the data pins
  @param mode: composed of the control pins
*/
static inline void send_byte_to_flash(uint8_t cmd, uint8_t mode) {
  //delayNanoseconds(DELAY); // include if needed 
  //(pins were swicthed too quickly 600MHZ)
  set_control_pins(mode);
  set_data_pins(cmd);
  //delayNanoseconds(DELAY);
  set_control_pins(mode | WE_HIGH); // lanch what is in the data bus in the memory
  //delayNanoseconds(DELAY);
}

/**
  @brief Read a single byte from the flash (assumes address to read from has been set before calling this function)
  @return 
*/
static inline uint8_t receive_byte_from_flash() {  
  delay_ms(DELAY);
  set_control_pins(DATA_OUTPUT);
  delay_ms(DELAY);
  set_control_pins(DATA_OUTPUT & (~RE_HIGH));  // setting RE LOW
  delay_ms(DELAY);

  if (globalPinMode != GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_INPUT;
    set_pin_modes();
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
  @brief sends the 5-byte-address to the nand flash using the frame and byte address as input
  @note 8,388,608 frames each with 128 bytes. frameAddr has 23 valid bits. byteAddr has 7 valid bits
  @param frameAddr: The address of the frame to write/read to/from (0 to 8,388,608)
  @param byteAddr: The address of which byte to write/read to/from within the frame (0 - 127) (typically 0 as we want to start writing/reading from the first byte of a frame)
*/
static inline void send_addr_to_flash(uint32_t frameAddr, uint8_t byteAddr) {
  Address addr = {(frameAddr >> 11) & 0b0000111111111111,                      // block
                  (frameAddr >> 5) & 0b00111111,                                // page
                  ((frameAddr & 0b00011111) << 7) | (byteAddr & 0b01111111)}; // column 

  send_byte_to_flash((uint8_t)(addr.column & 0b0000000011111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.column & 0b0001111100000000) >> 8), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)(((addr.block & 0b0000000000000011) << 6)) | (addr.page & 0b00111111), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((addr.block & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}

/**
  @brief Sets the address to the first byte of the specified block
  @param blockAddr: Which block to set the address to (0 - 4095)
*/
static inline void send_block_addr_to_flash(uint32_t blockAddr) {
  send_byte_to_flash((uint8_t)(((blockAddr & 0b0000000000000011) << 6) | (0b00000000 & 0b00111111)), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000001111111100) >> 2), ADDRESS_INPUT);
  send_byte_to_flash((uint8_t)((blockAddr & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}

/**
  @brief Read the status register from the nand flash. Same as the RB (Read/Busy) input pin
  @return The status register of the flash
*/
static inline uint8_t read_flash_status() {
  wait_for_ready_flag();
  send_byte_to_flash(0x70, COMMAND_INPUT);
  return receive_byte_from_flash();
}

/**
  @brief Read the ID register from the nand flash (not unique to each nand flash?)
  @return ID register of the nand flash
*/
static inline uint64_t read_flash_ID() {
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
  @brief Enable the flash write protection to prevent writing on accident. More info in flash data sheet
*/
static inline void write_protection() {
  wait_for_ready_flag();
  set_control_pins(WRITE_PROTECT);  // Write Protection
}

/**
  @brief Code to read 1 frame (128 consecutive bytes) from flash
  @frameAddr: The address of the frame to read from (0 to 8,388,608)
  @readFrameBytes: A pass by reference array of 128 bytes to store the read data
  @_length: How many bytes to read (typically 128 to get the full frame)
*/
static inline void read_frame(uint32_t frameAddr, uint8_t *readFrameBytes, uint8_t _length) {
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
  @brief Code to write 1 frame (128 consecutive bytes) to the flash
  @frameAddr: The address of the frame to write to (0 to 8,388,608)
  @bytes: A pass by reference array of 128 bytes to write to the flash
*/
static inline void write_frame(uint32_t frameAddr, uint8_t *bytes) {
  wait_for_ready_flag();
  send_byte_to_flash(0x80, COMMAND_INPUT);
  send_addr_to_flash(frameAddr, 0);  // Address Input
  delay_ms(10); //was 1 ms but I think that needs decreasing
  for (int byteAddr = 0; byteAddr < 128; byteAddr++) {
    send_byte_to_flash(bytes[byteAddr], DATA_INPUT);
  }
  
  send_byte_to_flash(0x10, COMMAND_INPUT);
}

/**
  @brief A blocking function which will erase a block on the flash
  @blockAddr: 0 to 4095
*/
static inline void erase_block(uint32_t blockAddr) {
  wait_for_ready_flag();
  send_byte_to_flash(0x60, COMMAND_INPUT); //0x60 is erase command.
  send_block_addr_to_flash(blockAddr);
  send_byte_to_flash(0xD0, COMMAND_INPUT); 
  wait_for_ready_flag();  // Blocking Function
}

/**
  @brief A blocking function which will erase the entire flash (all 4096 blocks)
*/
static inline void erase_all(){
  printf("WARNING: ERASING ALL DATA (UNPLUG NAND FLASH TO ABORT)\r\n");

  // you have 10 seconds to unplug the nand flash
  for (int countDown = 10; countDown > 0; countDown--) {
    printf("ERASING DATA IN: ");
    printf("%i", countDown);
    printf(" Seconds\r\n");
    delay_ms(1000);
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
  @brief returns the larger of x1 and x2
*/
static inline uint16_t max(uint16_t x1, uint16_t x2){
  return (x1 > x2) ? x1 : x2;
}

/**
  @brief returns the smaller of x1 and x2
*/
static inline uint16_t min(uint16_t x1, uint16_t x2){
  return (x1 < x2) ? x1 : x2;
}

/**
  @brief returns the absolute difference of x1 and x2
*/
static inline uint16_t diff(uint16_t x1, uint16_t x2) {
  return (uint16_t)abs((int)((int)x1 - (int)x2));
}

/**
  @brief Function which searches for next available block and returns the first frame address of that block
  @return how many frames were writte (e.g. 0 means flash is empty)
*/
static inline uint32_t get_next_available_frame_addr() {
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
  @brief Initialisation function to set pin modes and get the next free frame on the flash (saving this in frameAddressPointer)
*/
static inline void init_flash() {
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
  printf("FRAME ADDRESS POINTER %i.\r\n", frameAddressPointer);

  printf("%d\r\n", read_flash_ID());

  if (read_flash_ID() != 0){
    printf("Flash Working Correctly\r\n");
  }
}

// --------------- ERROR CORRECTION CODE BELOW -----------------

/**
  @brief Calculates CRC16-CCITT Checksum
  @return CRC16-CCITT Checksum
*/
static inline uint16_t calculate_CRC(uint8_t* data, uint8_t length) {
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
static inline void hash(uint8_t *_input, uint8_t *_output) {
  _memset(_output, 0, 120);

  for (int i = 0; i < 8*120; i++) {
    int j = ((i%120)*8) + (i/120);
    _output[i/8] |= get_bit_arr(_input, j) << (7-(i%8));
  }
}

/**
  @brief returns true if x is a power of 2, else false
  @return
*/
static inline bool is_power_of_two(int x) {
    return (x != 0) && ((x & (x - 1)) == 0);
}

/**
  @brief Calculate parity bits for a given encoded data frame
*/
static inline void calculate_parity_bits(uint8_t *_input, uint8_t *_output) {
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

/**
  @brief Hamming and CRC Encoding
  @return bytes
*/
static inline void encode_parity(FrameArray dataFrame, uint8_t *bytes) {
  zip(dataFrame, bytes);

  uint8_t parities[8];
  calculate_parity_bits(bytes, parities);
  for (int i = 0; i < 8; i++) {
    bytes[118+i] = parities[i];
  }
  uint16_t CRC_Check = calculate_CRC(bytes, 126);
  bytes[126] = (uint8_t)((CRC_Check >> 8) & 0xFF);
  bytes[127] = (uint8_t)(CRC_Check & 0xFF);
}

/**
  @brief Prints the capacity left on the flash to the terminal
*/
static inline void print_capacity_info() {
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
  @brief Writes a single FrameArray to the next available space on the flash
*/
static inline int log_frame(FrameArray _input) {
  //printf("LOGFRAME addr ");
  //printfln(frameAddressPointer);

  // frameArray to array of bytes; 8388607 is 2Gb end
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
  @brief Outputs the frame array at the address frameAddr
*/
static inline FrameArray recall_frame(uint32_t frameAddr) {
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
  @brief Outputs all data in byte format
*/
static inline void read_all_raw(){
  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(&array, 0, 128);
  
  bool skipBlank = true;

  for(uint32_t i = 0; i < lastFrameToRead; i++) {

    //check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      printf("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;

    }else{
      //read as a uint8_t array
      read_frame(i, &array, 128);
      print_frame(array);
    } 
  }
}

/**
  @brief Outputs all data in frame format
*/
static inline void read_all_frame(){
  FrameArray _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(&array, 0, 128);
  
  bool skipBlank = true;

  for(uint32_t i = 0; i < lastFrameToRead; i++) {

    //check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      printf("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;

    }else{
      //rpint out as a frame
      _output = recall_frame(i);
       printf("FN:%i\r\n", i);
      print_frame_array(_output);
    } 
  }
}

/**
  @brief Outputs all data in frame format
*/
static inline void read_all_csv(){
  FrameArray _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(&array, 0, 128);
  
  bool skipBlank = true;
  print_csv_header();

  for(uint32_t i = 0; i < lastFrameToRead; i++) {

    //check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank){
      printf("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;

    }else{
      //rpint out as a frame
      _output = recall_frame(i);
      //printf("FN:%i\r\n", i);
      print_frame_csv(_output);
    } 
  }
}

/**
  @brief Reads the entire flash and returns the info on the capacity of the flash and the amount of corruption (checks CRC and Hamming codes)
*/
static inline void read_all(){
  FrameArray _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();

  uint8_t _check = 0;
  
  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int data_empty = 0;

  uint8_t array[128];
  _memset(&array, 0, 128);

  for(uint32_t i = 0; i < lastFrameToRead; i++) {
    read_frame(i, &_check, 1);
    if (_check == 0xFF && false){
      printf("End of data in block.\r\n");
      i += 2048; //move to the next block
      i = i - (i%2048) - 1;

    }else{
      //read as a uint8_t array
      read_frame(i, &array, 128);
      print_frame(array);

      //read as a FrameArray
      /*
      _output = recall_frame(i);
       printf("FN:%i\r\n", i);
      print_frame_array(_output);
      */
      //output in more useful format, as bits of data not just bytes.
      
    }

    // TODO: _output is a FrameArray convert to csv

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
  }
  
  printf("----------------------------------------------\r\n");
  print_capacity_info();
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

#endif /* NAND_DRIVER_H */
