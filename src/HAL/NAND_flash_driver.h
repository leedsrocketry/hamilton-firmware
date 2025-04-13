/*
        Leeds University Rocketry Organisation - LURA
  Author Name: Thomas Groom, Alexnadra Posta
  Created on: 09 March 2023
  Description: NAND Flash driver
*/

#pragma once

#ifndef NAND_DRIVER_H
#define NAND_DRIVER_H

#include "debug.h"
#include "frame.h"
#include "mcu.h"
#include "lib/log.h"

// Params
#define NAND_STORAGE_CAPACITY 8388607  // 8MB

// Defines which are used when returning the status of a write to flash
#define NONE 0
#define SUCCESS 1
#define STORAGE_FULL_WARNING 2
#define STORAGE_FULL_ERROR 3

// Defines used in the status of frames which have been gone through error
// correction
#define DATA_INTACT 1
#define DATA_FIXED 2
#define DATA_CORRUPTED 3
#define EMPTY 4

// Defines a global delay mostly for debugging
#define DELAY 1
#define DELAY_PINMODE 5

// Manipulate control pins for commands
#define STANDBY 0b10001100            // CE# CLE ALE WE# RE# WP# X X
#define COMMAND_INPUT 0b01001100      // CE# CLE ALE WE# RE# WP# X X
#define ADDRESS_INPUT 0b00101100      // CE# CLE ALE WE# RE# WP# X X
#define DATA_INPUT 0b00001100         // CE# CLE ALE WE# RE# WP# X X
#define DATA_OUTPUT 0b00011100        // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT 0b00001000      // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT_OFF 0b00001100  // CE# CLE ALE WE# RE# WP# X X

// Defines used to toggle the WE or RE pins to latch a byte into the flash chip
#define WE_HIGH 0b00010000  // CE# CLE ALE WE# RE# WP# X X
#define RE_HIGH 0b00001000

// Structure used to break the address bits into block->page->column which have
// meanings defined by the flash chip
typedef struct Address {
  uint16_t block;   // 12 bits
  uint8_t page;     // 6 bits
  uint16_t column;  // 13 bits (12 used)
} Address;

// Data Pins - pins used in data/control/address transmission (8 bit parrallel
// bus)
static uint16_t data0 = PIN('B', 1);   // 2;
static uint16_t data1 = PIN('B', 2);   // 3;
static uint16_t data2 = PIN('E', 7);   // 4;
static uint16_t data3 = PIN('E', 8);   // 5;
static uint16_t data4 = PIN('E', 9);   // 11;
static uint16_t data5 = PIN('E', 10);  // 12;
static uint16_t data6 = PIN('E', 11);  // 8;
static uint16_t data7 = PIN('E', 12);  // 9;

// Control Pins - pins used to control the state of the flash chip
static uint16_t WP = PIN('B', 0);   // 13, Write Protection;
static uint16_t WE = PIN('C', 5);   // 14, Write Enable;
static uint16_t ALE = PIN('C', 4);  // 15, Address latch enable (where in the memory to store);
static uint16_t CLE = PIN('A', 7);  // 16, Command latch enable (When on, you can sent command);
static uint16_t CE = PIN('A', 6);   // 17, Check Enable (in case we want to test separatly);
static uint16_t RE = PIN('A', 5);   // 18, Read Enable;
static uint16_t RB = PIN('A', 4);   // 19, Ready/Busy;

// Stores the address of the next available frame (set of 128 bytes) (assumes
// all frames prior to this are full of valuable data) This variable is set by
// the get_next_available_frame_addr() function
static uint32_t frameAddressPointer = 0;

// Set all pins as gpio outputs by default
static uint8_t globalPinMode = GPIO_MODE_OUTPUT;

/**
  @brief: This function returns only the specified bit from an array of bytes
  @param arr: an array of bytes (uint8)
  @param pos: which bit in the array of bytes to access (msb: 0 to lsb:
  8*length(arr)-1)
  @return: the value of the bit at position "pos" in the byte array "arr"
*/
static inline bool get_bit_arr(uint8_t *arr, int pos) { return (bool)(arr[pos / 8] & (1 << (7 - (pos % 8)))); }

/**
  @brief This function returns the specific bit within a byte
  @param byte: the input byte
  @param pos: the position of the bit in question (msb: 0 to lsb: 7)
  @return: The value of the "pos" bit in the byte
*/
static inline bool get_bit(uint8_t byte, int pos) { return (bool)(byte & (1 << (7 - (pos % 8)))); }

/**
  @brief Converts a Frame struct to an array of bytes
  @param unzippedData: the frame array object to zip
  @param zippedData: a pass by reference to the byte array where the output is
  stored
  @return: None
*/
static inline void zip(Frame unzippedData, uint8_t *zippedData) {
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
  zippedData[i++] = (uint8_t)((unzippedData.accel.x >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accel.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accel.y >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.accel.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.accel.z >> 8) & 0xFF);
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
  zippedData[i++] = (uint8_t)(unzippedData.bme.pressure & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.temperature >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.bme.temperature & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.bme.humidity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.bme.humidity & 0xFF);

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
  @brief Converts a byte array to a Frame struct
  @param zippedData: a pass by reference to the byte array to be converted
  @return: the Frame
*/
static inline Frame unzip(uint8_t *zippedData) {
  Frame _unzippedData;
  _unzippedData.changeFlag = 1;
  int i = -1;

  // Resolve errors to do with uninitialised data, but this is obviously not
  // correct.
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

/**
  @brief Prints a byte in binary format
  @param myByte: byte to be printed
*/
static inline void print_byte(uint8_t myByte) {
  LOG("0b");
  for (int i = 7; i >= 0; i--) {
    LOG("%i", (myByte >> i) & 0b1);
  }
  LOG("\r\n");
}

/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/
static inline void print_frame(uint8_t dataArray[]) {
  LOG("u");
  for (int i = 0; i < 128; i++) {
    LOG("%X,", dataArray[i]);  // %02X adds a paddings of 0
  }
  LOG("\r\n");
}

/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/
static inline void print_frameHex(uint8_t dataArray[]) {
  for (int i = 0; i < 128; i++) {
    if (dataArray[i] < 16) {
      LOG("0");
    }
    LOG("%i", dataArray[i]);
    LOG(", ");
  }
  LOG("\r\n");
}

/**
  @brief Allocates the space for the frame and fills it with 0
  @param arr: array to be filled
  @param val: value to fill array with
  @param num: number of elements to fill
  @note in C, you only define a pointer and won't allocate the bytes (as C++),
  you need to do it manually. We also need to fill the array with 0s
*/
static inline void _memset(uint8_t *arr, uint8_t val, int num) {
  for (int i = 0; i < num; i++) {
    arr[i] = val;
  }
}

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_frame_array(Frame frameFormat) {
  LOG("Date: %i, %i:%i:%i:%i:\r\n", frameFormat.date.year, frameFormat.date.minute, frameFormat.date.second,
      frameFormat.date.millisecond, frameFormat.date.microsecond);

  LOG("ChangeFlag: %u\r\n", frameFormat.changeFlag);

  LOG("Accel HG: \tX: %i,\tY: %i,\tZ: %i\t\r\n", frameFormat.accel.x, frameFormat.accel.y, frameFormat.accel.z);

  LOG("IMU: \tX Rate: %ld,\tY Rate: %ld,\tZ Rate: %ld,\tX Offset: %ld,\tY Offset: %ld, \
        \tZ Offset: %ld,\tX Accel: %d,\tY Accel: %d,\tZ Accel: %d,\r\n",
      frameFormat.imu.x_rate, frameFormat.imu.y_rate, frameFormat.imu.z_rate, frameFormat.imu.x_offset,
      frameFormat.imu.y_offset, frameFormat.imu.z_offset, frameFormat.imu.x_accel, frameFormat.imu.y_accel,
      frameFormat.imu.z_accel);

  LOG("Barometer: \ttemp: %ld, \tpressure: %ld\r\n", frameFormat.barometer.temp, frameFormat.barometer.pressure);
  LOG("GNSS: \tLat: %d,\tLong: %d,\tAlt: %d,\tVel: %d\r\n", frameFormat.GNSS.latitude, frameFormat.GNSS.longitude,
      frameFormat.GNSS.altitude, frameFormat.GNSS.velocity);

  LOG("BME280: \tPressure: %ld,\tTemperature: %d,\tHumidity: %ld\r\n", frameFormat.bme.pressure,
      frameFormat.bme.temperature, frameFormat.bme.humidity);
}

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
static inline void print_csv_header() {
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
  LOG("%i:%i:%i:%i,", frameFormat.date.minute, frameFormat.date.second, frameFormat.date.millisecond,
      frameFormat.date.microsecond);

  LOG("%i,", frameFormat.changeFlag);

  LOG("%i,%i,%i,", frameFormat.accel.x, frameFormat.accel.y, frameFormat.accel.z);

  LOG("%li,%li,%li,%li,%li,%li,%i,%i,%i,", frameFormat.imu.x_rate, frameFormat.imu.y_rate, frameFormat.imu.z_rate,
      frameFormat.imu.x_offset, frameFormat.imu.y_offset, frameFormat.imu.z_offset, frameFormat.imu.x_accel,
      frameFormat.imu.y_accel, frameFormat.imu.z_accel);

  LOG("%li,%li,", frameFormat.barometer.temp, frameFormat.barometer.pressure);
  LOG("%i,%i,%i,%i,", frameFormat.GNSS.latitude, frameFormat.GNSS.longitude, frameFormat.GNSS.altitude,
      frameFormat.GNSS.velocity);
  LOG("%li,%i,%li,", frameFormat.bme.pressure, frameFormat.bme.temperature, frameFormat.bme.humidity);
  LOG("\r\n");
}

/**
  @brief Wait for the ready flag to be set
*/
static inline void wait_for_ready_flag() {
  int count = 1000 * 10;  // Try for 1 second before giving error
  while (gpio_read(RB) == LOW && count > 0) {
    delay_microseconds(1);
    count--;
  }
  if (count < 1) {
    // LOG("waitForReadyFlag: TIMEOUT\r\n");
  }
}

/**
  @brief Set all pins to the global pin mode (either GPIO_MODE_INPUT or
  GPIO_MODE_OUTPUT)
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
  delay_microseconds(DELAY_PINMODE);
}

/**
  @brief Set the control pins based on the input byte (i.e. COMMAND_INPUT,
  DATA_INPUT)
*/
static inline void set_control_pins(uint8_t controlRegister) {  // CE# CLE ALE WE# RE# WP#
  gpio_write(CE, get_bit(controlRegister, 0));
  gpio_write(CLE, get_bit(controlRegister, 1));
  gpio_write(ALE, get_bit(controlRegister, 2));
  gpio_write(WE, get_bit(controlRegister, 3));
  gpio_write(RE, get_bit(controlRegister, 4));
  gpio_write(WP, get_bit(controlRegister, 5));
}

/**
  @brief Set the data pins to the desired input byte (can be data, control or
  address information)
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
  @brief Write a single byte to the flash with control pins set to the "mode"
  byte and data pins set to the "cmd" byte (can be data, command or address
  byte)
  @param cmd: composed of the data pins
  @param mode: composed of the control pins
*/
static inline void send_byte_to_flash(uint8_t cmd, uint8_t mode) {
  // delay_ms(DELAY); // include if needed
  set_control_pins(mode);
  set_data_pins(cmd);
  // delay_ms(DELAY);
  set_control_pins(mode | WE_HIGH);  // lanch what is in the data bus in the memory
  // delay_ms(DELAY);
}

/**
  @brief Read a single byte from the flash (assumes address to read from has
  been set before calling this function)
*/
static inline uint8_t receive_byte_from_flash() {
  delay_microseconds(DELAY);
  set_control_pins(DATA_OUTPUT);
  delay_microseconds(DELAY);
  set_control_pins(DATA_OUTPUT & (~RE_HIGH));  // setting RE LOW
  delay_microseconds(DELAY);

  if (globalPinMode != GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_INPUT;
    set_pin_modes();
  }

  uint8_t data = (gpio_read(data7) << 7) | (gpio_read(data6) << 6) | (gpio_read(data5) << 5) | (gpio_read(data4) << 4) |
                 (gpio_read(data3) << 3) | (gpio_read(data2) << 2) | (gpio_read(data1) << 1) | (gpio_read(data0) << 0);
  return data;
}

/**
  @brief sends the 5-byte-address to the nand flash using the frame and byte
  address as input
  @note 8,388,608 frames each with 128 bytes. frameAddr has 23 valid bits.
  byteAddr has 7 valid bits
  @param frameAddr: The address of the frame to write/read to/from (0 to
  8,388,608)
  @param byteAddr: The address of which byte to write/read to/from within the
  frame (0 - 127) (typically 0 as we want to start writing/reading from the
  first byte of a frame)
*/
static inline void send_addr_to_flash(uint32_t frameAddr, uint8_t byteAddr) {
  Address addr = {(frameAddr >> 11) & 0b0000111111111111,                      // block
                  (frameAddr >> 5) & 0b00111111,                               // page
                  ((frameAddr & 0b00011111) << 7) | (byteAddr & 0b01111111)};  // column

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
  @brief Read the status register from the nand flash. Same as the RB
  (Read/Busy) input pin
  @return The status register of the flash
*/
static inline uint8_t read_flash_status() {
  wait_for_ready_flag();
  send_byte_to_flash(0x70, COMMAND_INPUT);
  return receive_byte_from_flash();
}

/**
  @brief Read the ID register from the nand flash (not unique to each nand
  flash?)
  @return ID register of the nand flash
*/
static inline uint64_t read_flash_ID() {
  uint64_t id = 0;

  wait_for_ready_flag();
  send_byte_to_flash(0x90, COMMAND_INPUT);
  send_byte_to_flash(0x00, ADDRESS_INPUT);

  for (int i = 0; i < 5; i++) {
    uint8_t byte = receive_byte_from_flash();
    id |= byte << (4 - i);
  }
  return id;
}

/**
  @brief Enable the flash write protection to prevent writing on
  accident. More info in flash data sheet
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
  delay_microseconds(10);
  set_control_pins(DATA_INPUT);
  for (int byteAddr = 0; byteAddr < 128; byteAddr++) {
    gpio_write(WE, 0);
    set_data_pins(bytes[byteAddr]);
    gpio_write(WE, 1);
  }
  send_byte_to_flash(0x10, COMMAND_INPUT);
}

/**
  @brief A blocking function which will erase a block on the flash
  @blockAddr: 0 to 4095
*/
static inline void erase_block(uint32_t blockAddr) {
  wait_for_ready_flag();
  send_byte_to_flash(0x60, COMMAND_INPUT);  // 0x60 is erase command.
  send_block_addr_to_flash(blockAddr);
  send_byte_to_flash(0xD0, COMMAND_INPUT);
  wait_for_ready_flag();  // Blocking Function
}

/**
  @brief A blocking function which will erase the entire flash (all 4096 blocks)
*/
static inline void erase_all() {
  LOG("WARNING: ERASING ALL DATA (UNPLUG NAND FLASH TO ABORT)\r\n");

  // you have 10 seconds to unplug the nand flash
  for (int countDown = 5; countDown > 0; countDown--) {
    LOG("ERASING DATA IN: ");
    LOG("%i", countDown);
    LOG(" Seconds\r\n");
    delay_ms(1000);
  }

  for (uint32_t block = 0; block < 64 * 4096; block++) {
    erase_block(block);
    if (block % 5000 == 0) {
      LOG("ERASING [");
      for (int i = 0; i < 50; i++) {
        if (i < block / (64 * 4096 * 0.01 * 2)) {
          LOG("#");
        } else {
          LOG(" ");
        }
      }
      LOG("] - ");
      int percentage = (int)(block / (64 * 4096 * 0.01));
      LOG("%d", percentage);
      LOG("%%\r\n");
    }
  }
  LOG("ERASING COMPLETE \r\n");
}

/**
  @brief returns the larger of x1 and x2
*/
static inline uint16_t max(uint16_t x1, uint16_t x2) { return (x1 > x2) ? x1 : x2; }

/**
  @brief returns the smaller of x1 and x2
*/
static inline uint16_t min(uint16_t x1, uint16_t x2) { return (x1 < x2) ? x1 : x2; }

/**
  @brief returns the absolute difference of x1 and x2
*/
static inline uint16_t diff(uint16_t x1, uint16_t x2) { return (uint16_t)abs((int)((int)x1 - (int)x2)); }

/**
  @brief Function which searches for next available block and returns the first
  frame address of that block
  @return how many frames were writte (e.g. 0 means flash is empty)
*/
static inline uint32_t get_next_available_frame_addr() {
  uint16_t prevPointer = 4096;
  uint16_t pointer = prevPointer / 2;
  uint8_t _check = 0;

  for (int i = 0; i < 11; i++) {
    read_frame(max(pointer - 1, 0) * 64 * 32, &_check,
               1);  // Dosn't need to be the whole frame -------
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
  LOG("ms delay: ");
  LOG(DELAY + "\n");
  LOG("Incorrect Bytes: ");
  LOG(wrong + "\n");
  LOG("Total Bytes: ");
  LOG(total + "\n");
  LOG("Percent Incorrect: ");
  LOG("%f", ((float)wrong * 100) / (float)total);
  LOG(" %%\n");
  LOG("Time took to write: ");
  LOG(duration / 1000);
  LOG(" ms\n");
  LOG("Time waiting while nand flash was busy: ");
 // LOG(count / 1000);
  LOG(" ms\n");
  LOG("Next available frame addr: ");
  LOG(getNextAvailableFrameAddr() + "\n");
}
*/

/**
  @brief Initialisation function to set pin modes and get the
  next free frame on the flash (saving this in frameAddressPointer)
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
  gpio_set_mode(CE, GPIO_MODE_OUTPUT);
  gpio_set_mode(RE, GPIO_MODE_OUTPUT);
  gpio_set_mode(WE, GPIO_MODE_OUTPUT);
  gpio_set_mode(WP, GPIO_MODE_OUTPUT);

  gpio_set_mode(RB, GPIO_MODE_INPUT);

  frameAddressPointer = get_next_available_frame_addr();
  // LOG("FRAME ADDRESS POINTER %li.\r\n", frameAddressPointer);

  // LOG("%lld\r\n", read_flash_ID());

  if (read_flash_ID() != 0) {
    logi("Flash Working Correctly\n");
  }
}

// --------------- ERROR CORRECTION CODE BELOW -----------------

/**
  @brief Calculates CRC16-CCITT Checksum
  @return CRC16-CCITT Checksum
*/
static inline uint16_t calculate_CRC(uint8_t *data, uint8_t length) {
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

  for (int i = 0; i < 8 * 120; i++) {
    int j = ((i % 120) * 8) + (i / 120);
    _output[i / 8] |= get_bit_arr(_input, j) << (7 - (i % 8));
  }
}

/**
  @brief returns true if x is a power of 2, else false
  @return
*/
static inline bool is_power_of_two(int x) { return (x != 0) && ((x & (x - 1)) == 0); }

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

  // Initialise variables
  uint8_t _word[15];
  uint8_t parities = 0;
  uint8_t parity = 0;
  int k = 0;

  for (int _set = 0; _set < 8; _set++) {
    for (int i = 0; i < 15; i++) {
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
      for (int j = 0; j < 128; j++) {  // j from 0 - 128
        if (j + 1 != 1 && is_power_of_two(j + 1) == 0) {
          if (bit_pos & (j + 1)) {
            parity ^= (_word[k / 8] >> (k % 8)) & 1;
          }
          k++;
        }
      }
      parities |= parity << (i % 8);
    }
    _output[_set] = parities;
  }
}

/**
  @brief Hamming and CRC Encoding
  @return bytes
*/
static inline void encode_parity(Frame dataFrame, uint8_t *bytes) {
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

/**
  @brief Prints the capacity left on the flash to the terminal
*/
static inline void print_capacity_info() {
  uint32_t lastFrameUsed = get_next_available_frame_addr();
  LOG("Used: ");
  uint32_t usedInMB = (lastFrameUsed * 128) / 1000000;
  LOG("%i", (int)usedInMB);
  LOG(" MB (");
  LOG("%f", lastFrameUsed / (4096 * 64 * 32 * 0.01));
  LOG("%%) | ");
  LOG("Estimated Time Remaining (hh:mm:ss): ");
  uint32_t hours = ((4096 * 64 * 32) - lastFrameUsed) / (1000 * 60 * 60) % 24;
  uint32_t minutes = ((4096 * 64 * 32) - lastFrameUsed) / (1000 * 60) % 60;
  uint32_t seconds = (((4096 * 64 * 32) - lastFrameUsed) / (1000)) % 60;
  if (hours < 10) LOG("0");
  LOG("%i", (int)hours);
  LOG(":");
  if (minutes < 10) LOG("0");
  LOG("%i", (int)minutes);
  LOG(":");
  if (seconds < 10) LOG("0");
  LOG("%i", (int)seconds);
  LOG("\r\n");
}

/**
  @brief Writes a single Frame to the next available space on the flash
*/
static inline int8_t save_frame(Frame _input) {
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

/**
  @brief Outputs the frame array at the address frameAddr
*/
static inline Frame recall_frame(uint32_t frameAddr) {
  uint8_t encoded[128];
  _memset(encoded, 0, 128);
  Frame _output;
  read_frame(frameAddr, encoded, 128);
  _output = unzip(encoded);  // Don't bother decoding parity bits
  return _output;
}

/**
  @brief Outputs all data in byte format
*/
static inline void read_all_raw() {
  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);

  bool skipBlank = true;

  for (uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank) {
      LOG("End of data in block.\r\n");
      i += 2048;  // move to the next block
      i = i - (i % 2048) - 1;

    } else {
      // read as a uint8_t array
      read_frame(i, array, 128);
      print_frame(array);
    }
  }
}

/**
  @brief Outputs all data in frame format
*/
static inline void read_all_frame() {
  Frame _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);

  bool skipBlank = true;

  for (uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank) {
      LOG("End of data in block.\r\n");
      i += 2048;  // move to the next block
      i = i - (i % 2048) - 1;

    } else {
      // rpint out as a frame
      _output = recall_frame(i);
      LOG("FN:%li\r\n", i);
      print_frame_array(_output);
    }
  }
}

/**
  @brief Outputs all data in frame format
*/
static inline void read_all_csv() {
  Frame _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;

  uint8_t array[128];
  _memset(array, 0, 128);

  bool skipBlank = true;
  print_csv_header();

  for (uint32_t i = 0; i < lastFrameToRead; i++) {
    // check if frame has no data written to it
    read_frame(i, &_check, 1);
    if (_check == 0xFF && skipBlank) {
      LOG("End of data in block.\r\n");
      i += 2048;  // move to the next block
      i = i - (i % 2048) - 1;

    } else {
      // rpint out as a frame
      _output = recall_frame(i);
      // LOG("FN:%i\r\n", i);
      print_frame_csv(_output);
    }
  }
}

/**
  @brief Reads the entire flash and returns the info on the capacity of the
  flash and the amount of corruption (checks CRC and Hamming codes)
*/
static inline void read_all() {
  uint32_t lastFrameToRead = get_next_available_frame_addr();
  uint8_t _check = 0;
  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int data_empty = 0;

  uint8_t array[128];
  _memset(array, 0, 128);

  for (uint32_t i = 0; i < lastFrameToRead; i++) {
    read_frame(i, &_check, 1);
    if (_check == 0xFF && false) {
      LOG("End of data in block.\r\n");
      i += 2048;  // move to the next block
      i = i - (i % 2048) - 1;
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
  LOG("%i", (data_intact + data_fixed) / (4096 * 64 * (4096 / 128)));
  LOG("%%\r\n");
}

/**
  @brief Routine to erase entire NAND flag
  @note WARNING: Deletes all data, permanently, be certain you want to use this.
*/
static inline void NAND_flash_erase() {
  watchdog_pat();
  erase_all();
  while (1);
}

/**
  @brief Routine to test NAND Flash reading and writing.
*/
static inline void NAND_flash_read() {
  LOG("==================== Reading NAND FLASH ====================\r\n");
  read_all_csv();
  print_capacity_info();
}

#endif /* NAND_DRIVER_H */
