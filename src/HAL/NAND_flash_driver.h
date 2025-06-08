/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Thomas Groom, Alexnadra Posta
  Created on: 09 March 2023
  Description: NAND Flash driver
*/

#pragma once

#ifndef NAND_DRIVER_H
#define NAND_DRIVER_H


#include "lib/log.h"
#include "mcu.h"
#include "frame.h"
#include "debug.h"

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
#define DELAY_PINMODE 5

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
#define DATA0 PIN('B', 1)
#define DATA1 PIN('B', 2) // 3
#define DATA2 PIN('E', 7) // 4
#define DATA3 PIN('E', 8) // 5 
#define DATA4 PIN('E', 9) // 11
#define DATA5 PIN('E', 10) // 12
#define DATA6 PIN('E', 11) // 8
#define DATA7 PIN('E', 12) // 9

// Control Pins - pins used to control the state of the flash chip
#define WP  PIN('B', 0) // 13, Write Protection
#define WE  PIN('C', 5) // 14, Write Enable
#define ALE PIN('C', 4) // 15, Address latch enable (where in the memory to store)
#define CLE PIN('A', 7) // 16, Command latch enable (When on, you can sent command)
#define CE  PIN('A', 6) // 17, Check Enable (in case we want to test separatly)
#define RE  PIN('A', 5) // 18, Read Enable
#define RB  PIN('A', 4) // 19, Ready/Busy

// Stores the address of the next available frame (set of 128 bytes) (assumes all frames prior to this are full of valuable data)
// This variable is set by the get_next_available_frame_addr() function
extern uint32_t frameAddressPointer;

// Set all pins as gpio outputs by default
extern uint8_t globalPinMode;

/**
  @brief: This function returns only the specified bit from an array of bytes
  @param arr: an array of bytes (uint8)
  @param pos: which bit in the array of bytes to access (msb: 0 to lsb: 8*length(arr)-1)
  @return: the value of the bit at position "pos" in the byte array "arr"
*/
bool get_bit_arr(uint8_t *arr, int pos);

/**
  @brief This function returns the specific bit within a byte
  @param byte: the input byte
  @param pos: the position of the bit in question (msb: 0 to lsb: 7)
  @return: The value of the "pos" bit in the byte
*/
bool get_bit(uint8_t byte, int pos);

/**
  @brief Converts a Frame struct to an array of bytes
  @param unzippedData: the frame array object to zip
  @param zippedData: a pass by reference to the byte array where the output is stored
  @return: None
*/
void zip(Frame unzippedData, uint8_t *zippedData);

/**
  @brief Converts a byte array to a Frame struct
  @param zippedData: a pass by reference to the byte array to be converted
  @return: the Frame
*/
Frame unzip(uint8_t *zippedData);

/**
  @brief Prints a byte in binary format
  @param myByte: byte to be printed
*/
void print_byte(uint8_t myByte);

/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/
void print_frame(uint8_t dataArray[]);

/**
  @brief Prints a byte in hex format
  @param dataArray: frame to be printed
*/	
void print_frameHex(uint8_t dataArray[]);

/**
  @brief Allocates the space for the frame and fills it with 0
  @param arr: array to be filled
  @param val: value to fill array with
  @param num: number of elements to fill
  @note in C, you only define a pointer and won't allocate the bytes (as C++),
  you need to do it manually. We also need to fill the array with 0s
*/
void _memset(uint8_t *arr, uint8_t val, int num);

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
void print_frame_array(Frame frameFormat);

/**
  @brief Prints to serial in a readable way
  @param frameFormat
*/
void print_csv_header();

/**
  @brief Wait for the ready flag to be set
*/
void wait_for_ready_flag();

/**
  @brief Set all pins to the global pin mode (either GPIO_MODE_INPUT or GPIO_MODE_OUTPUT)
*/
void set_pin_modes();

/**
  @brief Set the control pins based on the input byte (i.e. COMMAND_INPUT, DATA_INPUT)
*/
void set_control_pins(uint8_t controlRegister);

/**
  @brief Set the data pins to the desired input byte (can be data, control or address information)
*/
void set_data_pins(uint8_t Byte);

/**
  @brief Write a single byte to the flash with control pins set to the "mode" byte and data pins set to the "cmd" byte (can be data, command or address byte)
  @param cmd: composed of the data pins
  @param mode: composed of the control pins
*/
void send_byte_to_flash(uint8_t cmd, uint8_t mode);

/**
  @brief Read a single byte from the flash (assumes address to read from has been set before calling this function)
*/
uint8_t receive_byte_from_flash();

/**
  @brief sends the 5-byte-address to the nand flash using the frame and byte address as input
  @note 8,388,608 frames each with 128 bytes. frameAddr has 23 valid bits. byteAddr has 7 valid bits
  @param frameAddr: The address of the frame to write/read to/from (0 to 8,388,608)
  @param byteAddr: The address of which byte to write/read to/from within the frame (0 - 127) (typically 0 as we want to start writing/reading from the first byte of a frame)
*/
void send_addr_to_flash(uint32_t frameAddr, uint8_t byteAddr);

/**
  @brief Sets the address to the first byte of the specified block
  @param blockAddr: Which block to set the address to (0 - 4095)
*/
void send_block_addr_to_flash(uint32_t blockAddr);

/**
  @brief Read the status register from the nand flash. Same as the RB (Read/Busy) input pin
  @return The status register of the flash
*/
uint8_t read_flash_status();

/**
  @brief Read the ID register from the nand flash (not unique to each nand flash?)
  @return ID register of the nand flash
*/
uint64_t read_flash_ID();

/**
  @brief Enable the flash write protection to prevent writing on accident. More info in flash data sheet
*/
void write_protection();

/**
  @brief Code to read 1 frame (128 consecutive bytes) from flash
  @frameAddr: The address of the frame to read from (0 to 8,388,608)
  @readFrameBytes: A pass by reference array of 128 bytes to store the read data
  @_length: How many bytes to read (typically 128 to get the full frame)
*/
void read_frame(uint32_t frameAddr, uint8_t *readFrameBytes, uint8_t _length);

/**
  @brief Code to write 1 frame (128 consecutive bytes) to the flash
  @frameAddr: The address of the frame to write to (0 to 8,388,608)
  @bytes: A pass by reference array of 128 bytes to write to the flash
*/
void write_frame(uint32_t frameAddr, uint8_t *bytes);

/**
  @brief A blocking function which will erase a block on the flash
  @blockAddr: 0 to 4095
*/
void erase_block(uint32_t blockAddr);

/**
  @brief A blocking function which will erase the entire flash (all 4096 blocks)
*/
void erase_all();

/**
  @brief returns the larger of x1 and x2
*/
uint16_t max(uint16_t x1, uint16_t x2);

/**
  @brief returns the smaller of x1 and x2
*/
uint16_t min(uint16_t x1, uint16_t x2);

/**
  @brief returns the absolute difference of x1 and x2
*/
uint16_t diff(uint16_t x1, uint16_t x2);

/**
  @brief Function which searches for next available block and returns the first frame address of that block
  @return how many frames were writte (e.g. 0 means flash is empty)
*/
uint32_t get_next_available_frame_addr();

/**
  @brief Initialisation function to set pin modes and get the next free frame on the flash (saving this in frameAddressPointer)
*/
void init_flash();

// --------------- ERROR CORRECTION CODE BELOW -----------------

/**
  @brief Calculates CRC16-CCITT Checksum
  @return CRC16-CCITT Checksum
*/
uint16_t calculate_CRC(uint8_t* data, uint8_t length);

/**
  @brief Hamming code hashing
*/
void hash(uint8_t *_input, uint8_t *_output);

/**
  @brief returns true if x is a power of 2, else false
  @return
*/
bool is_power_of_two(int x);

/**
  @brief Calculate parity bits for a given encoded data frame
*/
void calculate_parity_bits(uint8_t *_input, uint8_t *_output);

/**
  @brief Hamming and CRC Encoding
  @return bytes
*/
void encode_parity(Frame dataFrame, uint8_t *bytes);

/**
  @brief Prints the capacity left on the flash to the terminal
*/
void print_capacity_info();

/**
  @brief Writes a single Frame to the next available space on the flash
*/
int log_frame(Frame _input);

/**
  @brief Outputs the frame array at the address frameAddr
*/
Frame recall_frame(uint32_t frameAddr);

/**
  @brief Outputs all data in byte format
*/
void read_all_raw();

/**
  @brief Outputs all data in frame format
*/
void read_all_frame();

/**
  @brief Outputs all data in frame format
*/
void read_all_csv();

/**
  @brief Reads the entire flash and returns the info on the capacity of the flash and the amount of corruption (checks CRC and Hamming codes)
*/
void read_all();

/**
  @brief Routine to erase entire NAND flag
  @note WARNING: Deletes all data, permanently, be certain you want to use this.
*/
void NAND_flash_erase();

/**
  @brief Routine to test NAND Flash reading and writing.
*/
void NAND_flash_read();

#endif /* NAND_DRIVER_H */
