/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Tyler Green
  Created on: 15 December 2023
  Description: header file for the Pad Radio module SI446
*/


// couple of questions?????????
// what exactly is the spi write/read function we be using, how to link it here
// what radio information do we want - mode, freq, power, rssi
// how can I do the CTS checking?
// there are some quick access registers - do we need to set them up
// any update on the ATMega MCU attached to the gpio?
// any interupts wont be seen via spi - only on interuppt pin 


#ifndef SI446_DRIVER_H
#define SI446_DRIVER_H

#include "HAL/mcu.h"
#include "debug.h"

#pragma region Macros //_______________ MACROS _______________________

/** @name Constants */
#define SI446_CTS_TIME_OUT                                  20
#define SI446_CS								                            9

/** @name SI446 Machine States */
#define SI446_NO_CHANGE_STATE                               0
#define SI446_SLEEP_STATE                                   1
#define SI446_SPI_ACTIVE_STATE                              2
#define SI446_READY_STATE                                   3
#define SI446_NO_ANT_READY_STATE                            4
#define SI446_TX_TUNE_STATE                                 5
#define SI446_RX_TUNE_STATE                                 6
#define SI446_TX_STATE                                      7
#define SI446_RX_STATE                                      8

/** @name SI446 chip Status */
#define SI446_OK                                            0
#define SI446_E_NULL_PTR                                    -1
#define SI446_E_CTS_INVALID                                 -2
#define SI446_E_CTS_TIME_OUT                                -3       
#define SI446_E_BAD_COMMAND                                 -4
#define SI446_E_INVALID_CMD_ARGS                            -5
#define SI446_E_INVALID_CMD_ISSUE                           -6
#define SI446_E_FIFO_UNDERFLOW_OVERFLOW                     1 // not sure

/** @name Chip settings */
// found in "AN625: Si446x API Descriptions"




/** @name Commands */
// found in "AN625: Si446x API Descriptions"
#define SI446_POWER_UP_CMD                                  0x02
#define SI446_SET_PROPERTY_CMD                              0x11
#define SI446_GET_PROPERTY_CMD                              0x12
#define SI446_GET_CHIP_STATUS                               0x23
#define SI446_START_TX_CMD                                  0x31
#define SI446_START_RX_CMD                                  0x32
#define SI446_WRITE_TX_FIFO_CMD                             0x66
#define SI446_READ_RX_FIFO_CMD                              0x77
#define SI446_FIFO_INFO                                     0x15

/** @name properties registers */
//#define SI446_ 

#pragma endregion Macros

#pragma region Structs/Emun //_______________ STRUCTS _______________________

/**
 * @brief SI446 settings
 */
typedef struct SI446_settings
{

  // radio settings
  uint8_t SI446_channel;
  uint8_t SI446_RX_TIMEOUT_STATE; // state to go to if RX times out
  uint8_t SI446_RX_VALID_STATE; // next state to go to if RX successful
  uint8_t SI446_RX_INVALID_STATE; // state to go to if RX recivied is invalid

} SI446_settings;

#pragma endregion Structs/Emun

#pragma region Public //_________________ PUBLIC _________________
/**
  @brief Initialization of the SI446 module
  @note
  @return Success/Failure
*/
int8_t SI446_init(SPI_TypeDef *spi);

/**
  @brief loads the data into the FIFO ready for transmittion
  @note doesn't need to wait for CTS - data  can be read out straight away
  @param data ptr to the data which is to be sent
*/
int8_t SI446_write_data(uint8_t *data, size_t byteCount);

/**
  @brief reads recieved data from the FIFO buffer
  @note doesn't need to wait for CTS - data  can be read out straight away
  @param data ptr to the data which is to be sent
*/
int8_t SI446_read_data(uint8_t *data, size_t byteCount);

/**
  @brief send SI446 into recieve mode using current radio settings
  @note
*/
int8_t SI446_recieve();

/**
  @brief send SI446 into transmit mode using current radio settings
  @note
*/
int8_t SI446_transmit();

#pragma endregion Public


#pragma region Private  //_________________ PRIVATE _________________

/**
  @brief funtion to deal with recieving data from the sensor
  @note waits until CTS is correct then records data
  @param byteCount number of bytes to clock and read
  @param data ptr to the data storage location
  @return Success/Failure
*/
int8_t SI446_get_response(int byteCount, uint8_t *data);

/**
  @brief checks if CTS bit is set
  @note e.g. used to check if SI446_command with no response has been correctly carried out
  @param desired -1: no time_out, 
                  0: default (SI446_CTS_TIME_OUT)
                 >0: custom time (in ms) to wait for CTS response.
  @return Success/Failure
*/
int8_t SI446_check_CTS(int desired);

/**
  @brief SI446_command: power up the sensor with specified function
  @return Success/Failure
*/
int8_t SI446_power_up();

#pragma endregion Private

#endif /* SI446_DRIVER_H */
