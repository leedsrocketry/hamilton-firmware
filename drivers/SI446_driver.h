/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 10 June 2023
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
#include "mcu.h"

#pragma region Macros //_______________ MACROS _______________________

/** @name Constants */
#define SI446_CTS_TIME_OUT                                  20

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

SPI_TypeDef SI446_SPI;

#pragma endregion Macros

#pragma region Structs/Emun //_______________ STRUCTS _______________________

typedef struct SI446_chip_settings
{
  uint32_t SI446_XO_FREQUENCY;
  uint8_t BOOT_OPTIONS;
  uint8_t BOOT_OPTIONS;
} SI446_chip_settings;

/**
 * @brief SI446 radio settings
 */
typedef struct SI446_Radio_settings
{
  uint8_t SI446_channel;
  uint8_t SI446_CURRENT_RSSI;

  uint8_t SI446_RX_TIMEOUT_STATE;
  uint8_t SI446_RX_VALID_STATE;
  uint8_t SI446_RX_INVALID_STATE;

} SI446_Radio_settings;

/**
 * @brief SI446 radio settings
 */
typedef struct SI446_Packet_settings
{
  uint8_t SI446_channel

} SI446_Packet_settings;

/**
 * @brief SI446 transmit data structure when rocket in flight ?
 */
typedef struct SI446_data
{
  uint8_t altitude;
  uint8_t batteryLevel;
  uint8_t altitude_barom;
  uint8_t gnss_x_coord;
  uint8_t gnss_y_coord;
  uint8_t gnss_z_coord;
} SI446_Packet_settings;

#pragma endregion Structs/Emun

#pragma region Public //_________________ PUBLIC _________________
/**
  @brief Initialization of the SI446 module
  @note
  @return Success/Failure
*/
int8_t init_SI446(SPI_TypeDef spi);

/**
  @brief loads the data into the FIFO ready for transmittion
  @note
  @param data ptr to the data which is to be sent
*/
void Write_data_SI446(uint8_t *data);

/**
  @brief reads recieved data from the FIFO buffer
  @note
  @param data ptr to the data which is to be sent
*/
void Read_data_SI446(uint8_t *data);

/**
  @brief send SI446 into recieve mode using current radio settings
  @note
*/
int8_t Recieve_SI446();

/**
  @brief send SI446 into transmit mode using current radio settings
  @note
*/
int8_t Transmit_SI446();


#pragma endregion Public


#pragma region Private  //_________________ PRIVATE _________________
/**
  @brief send a command to the SI4460 sensor
  @note doesnt deal with the response
  @param byteCount number of bytes in the command to be sent (includes command)
  @param data pointer to the command parameters which are to be sent
  @return Success/Failure
*/


#pragma endregion Private

#endif /* SI446_DRIVER_H */
