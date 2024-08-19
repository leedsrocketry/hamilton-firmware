/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Jack Kendall
  Created on: 01 August 2024
  Description: header file for the HC-12 transceiver
*/

#ifndef HC12_DRIVER_H
#define HC12_DRIVER_H

#include "stm32l4r5xx.h"

#define LOW false
#define HIGH true

#define SET_PIN PIN('D', 7)

#define ENTER_COMMAND_MODE  "AT"
#define SET_BAUD            "AT+B" // + 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
#define SET_CHANNEL         "AT+C" // + e.g. 021 for Channel 21
#define SET_MODE            "AT+FU" // + 1, 2, 3
#define SET_POWER           "AT+P" // + 1,2,3,4,5,6,7,8
#define RETURN_PARAM        "AT+R" // + B, C, F, P
#define RETURN_ALL_PARAM    "AT+RX"
#define SET_DATA            "AT+U" // + data bits then check bits then stop bits -> N (no), O (odd), E (even), 1 (1 stop bit), 2 (2 stop bits), 3 (1.5 stop bits)
#define RETURN_VER          "AT+V"
#define SLEEP               "AT+SLEEP"
#define SET_DEFAULT_PARAM   "AT+DEFAULT"
#define UPDATE              "AT+UPDATE"

/**
	@brief Configures the settings for the HC12, and returns its params to the ground station
  @param uart UART which HC-12 is connected to
*/
void HC12_init(USART_TypeDef *uart);

/**
	@brief Transmits the passed data to the groundstation
  @param uart UART which HC-12 is connected to
  @param data data to send across the HC-12
*/
void HC12_transmit(USART_TypeDef *uart, char *data);

#endif