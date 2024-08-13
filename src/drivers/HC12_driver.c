/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Jack Kendall
    Created on: 01 August 2024
    Description: Driver file for the HC-12 transceiver https://www.allaboutcircuits.com/projects/understanding-and-implementing-the-hc-12-wireless-transceiver-module/

    ---------------------------------------
*/

#include "HC12_driver.h"
#include "mcu.h"
#include <string.h>

void HC12_init(USART_TypeDef *uart) {
    LOG("Initialising Pad Radio...\r\n");
    uart_read_ready(uart); // Ready to receive

    gpio_write(SET_PIN, LOW); // Set pin low to enter command mode
    uart_write_buf(uart, RETURN_ALL_PARAM, sizeof(RETURN_ALL_PARAM)/sizeof(uint8_t)); // Send command to return parameters
    LOG("HC-12 Parameters received...\r\n");
    char params = uart_read_byte(uart); // Read result and save to var
    gpio_write(SET_PIN, HIGH); // Set pin high to exit command mode

    LOG("===== PAD RADIO SETTINGS =====\r\n");
    LOG(params); // Send params to log
    char txt = "Pad Radio Settings:";
    uart_write_buf(uart, txt, sizeof(txt)/sizeof(uint8_t)); // Send text to ground
    uart_write_buf(uart, params, sizeof(params)/sizeof(uint8_t)); // Send params to ground
}

void HC12_transmit(USART_TypeDef *uart, char *data) {
    uart_write_buf(uart, data, strlen(data)); // Send the actual length of the string
}

// Write function for receiving from Ground Station Here
