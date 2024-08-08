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

void HC12_init() {
    uart_read_ready(UART2); // Ready to receive
    int len = 0; // CHANGEME!!!!!!!!!!!

    gpio_write(SET_PIN, LOW); // Set pin low to enter command mode
    uart_write_buf(UART2, RETURN_ALL_PARAM, len); // Get all settings
    gpio_write(SET_PIN, HIGH); // Set pin high to exit command mode

    char params = uart_read_byte(UART2); // Read result and save to var

    printf(params); // print var
    LOG("===== PAD RADIO SETTINGS =====\r\n");
    // SEND SOMETHING SAYING PAD RADIO SETTINGS:!!!!!!!!!!!!!!!
    uart_write_buf(UART2, params, len); // Send params to ground
}

void HC12_transmit() {
    return 0;
}
