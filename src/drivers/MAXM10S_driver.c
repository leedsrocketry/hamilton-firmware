/*
	Leeds University Rocketry Organisation - LURA
  Author Name: 
  Created on: 
  Description: Driver file for the GNSS module MAX-M10S-00B (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include <stdarg.h>
#include "MAXM10S_driver.h"

// Disable all NMEA messages
static const uint8_t DISABLE_NMEA1[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
static const uint8_t DISABLE_NMEA2[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24};
static const uint8_t DISABLE_NMEA3[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x25};
static const uint8_t DISABLE_NMEA4[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x26};
static const uint8_t DISABLE_NMEA5[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x27};
static const uint8_t DISABLE_NMEA6[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x28};

// Set navigation rate to 10 Hz (100 ms)
static const uint8_t SET_RATE[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};

// Configure output protocol to UBX only
static const uint8_t SET_UBX_ONLY[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x44};

// Enable UBX-NAV-PVT message
static const uint8_t ENABLE_UBX_NAV_PVT[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};

// Save configuration
static const uint8_t SAVE_CONFIG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};

// Reset to default settings
static const uint8_t RESET_TO_DEFAULT[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 
                                           0xFF, 0xFF, 0xFF, 0xFF, // Clear mask (0xFFFF)
                                           0x00, 0x00, 0x00, 0x00, // Save mask
                                           0xFF, 0xFF, 0xFF, 0xFF, // Load mask (0xFFFF)
                                           0x17, 0x31, 0xBF};      // Checksum

static const uint8_t SOFT_RESET[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x66};


uint8_t MAXM10S_init(USART_TypeDef *uart)
{
    // uart_write_buf(uart, DISABLE_NMEA1, sizeof(DISABLE_NMEA1));
    // uart_write_buf(uart, DISABLE_NMEA2, sizeof(DISABLE_NMEA2));
    // uart_write_buf(uart, DISABLE_NMEA3, sizeof(DISABLE_NMEA3));
    // uart_write_buf(uart, DISABLE_NMEA4, sizeof(DISABLE_NMEA4));
    // uart_write_buf(uart, DISABLE_NMEA5, sizeof(DISABLE_NMEA5));
    // uart_write_buf(uart, DISABLE_NMEA6, sizeof(DISABLE_NMEA6));
    
    // uart_write_buf(uart, SET_RATE, sizeof(SET_RATE));
    // uart_write_buf(uart, SET_UBX_ONLY, sizeof(SET_UBX_ONLY));
    // uart_write_buf(uart, ENABLE_UBX_NAV_PVT, sizeof(ENABLE_UBX_NAV_PVT));
    // uart_write_buf(uart, SAVE_CONFIG, sizeof(SAVE_CONFIG));

    uart_write_buf(uart, RESET_TO_DEFAULT, sizeof(RESET_TO_DEFAULT)/sizeof(uint8_t));

    uart_write_buf(uart, SOFT_RESET, sizeof(SOFT_RESET));

    return 0; 
}

  // char* cmd = "$PUBX,41,1,0007,0003,9600,0*10\r\n";
  // //uart_write_buf(uart, cmd, strlen(cmd));
  // uart_write_buf(uart, configUBX, sizeof(configUBX)/sizeof(uint8_t));
  // delay_ms(250);
  // uart_write_buf(uart, setNMEA410, sizeof(setNMEA410)/sizeof(uint8_t));
  // delay_ms(250);
  // uart_write_buf(uart, setGNSS, sizeof(setGNSS)/sizeof(uint8_t));
  // delay_ms(250);

  // // get ID
  // delay_ms(1000);
  // uart_write_buf(uart, getDeviceID, sizeof(getDeviceID) / sizeof(uint8_t));

  // uint8_t response[8]; // Assuming an 8-byte response, adjust if necessary
  // for (int i = 0; i < 8; i++) {
  //     response[i] = uart_read_byte(uart);
  // }

  // // Print the received bytes for debugging
  // printf("Device ID Response: ");
  // for (int i = 0; i < 8; i++) {
  //     printf("%02X ", response[i]);
  // }
  // printf("\r\n");

  // for(;;)
  // {
  //   //uint8_t t = uart_read_byte(uart);
  //   // uint8_t t = (uint8_t)(uart->RDR & 255);
  //   // LOG("%02X\r\n", t);

  //   // uint8_t* results;
  //   // uart_read_buf(uart, results, 8);

  //   //LOG("%d\r\n", uart_read_ready(uart));


  //   // char c = uart_read_byte(uart);
  //   // while(c != '$')
  //   // {
  //   //   LOG("%c", c);
  //   //   c = uart_read_byte(uart);
  //   // }
    
  //   // LOG("\r\n");

  //   // LOG("%d\r\n", results[0]);
  // }

uint8_t MAXM10S_get_data(USART_TypeDef *uart)
{

}