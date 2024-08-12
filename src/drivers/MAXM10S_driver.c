/*
	Leeds University Rocketry Organisation - LURA
  Author Name: 
  Created on: 
  Description: Driver file for the GNSS module MAX-M10S-00B (https://www.mouser.co.uk/ProductDetail/u-blox/MAX-M10S-00B?qs=A6eO%252BMLsxmT0PfQYPb7LLQ%3D%3D)
*/

#include <stdarg.h>
#include "MAXM10S_driver.h"

static const uint8_t configUBX[]={0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x9A,0x79};
static const uint8_t setNMEA410[]={0xB5,0x62,0x06,0x17,0x14,0x00,0x00,0x41,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x75,0x57};
static const uint8_t setGNSS[]={0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x20,0x04,0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01,0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,0xDF,0xFB};

static const uint8_t getDeviceID[]={0xB5,0x62,0x27,0x03,0x00,0x00,0x2A,0xA5};

uint8_t MAXM10S_init(USART_TypeDef *uart)
{
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

  for(;;)
  {
    //uint8_t t = uart_read_byte(uart);
    // uint8_t t = (uint8_t)(uart->RDR & 255);
    // LOG("%02X\r\n", t);

    // uint8_t* results;
    // uart_read_buf(uart, results, 8);

    //LOG("%d\r\n", uart_read_ready(uart));


    char c = uart_read_byte(uart);
    while(c != '$')
    {
      LOG("%c", c);
      c = uart_read_byte(uart);
    }
    
    LOG("\r\n");

    // LOG("%d\r\n", results[0]);
  }

}

uint8_t MAXM10S_get_data(USART_TypeDef *uart)
{

}