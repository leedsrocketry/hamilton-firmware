//
// Created by Morgan Thomas on 06/08/2024.
//

#ifndef HAMILTON_FIRMWARE_SD_CARD_H
#define HAMILTON_FIRMWARE_SD_CARD_H

#include "HAL/mcu.h"
// SD Card public functions

typedef struct {
  SPI_TypeDef *spi_interface;
  uint8_t chip_select;
  uint8_t version;
  uint8_t compatible;
  uint8_t initialised;
  uint8_t high_capacity;
} SD_Card;

SD_Card init_sd_card(SPI_TypeDef *spi_device, uint8_t chip_select);
uint8_t sd_write_block(SD_Card *sd_card, uint32_t address, uint8_t *data);
uint8_t sd_read_block(SD_Card *sd_card, uint32_t address, uint8_t *result_ptr);
void sd_send_command(SD_Card *sd_device, uint8_t command, uint32_t argument, uint8_t *response, uint8_t cs);
uint8_t cmd_resp_size(uint8_t command);
static inline void sd_transmit_receive(SPI_TypeDef *spi, uint8_t *send_byte, uint8_t transmit_size,
                                       uint8_t receive_size, uint8_t *result_ptr, uint8_t wait_cmds);

// SD Commands
#define CMD0 0b00000000
#define CMD1 0b00000001
#define CMD8 0b00001000
#define CMD9 0b00001001
#define CMD10 0b00001010
#define CMD12 0b00001100
#define CMD13 0b00001101
#define CMD16 0b00010000
#define CMD17 0b00010001
#define CMD18 0b00010010
#define CMD24 0b00011000
#define CMD25 0b00011001
#define CMD27 0b00011011
#define CMD28 0b00011100
#define CMD29 0b00011101
#define CMD30 0b00011110
#define CMD32 0b00100000
#define CMD33 0b00100001
#define CMD38 0b00100110
#define CMD55 0b00110111
#define CMD56 0b00111000
#define CMD58 0b00111010
#define CMD59 0b00111011

// Application specific commands to be run after command number 55
#define ACMD13 0b00001101
#define ACMD22 0b00101110
#define ACMD23 0b00101111
#define ACMD41 0b00101001
#define ACMD42 0b01000010
#define ACMD51 0b01000010

#endif  // HAMILTON_FIRMWARE_SD_CARD_H
