//
// Created by Morgan Thomas on 06/08/2024.
//
// Resources
// https://eax.me/stm32-sdcard/
// https://github.com/afiskon/stm32-sdcard/tree/master?tab=readme-ov-file
//

#include "sd_card.h"

int8_t CRCTable[256];

void GenerateCRCTable() {
    int i, j;
    uint8_t CRCPoly = 0x89;  // the value of our CRC-7 polynomial

    // generate a table value for all 256 possible byte values
    for (i = 0; i < 256; ++i) {
        CRCTable[i] = (i & 0x80) ? i ^ CRCPoly : i;
        for (j = 1; j < 8; ++j) {
            CRCTable[i] <<= 1;
            if (CRCTable[i] & 0x80)
                CRCTable[i] ^= CRCPoly;
        }
    }
}

// adds a message byte to the current CRC-7 to get a the new CRC-7
uint8_t CRCAdd(uint8_t crc, uint8_t message_byte) {
    return CRCTable[(crc << 1) ^ message_byte];
}

// returns the CRC-7 for a message of "length" bytes
uint8_t getCRC(uint8_t message[], uint8_t length)
{
    int i;
    uint8_t crc = 0;

    for (i = 0; i < length; ++i)
        crc = CRCAdd(crc, message[i]);

    return crc;
}

SD_Card init_sd_card(SPI_TypeDef *spi_device, uint8_t chip_select) {
    SD_Card sd_card_dev = {.spi_interface=spi_device,.chip_select=chip_select,
                           .version=0, .compatible=0, .initialised=0, .high_capacity=0};
    uint8_t r1[cmd_resp_size(CMD0)];
    r1[0] = 255;
    sd_send_command(&sd_card_dev, CMD0, 0, r1, true);
    if (!(r1[0] & 0x01)){
        LOG("SD card is not in idle state 0x%02x\r\n", r1[0]);
    }

    uint8_t r7[cmd_resp_size(CMD8)];
    // cmd8 args explained
    // 31-11 (20 bits) reserved
    // 11-7 (4 bits) host voltage supply 0001 for 2.7-3.6v
    // 7-0 (8 bits) Check pattern any random bits to check command was completed
    uint32_t cmd8_argument = 0x000001AA;
    sd_send_command(&sd_card_dev, CMD8, cmd8_argument, r7, true);
    if ( r7[0] & 0b00000100){ // If this is an invalid command the sd card is v1
        sd_card_dev.version = 1;
        LOG("INFO - Version 1.0 SD Card\r\n");

        // Read OCR
        uint8_t r3[cmd_resp_size(CMD58)];
        sd_send_command(&sd_card_dev, CMD58, 0, r3, true);
        if (r3[0] > 1) {
            LOG("ERROR - Error occurred whilst trying to read OCR register\r\n");
        } else {
            if ((r3[2] & 0b00010000) || (r3[2] & 0b00100000)) {
                LOG("INFO - SD card has correct voltage");
                sd_card_dev.compatible = 1;
                r1[0] = 0xff;
                uint8_t broken = 0;
                for (uint8_t i = 0; i < 5; i++) {
                    sd_send_command(&sd_card_dev, CMD55, 0x00000000, r1, true);
                    sd_send_command(&sd_card_dev, ACMD41, 0x00000000, r1, true);
                    if ((r1[0] & 0x04)) {
                        LOG("ERROR - Not an SD card\r\n");
                        sd_card_dev.compatible = 0;
                        sd_card_dev.initialised = 0;
                        return sd_card_dev;
                    }
                    if (!r1[0]){
                        broken = 1;
                        break;
                    }
                }
                if (!broken){
                    LOG("Error - Failed to initialise SD card ACMD41 timed out\r\n");
                    return sd_card_dev;
                }
                sd_card_dev.initialised = 1;
                sd_send_command(&sd_card_dev, CMD16, 0x00000200, r1, true);
                LOG("INFO - SD card initialised and ready");
            } else {
                LOG("ERROR - SD card is supplied incorrect voltage");
            }
        }
    } else if ( r7[4] == 0xAA) { // The SD card is v2 if the command is valid
        sd_card_dev.version = 2;
        LOG("INFO - Version 2.0 SD Card\r\n");
        if ((r7[3] & 0x1f) == 0x1) {
            LOG("INFO - Voltage accepted\r\n");

            // Read OCR
            uint8_t r3[cmd_resp_size(CMD58)];
            sd_send_command(&sd_card_dev, CMD58, 0, r3, true);
            if (r3[0] > 1) {
                LOG("ERROR - Error occurred whilst trying to read OCR register\r\n");

            } else {
                if ((r3[2] & 0b00010000) || (r3[2] & 0b00100000)) {
                    LOG("INFO - SD card has correct voltage\r\n");
                    sd_card_dev.compatible = 1;
                    uint8_t broken = 0;
                    for (uint8_t i = 0; i < 10; i++) {
                        sd_send_command(&sd_card_dev, CMD55, 0x00000000, r1, true);
                        sd_send_command(&sd_card_dev, ACMD41, 0x40000000, r1, true);
                        if (!r1[0]) {
                            broken = 1;
                            break;
                        }
                    }
                    if (!broken) {
                        LOG("Error - Failed to initialise SD card ACMD41 timed out card in idle state\r\n");
                        return sd_card_dev;
                    }

                    sd_send_command(&sd_card_dev, CMD58, 0, r3, true);
                    if (r3[1] & 0x40) {
                        sd_card_dev.high_capacity = 1;
                    }

                    sd_send_command(&sd_card_dev, CMD16, 0x00000200, r1, true);
                    sd_card_dev.initialised = 1;
                    LOG("INFO - SD card initialised and ready\r\n");
                } else {
                    LOG("ERROR - SD card is supplied incorrect voltage\r\n");
                }
            }
        } else {
            LOG("ERROR - Voltage not accepted\r\n");
        }
    } else {
        LOG("ERROR - An error occurred whilst initialising the card\r\n");

    }

    return sd_card_dev;
}


void select_sd_card(SD_Card *sd_device){
    spi_enable_cs(sd_device->spi_interface, sd_device->chip_select);
}

void deselect_sd_card(SD_Card *sd_device){
    spi_disable_cs(sd_device->spi_interface, sd_device->chip_select);
}

void sd_send_command(SD_Card *sd_device, uint8_t command, uint32_t argument, uint8_t *response, uint8_t cs) {
    uint8_t command_token[6];

    command_token[0] = 0b01000000 | command; // adding start and transmission bit to command

    command_token[1] = (uint8_t)((argument >> 24) & 0xFF); // add argument into array 1-4
    command_token[2] = (uint8_t)((argument >> 16)& 0xFF);
    command_token[3] = (uint8_t)((argument >> 8) & 0xFF);
    command_token[4] = (uint8_t)(argument & 0xFF);

    command_token[5] = getCRC(command_token, 5) << 1 | 0b00000001;
    uint8_t recv_size = cmd_resp_size(command);

    if (cs){
        select_sd_card(sd_device);
    }
    sd_transmit_receive(sd_device->spi_interface, command_token, 6, recv_size, response, 1);

    if (cs) {
        deselect_sd_card(sd_device);
    }
}


uint8_t cmd_resp_size(uint8_t command) {
    uint8_t recv_size;
    switch (command) {
        case CMD8:
            recv_size = 5;
            break;
        case CMD13:
            recv_size = 2;
            break;
        case CMD58:
            recv_size = 5;
            break;
        default:
            recv_size = 1;
    }
    return recv_size;

}

inline void sd_transmit_receive(SPI_TypeDef *spi, uint8_t *send_byte, uint8_t transmit_size,
                                        uint8_t receive_size, uint8_t *result_ptr, uint8_t wait_cmds) {
    spi_ready_write(spi);

    // Transmit data
    for (uint8_t i = 0; i < transmit_size; i++) {
        spi_transmit(spi, send_byte[i]);
    }

    for (uint8_t i = 0; i < wait_cmds; i++){
        spi_transmit(spi, 0xff);
    }

    // Receive data
    for (uint8_t i = 0; i < receive_size; i++) {
        uint8_t received = spi_transmit(spi, 0xff);
//        LOG("0x%02x\r\n", received);
        result_ptr[i] = received;
        spi_ready_write(spi);
    }

}

uint8_t sd_read_block(SD_Card *sd_card, uint32_t address, uint8_t *result_ptr) {
    uint8_t *r1 = 0xff;
    select_sd_card(sd_card);

    sd_send_command(sd_card, CMD18, address, r1, false);

    if (r1[0] != 0xFF) {
        uint8_t read_attempt = 0;
        while (read_attempt++ < 100) {
            r1[0] = spi_transmit(sd_card->spi_interface, 0x00);
            if (r1[0] != 0xFF) {
                LOG("Response - 0x%02x\r\n", r1[0]);
                break;
            }
        }
        if (read_attempt >= 99) {
            return 1;
        }

        if (r1[0] == 0xFE) {
            for (uint16_t i = 0; i < 512; i++) {
                result_ptr[i] = spi_transmit(sd_card->spi_interface, 0xFF);
            }
            // Read CRC
            (void) spi_transmit(sd_card->spi_interface, 0xFF);
            (void) spi_transmit(sd_card->spi_interface, 0xFF);

            (void) spi_transmit(sd_card->spi_interface, 0x00);
            sd_send_command(sd_card, CMD12, 0, r1, false);
            deselect_sd_card(sd_card);
            return 0;
        } else {
            deselect_sd_card(sd_card);
            return 2;
        }
    } else {
        return 3;
    }
}


uint8_t sd_write_block(SD_Card *sd_card, uint32_t address, uint8_t *data) {
    if (sd_card->initialised) {
        uint8_t r1[1];
        sd_send_command(sd_card, CMD24, address, r1, true);

        // Send dummy bytes till sd ready
        uint8_t i;
        select_sd_card(sd_card);
        for (i = 0; i < 20; i++){
            spi_ready_write(sd_card->spi_interface);
            r1[0] = spi_transmit(sd_card->spi_interface, 0x00);
            if (!r1[0]) {
                break;
            }
        }
        if (i < 20){
            LOG("INFO - Write block timed out");
            return 1;
        }

        spi_ready_write(sd_card->spi_interface);
        (void) spi_transmit(sd_card->spi_interface, 0x00);
        (void) spi_transmit(sd_card->spi_interface, 0xFE);

        // Send data
        for (uint16_t i = 0; i < 512; i++) {
            (void) spi_transmit(sd_card->spi_interface, data[i]);
        }
        // Send CRC
        (void) spi_transmit(sd_card->spi_interface, 0x00);
        (void) spi_transmit(sd_card->spi_interface, 0x00);


        // Wait for write to be finished
        uint8_t res;
        for (i = 0; i < 100; i++) {
            res = spi_transmit(sd_card->spi_interface, 0x00);
            if (res) {
                break;
            }
        }
        if (i >= 100) {
            LOG("ERROR - Block write timed out");
            return 2;
        }
        return 0;
    }
    return 3;
}