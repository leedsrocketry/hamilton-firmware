/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Tyler Green
    Created on: 15 December 2023
    Description: Driver file for the
    Pad Radio module SI446 (https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)*/

#include "SI446_driver.h"


SPI_TypeDef *SI446_SPI;
SI446_settings chip_settings;
// __________________________________ Public _________________

int8_t SI446_init(SPI_TypeDef *spi){
    int8_t SI446_retVal;
    SI446_SPI = spi;
    chip_settings.SI446_channel = 1; //?????;
    SI446_retVal = SI446_power_up();
    return SI446_retVal;
};

int8_t SI446_transmit(){
    int8_t SI446_retVal;
    uint8_t SI446_command[4];
    SI446_command[0] = SI446_START_TX_CMD;
    SI446_command[1] = chip_settings.SI446_channel;
    SI446_command[2] = 00110000;   // go into ready state, send data currently in FIFO, imeadiately
    SI446_command[3] = 0; // use the packet handler packet config.
    spi_enable_cs(SI446_SPI, SI446_CS);// send_CTS_HIGH
    spi_write_buf(SI446_SPI, SI446_command, 4);
    SI446_retVal = SI446_check_CTS(-1);   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    spi_write_buf(SI446_SPI, SI446_command, 4);
    spi_disable_cs(SI446_SPI, SI446_CS);// send_CTS_low
    return SI446_retVal;
}

int8_t SI446_Recieve(){
    int8_t SI446_retVal;
    uint8_t SI446_command[7];
    SI446_command[0] = SI446_START_RX_CMD;
    SI446_command[1] = chip_settings.SI446_channel;
    SI446_command[2] = 1;  // go into RX state imeadiately
    SI446_command[3] = 0; // use the packet handler packet config.
    SI446_command[4] = chip_settings.SI446_RX_TIMEOUT_STATE;
    SI446_command[5] = chip_settings.SI446_RX_VALID_STATE;
    SI446_command[6] = chip_settings.SI446_RX_INVALID_STATE;
    spi_enable_cs(SI446_SPI, SI446_CS);// send_CTS_HIGH
    spi_write_buf(SI446_SPI, SI446_command, 7);
    SI446_retVal = SI446_check_CTS(-1);   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    spi_disable_cs(SI446_SPI, SI446_CS);// send_CTS_low
    return SI446_retVal;
}

int8_t SI446_write_data(uint8_t *data, size_t byteCount){
// NOT CHECKED::  there is enough sotrage in the FIFO to write to?   
    spi_enable_cs(SI446_SPI, SI446_CS);    // send_CTS_HIGH
    spi_write_byte(SI446_SPI, SI446_WRITE_TX_FIFO_CMD);
    spi_write_buf(SI446_SPI, data, byteCount);
    spi_disable_cs(SI446_SPI, SI446_CS); // send_CTS_LOW
    return 0;
};

int8_t SI446_read_data(uint8_t *data, size_t byteCount){
    // NOT CHECKED::  there is enough data in the FIFO to read from?

    spi_enable_cs(SI446_SPI, SI446_CS);// send_CTS_HIGH
    spi_write_byte(SI446_SPI, SI446_READ_RX_FIFO_CMD);
    spi_ready_read(SI446_SPI);
    spi_read_buf(SI446_SPI, data, byteCount);
    spi_disable_cs(SI446_SPI, SI446_CS);// send_CTS_LOW
    return 0;
};

//_________________________________ Private ____________________


int8_t SI446_get_response(int byteCount, uint8_t *data){
    int8_t SI446_retVal;
    // wait till CTS is correct
    SI446_retVal = SI446_check_CTS(0);
    if(SI446_retVal){
        for(int i = 0; i < byteCount; i ++){
            data[i] = spi_read_byte(SI446_SPI);
        }
        SI446_retVal = 1;
        
    } // CTS not correct
    return SI446_retVal;
};

int8_t SI446_check_CTS(int desired){
    int SI446_time_out = SI446_CTS_TIME_OUT;
    if(desired > 0){ // use user defined  
    SI446_time_out = desired;
    }
    else if(desired < 0){   // no timeout needed 
        if(spi_read_byte(SI446_SPI) == 0xFF){ 
            return 1; // CTS ready
        };
        return SI446_E_CTS_INVALID;
    }
    // check for time out
    while(SI446_time_out > 0){
        if((SI446_SPI->SR & BIT(1)) && (SI446_SPI->SR & BIT(0))){ // there is data
            LOG("Got data!");
            if(spi_read_byte(SI446_SPI) == 0xFF){
                return 1;
            }
            else{
                // NEED TO SEND NSELC HIGH AND
                spi_disable_cs(SI446_SPI, SI446_CS);
                return SI446_E_CTS_INVALID;
            }
        }
        else{
            LOG("No data!");
            delay_ms(1);   // wait 1ms
            SI446_time_out --;
        }
    }
    return SI446_E_CTS_TIME_OUT;
};

int8_t SI446_power_up(){
    int8_t SI446_retVal;
    uint8_t packet[7];
    uint32_t xo_freq = 30000000; // default
    LOG("SI446_power_up()...\r\n");
    packet[0] = SI446_POWER_UP_CMD;
    packet[1] = 1;
    packet[2] = 0;
    packet[3] = (uint8_t)(xo_freq >> 24);
    packet[4] = (uint8_t)(xo_freq >> 16);
    packet[5] = (uint8_t)(xo_freq >> 8);
    packet[6] = (uint8_t)(xo_freq);
    LOG("sending cmd...");
    spi_enable_cs(SI446_SPI, SI446_CS);// send_CTS_HIGH
    spi_write_buf(SI446_SPI, packet, 7);
    LOG("checking CTS...");
    SI446_retVal = SI446_check_CTS(50); // ensure it has been processed correctly..takes longer
    spi_disable_cs(SI446_SPI, SI446_CS);// send_CTS_LOW
    LOG("completed!  val: %d", SI446_retVal);
    return SI446_retVal;
}

