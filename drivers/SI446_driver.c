/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta, Evan Madurai
    Created on: 10 June 2023
    Description: Driver file for the
    Pad Radio module SI446 (https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)*/

#include "SI446_driver.h"


// __________________________________ Public _________________

int8_t init_SI446(SPI_Type_Def *spi){
    int8_t retVal;
    SI446_SPI = spi;

    return retVal;
};

void Write_data_SI446(uint8_t *data, size_t byteCount){
    spi_write_byte(SI446_SPI, SI446_WRITE_TX_FIFO_CMD);
    spi_write_buf(SI446_SPI, data, byteCount);
};

void Read_data_SI446(uint8_t *data, size_t byteCount){
    spi_write_byte(SI446_SPI, SI446_WRITE_RX_FIFO_CMD);
    spi_ready_read(SI446_SPI);
    for(int i = 0; i < byteCount; i ++){
        data[i] = spi_read_byte(SI446_SPI);
    }
};

int8_t Transmit_SI446(){
    int8_t retVal;
    uint8_t command[4];
    command[0] = SI446_START_TX_CMD;
    command[1] = SI446_Radio_settings.SI446_channel;
    command[2] = 00110000;   // go into ready state, send data currently in FIFO, imeadiately
    command[3] = 0; // use the packet handler packet config.
    spi_write_buf(SI446_SPI, command, 4);
    retVal = check_CTS(-1)   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    return retVal;
}

int8_t Recieve_SI446(){
    int8_t retVal;
    uint8_t command[7];
    command[0] = SI446_START_RX_CMD;
    command[1] = SI446_Radio_settings.SI446_channel;
    command[2] = 1;  // go into RX state imeadiately
    command[3] = 0; // use the packet handler packet config.
    command[4] = SI446_Radio_settings.SI446_RXTIME_OUT_STATE;
    command[5] = SI446_Radio_settings.SI446_RX_VALID_STATE;
    command[6] = SI446_Radio_settings.SI446_RX_INVALID_STATE;
    spi_write_buf(SI446_SPI, command, 7);
    retVal = check_CTS(-1)   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    return retVal;
}


//_________________________________ Private ____________________

/**
  @brief funtion to deal with recieving data from the sensor
  @note waits until CTS is correct then records data
  @param byteCount number of bytes to clock and read
  @param data ptr to the data storage location
  @return Success/Failure
*/
static int8_t get_response(int byteCount, uint8_t *data){
    int8_t retVal;
    // wait till CTS is correct
    retVal = check_CTS(0)
    if(retVal){
        for(int i = 0; i < byteCount; i ++){
            data[i] = spi_read_byte(SI446_SPI);
        }
        retVal = 1;
    } // CTS not correct
    return retVal;
};

/**
  @brief checks if CTS bit is set
  @note e.g. used to check if command with no response has been correctly carried out
  @param desired -1: no time_out, 
                  0: default (SI446_CTS_TIME_OUT)
                 >0: custom time (in ms) to wait for CTS response.
  @return Success/Failure
*/
static int8_t check_CTS(int desired){
    int time_out = SI446_CTS_TIME_OUT;
    if(desired > 0){ // use user defined  
    time_out = desired
    }
    else if(desired < 0){   // no timeout needed
        spi_ready_ready(SI446_SPI); // wait till buffer not empty 
        if(spi_read_byte(SI446_SPI) == 0xFF){ 
            return 1; // CTS ready
        };
        return SI446_E_CTS_INVALID;
    }
    // check for time out
    while(time_out > 0){
        if((SI446_SPI->SR & BIT(1)) && (SI446_SPI->SR & BIT(0))){ // there is data
            if(spi_read_byte(SI446_SPI) == 0xFF){
                return 1;
            }
            else{
                // NEED TO SEND NSELC HIGH AND 
                return SI446_E_CTS_INVALID;
            }
        }
        else{
            delay(1);   // wait 1ms
            time_out --;
        }
    }
    return SI446_E_CTS_TIME_OUT;
};

/**
  @brief COMMAND: power up the sensor with specified function
  @return Success/Failure
*/
static int8_t Power_up(){
    int8_t retVal;
    uint8_t packet[7];
    uint32_t xo_freq = 30000000; // default

    packet[0] = SI446_POWER_UP_CMD;
    packet[1] = 1;
    packet[2] = 0;
    packet[3] = (uint8_t)(xo_freq >> 24);
    packet[4] = (uint8_t)(xo_freq >> 16);
    packet[5] = (uint8_t)(xo_freq >> 8);
    packet[6] = (uint8_t)(xo_freq);
    spi_write_buf(SI446_SPI, packet, 7);
    retVal = check_CTS(50); // ensure it has been processed correctly..takes longer
    return retVal;
};


