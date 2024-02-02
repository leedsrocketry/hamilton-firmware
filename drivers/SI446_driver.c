/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Tyler Green
    Created on: 15 December 2023
    Description: Driver file for the
    Pad Radio module SI446 (https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)*/

#include "SI446_driver.h"


// __________________________________ Public _________________

int8_t SI446_init(SPI_Type_Def *spi){
    int8_t SI446_retVal;
    SI446_SPI = spi;
    SI446_settings.SI446_channel; //?????;

    SI446_retVal = SI446_Power_up();
    return SI446_retVal;
};

void SI446_write_data(uint8_t *data, size_t byteCount){
    // send_CTS_HIGH
    spi_write_byte(SI446_SPI, SI446_WRITE_TX_FIFO_CMD);
    spi_write_buf(SI446_SPI, data, byteCount);
    // send_CTS_LOW
};
void SI446_read_byte(){
    // send_CTS_HIGH
    spi_write_byte(SI446_SPI, SI446_WRITE_RX_FIFO_CMD);
    spi_ready_read(SI446_SPI);
    spi_read_byte(SI446_SPI);
    // send_CTS_LOW
};

int8_t SI446_transmit(){
    int8_t SI446_retVal;
    uint8_t SI446_command[4];
    SI446_command[0] = SI446_START_TX_CMD;
    SI446_command[1] = SI446_settings.SI446_channel;
    SI446_command[2] = 00110000;   // go into ready state, send data currently in FIFO, imeadiately
    SI446_command[3] = 0; // use the packet handler packet config.
    // send_CTS_HIGH
    spi_write_buf(SI446_SPI, SI446_command, 4);
    SI446_retVal = SI446_check_CTS(-1)   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    spi_write_buf(SI446_SPI, SI446_command, 4);
    // send_CTS_low
    return SI446_retVal;
}

int8_t SI446_Recieve(){
    int8_t SI446_retVal;
    uint8_t SI446_command[7];
    SI446_command[0] = SI446_START_RX_CMD;
    SI446_command[1] = SI446_settings.SI446_channel;
    SI446_command[2] = 1;  // go into RX state imeadiately
    SI446_command[3] = 0; // use the packet handler packet config.
    SI446_command[4] = SI446_settings.SI446_RXTIME_OUT_STATE;
    SI446_command[5] = SI446_settings.SI446_RX_VALID_STATE;
    SI446_command[6] = SI446_settings.SI446_RX_INVALID_STATE;
    // send_CTS_HIGH
    spi_write_buf(SI446_SPI, SI446_command, 7);
    SI446_retVal = SI446_check_CTS(-1)   // CTS returned once chip is in TX state - doesnt mean packet has been sent
    // send_CTS_low
    return SI446_retVal;
}


//_________________________________ Private ____________________


static int8_t SI446_get_response(int byteCount, uint8_t *data){
    int8_t SI446_retVal;
    // wait till CTS is correct
    SI446_retVal = SI446_check_CTS(0)
    if(SI446_retVal){
        for(int i = 0; i < byteCount; i ++){
            data[i] = spi_read_byte(SI446_SPI);
        }
        SI446_retVal = 1;
    } // CTS not correct
    return SI446_retVal;
};


static int8_t SI446_check_CTS(int desired){
    int SI446_time_out = SI446_CTS_TIME_OUT;
    if(desired > 0){ // use user defined  
    SI446_time_out = desired
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


static int8_t SI446_power_up(){
    int8_t SI446_retVal;
    uint8_t packet[7];
    uint32_t xo_freq = 30000000; // default

    packet[0] = SI446_POWER_UP_CMD;
    packet[1] = 1;
    packet[2] = 0;
    packet[3] = (uint8_t)(xo_freq >> 24);
    packet[4] = (uint8_t)(xo_freq >> 16);
    packet[5] = (uint8_t)(xo_freq >> 8);
    packet[6] = (uint8_t)(xo_freq);
    // send_CTS_HIGH
    spi_write_buf(SI446_SPI, packet, 7);
    SI446_retVal = SI446_check_CTS(50); // ensure it has been processed correctly..takes longer
    // send_CTS_LOW
    return SI446_retVal;
};


