/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta, Evan Madurai, Oliver Martin
    Created on: 10 June 2023
    Description: Driver file for the Temperature module MAX31855KASA+T (https://www.mouser.co.uk/ProductDetail/Analog-Devices/ADXL375BCCZ?qs=S4ILP0tmc7Q%2Fd%2FHPWf9YpQ%3D%3D)
*/

//Does anything need to be done for the CS pin??

#include "MAX31855_driver.h"

int8_t MAX31855_init(SPI_TypeDef spi){
    MAX31855_SPI = spi;
};

int8_t MAX31855_get_data(){
    //Device will start sending data as soon as CS is pulled low, no data needs to be sent
    //send no data, recieve first 14 bits which is th cold-junction compensated thermocouple temperature

    //SPI send 0 bits, recieve 2 bytes
    uint16_t rawData = spi_transmit_receive(MAX31855_SPI, 0, 0, 2);

    //now have a 16 bit response where we only want the first 14 bits
    //Remove last 2 bits
    uint16_t bitShifted = rawData >> 2; 
    //convert to number using the values of each bit, 0.25 resolution.

};

int8_t MAX31855_get_full_data(){
    //Device will start sending data as soon as CS is pulled low, no data needs to be sent
    //send no data, recieve 32 bits, 
    /*
    Recieved in order of D31 first to D0 last.
    D31-D18: first 14 bits are cold-junction compensated thermocouple temperature, D31 is MSB, D31 is 1024 and D14 is 0.25
    D17: reserved
    D16: Fault Bit
    D15-D4: Signed 12 bit internal temperature data, D15 is sign, D14 MSB worth 2^6, D4 LSB worth 2^-4
    D3: reserved
    D2: 1 = short to VCC
    D1: 1 = short for GND
    D0: 1 = Open circuit
    */

    struct MAX31855_data data;
    //SPI send 0 bits, recieve 4 bytes
    uint32_t rawData = spi_transmit_receive(MAX31855_SPI, 0, 0, 4);
    
    //get first 14 bits for the temperature reading
    data.temp = rawData >> (18) //need to confirm this is correct

    //get internal temperature

    //get all fault info

    

};