/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Oliver Martin
    Created on: 10 June 2023
    Description: Driver file for the Thermocoupler module MAX31855KASA+T
   (http://datasheets.maximintegrated.com/en/ds/MAX31855.pdf)
*/

#include "MAX31855_driver.h"

MAX31855_data MAX31855_init(MAX31855_data* data, SPI_TypeDef spi, int CS) {
  data->MAX31855_SPI = spi;
  data->MAX31855_CS = CS;
};

MAX31855_data MAX31855_get_data(MAX31855_data* data) {
  // Device will start sending data as soon as CS is pulled low, no data needs
  // to be sent send no data, recieve first 14 bits which is th cold-junction
  // compensated thermocouple temperature

  // SPI send 0 bits, recieve 2 bytes
  uint32_t rawData = spi_transmit_receive(data->MAX31855_SPI, data->MAX31855_CS, 0, 0, 2);

  // now have a 16 bit response where we only want the first 14 bits
  // Remove last 2 bits
  uint16_t bitShifted = (uint16_t)rawData >> 2;

  // convert to number using the values of each bit, 0.25 resolution.
  data->temp = bitShifted;
  data->temp = data->temp / 4;

  // Fault
  data->fault = (0b000000000001 & rawData);

  return data;
};

MAX31855_data MAX31855_get_full_data(MAX31855_data* data) {
  // Device will start sending data as soon as CS is pulled low, no data needs
  // to be sent send no data, recieve 32 bits,
  /*
  Recieved in order of D31 first to D0 last.
  D31-D18: first 14 bits are cold-junction compensated thermocouple temperature,
  D31 is MSB, D31 is 1024 and D14 is 0.25 D17: reserved D16: Fault Bit D15-D4:
  Signed 12 bit internal temperature data, D15 is sign, D14 MSB worth 2^6, D4
  LSB worth 2^-4 D3: reserved D2: 1 = short to VCC D1: 1 = short for GND D0: 1 =
  Open circuit
  */

  // data structure
  MAX31855_data data;

  // SPI send 0 bits, recieve 4 bytes
  uint32_t rawData = spi_transmit_receive(MAX31855_SPI, 0, 0, 4);

  //----------- Get temperature reading -------------
  // get first 14 bits for the temperature reading
  uint16_t rawTempBits = (uint16_t)rawData >> 18;
  data.temp = rawTempBits;
  // divide by 4 for 0.25 resolution
  data.temp = data.temp / 4;

  // Any Fault (D16)
  data.fault = (0b000000000001 & rawTempBits);

  //---------- Get internal temperature --------------
  // mask the 12 bits where information is and shift to end
  rawTempBits = ((0b00000000000000001111111111110000 & rawData) >> 4);

  data.internalTemp = rawTempBits;
  // Divide by 2^4 for the resolution defined.
  data.internalTemp = data.internalTemp / 16;

  //----------- Get all fault info ------------------
  uint8_t faultBits = 0b00000000000000000000000000000111 & rawData;
  if (data.fault) {
    // if bit D2 is 1
    if (faultBits >> 2) {
      // there is a short to VCC
      data.faultType = 1;
    }
    // if bit D1 is 1
    else if ((faultBits & 0b00000010) >> 1) {
      // there is a short to GND
      data.faultType = 2;
    } else if ((faultBits & 0b00000001)) {
      // there is a open circuit fault
      data.faultType = 3;
    } else {
      // Unknown fault
      data.faultType = 4;
    }
  } else {
    data.faultType = 0;
  }

  return data;
};