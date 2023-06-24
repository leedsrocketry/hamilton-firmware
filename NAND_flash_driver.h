/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Thomas Groom, Alexnadra Posta
  Created on: 09 March 2023
  Description: NAND Flash driver
*/

#include "mcu.h"

#define NONE 0

#define SUCCESS 1
#define STORAGE_FULL_WARNING 2
#define STORAGE_FULL_ERROR 3

#define DATA_INTACT 1
#define DATA_FIXED 2
#define DATA_CORRUPTED 3
#define EMPTY 4

#define DELAY 1
#define DELAY_PINMODE 50


typedef struct dateTime {
  uint8_t Year;          // 0 - 128
  uint8_t Month;         // 1 - 12
  uint8_t Day;           // 1 - 32
  uint8_t Hour;          // 0 - 23
  uint8_t Minute;        // 0 - 59
  uint8_t Second;        // 0 - 59
  uint16_t Millisecond;  // 0 - 999
  uint16_t Microsecond;  // 0 - 999
} dateTime;


typedef struct vector3 {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} vector3;

typedef struct GNSS_Data{
  uint16_t Latitude;
  uint16_t Longitude;
  uint32_t Heading1;
  uint32_t Velocity;
} GNSS_Data;

typedef struct frameArray {
  dateTime Date;
  uint16_t ChangeFlag;  //IS THIS NEEDED? CAN THIS BE DONE BETTER?
  vector3 AccelHighG;
  vector3 AccelLowG;
  vector3 Gyroscope;
  uint32_t Barometer;
  uint16_t Thermocouple[4];
  uint16_t Humidity;
  uint32_t Temp;
  uint16_t MagneticFieldStrength;
  GNSS_Data GNSS;
  uint16_t ADC[2];

  uint8_t HammingCode[8];
  uint16_t CRC_Check;

  int successFlag; // Not used in zip
} frameArray;


typedef struct address {
    uint16_t block;  // 12 bits
    uint8_t page;    // 6 bits
    uint16_t column; // 13 bits (12 used)
} address;

#define STANDBY 0b10001100 // CE# CLE ALE WE# RE# WP# X X
#define COMMAND_INPUT  0b01001100 // CE# CLE ALE WE# RE# WP# X X
#define ADDRESS_INPUT  0b00101100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_INPUT     0b00001100 // CE# CLE ALE WE# RE# WP# X X
#define DATA_OUTPUT    0b00011100 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT  0b00001000 // CE# CLE ALE WE# RE# WP# X X
#define WRITE_PROTECT_OFF   0b00001100 // CE# CLE ALE WE# RE# WP# X X

#define WE_HIGH        0b00010000  // CE# CLE ALE WE# RE# WP# X X
#define RE_HIGH       0b00001000

uint16_t data0 = PIN('D', 0); //2;
uint16_t data1 = PIN('D', 1); //3;
uint16_t data2 = PIN('D', 2); //4;
uint16_t data3 = PIN('D', 3); //5;
uint16_t data4 = PIN('D', 4); //11;
uint16_t data5 = PIN('D', 5); //12;
uint16_t data6 = PIN('D', 6); //8;
uint16_t data7 = PIN('D', 7); //9;

uint16_t WP  = PIN('D', 8); // 13;
uint16_t WE  = PIN('D', 9); // 14;
uint16_t ALE = PIN('E', 7); // 15;
uint16_t CLE = PIN('E', 8); // 16;
uint16_t CE  = PIN('E', 9); // 17;
uint16_t RE  = PIN('E', 11); // 18;
uint16_t RB  = PIN('E', 13); // 19;

uint32_t frameAddressPointer = 0;

uint8_t globalPinMode = GPIO_MODE_OUTPUT;

bool getBitArr(uint8_t *arr, int pos) {
  return (bool)(arr[pos/8] & (1 << (7-(pos%8))));
}

bool getBit(uint8_t byte, int pos) {
  return (bool)(byte & (1 << (7-(pos%8))));
}

void zip(frameArray unzippedData, uint8_t *zippedData) {
  int i = -1;

  zippedData[i++] = unzippedData.Date.Year;
  zippedData[i++] = unzippedData.Date.Month;
  zippedData[i++] = unzippedData.Date.Day;
  zippedData[i++] = unzippedData.Date.Minute;
  zippedData[i++] = unzippedData.Date.Second;
  zippedData[i++] = (uint8_t)((unzippedData.Date.Millisecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Date.Millisecond & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Date.Microsecond >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Date.Microsecond & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.ChangeFlag >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.ChangeFlag & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.AccelHighG.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelHighG.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.AccelHighG.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelHighG.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.AccelHighG.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelHighG.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.AccelLowG.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelLowG.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.AccelLowG.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelLowG.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.AccelLowG.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.AccelLowG.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.Gyroscope.x  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Gyroscope.x & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Gyroscope.y  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Gyroscope.y & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Gyroscope.z  >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Gyroscope.z & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.Barometer >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Barometer >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Barometer >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Barometer & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.Thermocouple[0] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Thermocouple[0] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Thermocouple[1] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Thermocouple[1] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Thermocouple[2] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Thermocouple[2] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Thermocouple[3] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Thermocouple[3] & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.Humidity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Humidity & 0xFF);
  
  zippedData[i++] = (uint8_t)((unzippedData.Temp >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Temp >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.Temp >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.Temp & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.MagneticFieldStrength >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.MagneticFieldStrength & 0xFF); 

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Latitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.Latitude & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Longitude >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.Longitude & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Heading1 >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Heading1 >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Heading1 >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.Heading1 & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Velocity >> 24) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Velocity >> 16) & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.GNSS.Velocity >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.GNSS.Velocity & 0xFF);

  zippedData[i++] = (uint8_t)((unzippedData.ADC[0] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.ADC[0] & 0xFF);
  zippedData[i++] = (uint8_t)((unzippedData.ADC[1] >> 8) & 0xFF);
  zippedData[i++] = (uint8_t)(unzippedData.ADC[1] & 0xFF);

  zippedData[118] = unzippedData.HammingCode[0];
  zippedData[119] = unzippedData.HammingCode[1];
  zippedData[120] = unzippedData.HammingCode[2];
  zippedData[121] = unzippedData.HammingCode[3];
  zippedData[122] = unzippedData.HammingCode[4];
  zippedData[123] = unzippedData.HammingCode[5];
  zippedData[124] = unzippedData.HammingCode[6];
  zippedData[125] = unzippedData.HammingCode[7];

  zippedData[126] = (uint8_t)((unzippedData.CRC_Check >> 8) & 0xFF);
  zippedData[127] = (uint8_t)(unzippedData.CRC_Check & 0xFF);
}

frameArray unzip(uint8_t *zippedData) {
  frameArray unzippedData;
  int i = -1;

  unzippedData.Date.Year = zippedData[i++];
  unzippedData.Date.Month = zippedData[i++];
  unzippedData.Date.Day = zippedData[i++];
  unzippedData.Date.Minute = zippedData[i++];
  unzippedData.Date.Second = zippedData[i++];
  unzippedData.Date.Millisecond = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Date.Millisecond |= zippedData[i++];
  unzippedData.Date.Microsecond = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Date.Microsecond |= zippedData[i++];

  unzippedData.ChangeFlag = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.ChangeFlag |= zippedData[i++];

  unzippedData.AccelHighG.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelHighG.x |= zippedData[i++];
  unzippedData.AccelHighG.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelHighG.y |= zippedData[i++];
  unzippedData.AccelHighG.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelHighG.z |= zippedData[i++];

  unzippedData.AccelLowG.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelLowG.x |= zippedData[i++];
  unzippedData.AccelLowG.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelLowG.y |= zippedData[i++];
  unzippedData.AccelLowG.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.AccelLowG.z |= zippedData[i++];

  unzippedData.Gyroscope.x = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Gyroscope.x |= zippedData[i++];
  unzippedData.Gyroscope.y = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Gyroscope.y |= zippedData[i++];
  unzippedData.Gyroscope.z = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Gyroscope.z |= zippedData[i++];

  unzippedData.Barometer = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.Barometer |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.Barometer |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Barometer |= zippedData[i++];

  unzippedData.Thermocouple[0] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Thermocouple[0] |= zippedData[i++];
  unzippedData.Thermocouple[1] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Thermocouple[1] |= zippedData[i++];
  unzippedData.Thermocouple[2] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Thermocouple[2] |= zippedData[i++];
  unzippedData.Thermocouple[3] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Thermocouple[3] |= zippedData[i++];

  unzippedData.Humidity = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Humidity |= zippedData[i++];
  
  unzippedData.Temp = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.Temp |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.Temp |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.Temp |= zippedData[i++];

  unzippedData.MagneticFieldStrength = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.MagneticFieldStrength |= zippedData[i++];

  unzippedData.GNSS.Latitude = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.Latitude |= zippedData[i++];
  unzippedData.GNSS.Longitude = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.Longitude |= zippedData[i++];

  unzippedData.GNSS.Heading1 = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.GNSS.Heading1 |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.GNSS.Heading1 |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.Heading1 |= zippedData[i++];

  unzippedData.GNSS.Velocity = (zippedData[i++] << 24) & (0xFF << 24);
  unzippedData.GNSS.Velocity |= (zippedData[i++] << 16) & (0xFF << 16);
  unzippedData.GNSS.Velocity |= (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.GNSS.Velocity |= zippedData[i++];

  unzippedData.ADC[0] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.ADC[0] |= zippedData[i++];
  unzippedData.ADC[1] = (zippedData[i++] << 8) & (0xFF << 8);
  unzippedData.ADC[1] |= zippedData[i++];

  unzippedData.HammingCode[0] = zippedData[118];
  unzippedData.HammingCode[1] = zippedData[119];
  unzippedData.HammingCode[2] = zippedData[120];
  unzippedData.HammingCode[3] = zippedData[121];
  unzippedData.HammingCode[4] = zippedData[122];
  unzippedData.HammingCode[5] = zippedData[123];
  unzippedData.HammingCode[6] = zippedData[124];
  unzippedData.HammingCode[7] = zippedData[125];

  unzippedData.CRC_Check = (zippedData[126] << 8) & (0xFF << 8);
  unzippedData.CRC_Check |= zippedData[127]; 

  return unzippedData;
}

void printByte(uint8_t myByte) {
  printf("0b");
  for (int i = 7; i >= 0; i--) {
    printf("%i", (myByte >> i) & 0b1);
  }
  printf("\r\n");
}


void printFrame(uint8_t dataArray[]) {
  printf("u");
  for (int i = 0; i < 128; i++) {
    /*if(dataArray[i] < 16) {
      printf("0");
    }*/
    printf("%02X", dataArray[i]);
  }
  //printfln("");
}


void printFrameHex(uint8_t dataArray[]) {
  for (int i = 0; i < 128; i++) {
    if(dataArray[i] < 16) {
      printf("0");
    }
    printf("%i", dataArray[i]);
    printf(", ");
  }
  printf("\r\n");
}

void _memset(uint8_t *arr, uint8_t val, int num){
  for(int i = 0; i < num; i++){
    arr[i] = val;
  }
}


void printFrameArray(frameArray frameFormat) {
  uint8_t dataArray[128];
  _memset(dataArray, 0, 128);
  zip(frameFormat, dataArray);
  printFrame(dataArray);
}


void waitForReadyFlag() {
  int count = 1000*100; // Try for 1 second before giving error
  while(gpio_read(RB) == LOW && count > 0){
    delayNanoseconds(10);
    count--;
  }
  if(count < 1){
    printf("waitForReadyFlag: TIMEOUT\r\n");
  } 
}


void setPinModes() {
  gpio_set_mode(data0, globalPinMode);
  gpio_set_mode(data1, globalPinMode);
  gpio_set_mode(data2, globalPinMode);
  gpio_set_mode(data3, globalPinMode);
  gpio_set_mode(data4, globalPinMode);
  gpio_set_mode(data5, globalPinMode);
  gpio_set_mode(data6, globalPinMode);
  gpio_set_mode(data7, globalPinMode);
  delayMicroseconds(DELAY_PINMODE);
}


void setControlPins(uint8_t controlRegister) {  // CE# CLE ALE WE# RE# WP#
  gpio_write(CE, getBit(controlRegister, 0));
  gpio_write(CLE, getBit(controlRegister, 1));
  gpio_write(ALE, getBit(controlRegister, 2));
  gpio_write(WE, getBit(controlRegister, 3));
  gpio_write(RE, getBit(controlRegister, 4));
  gpio_write(WP, getBit(controlRegister, 5));
}


void setDataPins(uint8_t Byte) {
  if (globalPinMode == GPIO_MODE_INPUT) {
    globalPinMode = GPIO_MODE_OUTPUT;
    setPinModes();
  }

  gpio_write(data0, getBit(Byte, 7));
  gpio_write(data1, getBit(Byte, 6));
  gpio_write(data2, getBit(Byte, 5));
  gpio_write(data3, getBit(Byte, 4));
  gpio_write(data4, getBit(Byte, 3));
  gpio_write(data5, getBit(Byte, 2));
  gpio_write(data6, getBit(Byte, 1));
  gpio_write(data7, getBit(Byte, 0));
}


void sendByteToFlash(uint8_t cmd, uint8_t mode) {
  //delayNanoseconds(DELAY);
  setControlPins(mode);
  setDataPins(cmd);
  //delayNanoseconds(DELAY);
  setControlPins(mode | WE_HIGH);
  //delayNanoseconds(DELAY);
}


uint8_t receiveByteFromFlash() {
  delayNanoseconds(DELAY);
  setControlPins(DATA_OUTPUT);
  delayNanoseconds(DELAY);
  setControlPins(DATA_OUTPUT & (~RE_HIGH));  // setting RE LOW
  delayNanoseconds(DELAY);
  if(globalPinMode != GPIO_MODE_INPUT){
    globalPinMode = GPIO_MODE_INPUT;
    setPinModes();
  }
  uint8_t data = (gpio_read(data7) << 7)
               | (gpio_read(data6) << 6)
               | (gpio_read(data5) << 5)
               | (gpio_read(data4) << 4)
               | (gpio_read(data3) << 3)
               | (gpio_read(data2) << 2)
               | (gpio_read(data1) << 1)
               | (gpio_read(data0) << 0);
  return data;
}

// sends the 5-byte-address to the nand using the frame and byte address as input
// 8,388,608 frames each with 128 bytes. frameAddr has 23 valid bits. byteAddr has 7 valid bits
void sendAddrToFlash(uint32_t frameAddr, uint8_t byteAddr) {
    address addr = {(frameAddr >> 11) & 0b0000111111111111,                      // block
                          (frameAddr >> 5) & 0b00111111,                               // page
                          ((frameAddr & 0b00011111) << 7) | (byteAddr & 0b01111111)};  // column 

    sendByteToFlash((uint8_t)(addr.column & 0b0000000011111111), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)((addr.column & 0b0001111100000000) >> 8), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)(((addr.block & 0b0000000000000011) << 6)) | (addr.page & 0b00111111), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)((addr.block & 0b0000001111111100) >> 2), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)((addr.block & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}


void sendBlockAddrToFlash(uint32_t blockAddr) {
    sendByteToFlash((uint8_t)(((blockAddr & 0b0000000000000011) << 6) | (0b00000000 & 0b00111111)), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)((blockAddr & 0b0000001111111100) >> 2), ADDRESS_INPUT);
    sendByteToFlash((uint8_t)((blockAddr & 0b0000110000000000) >> 10), ADDRESS_INPUT);
}

// Read the status register from the nand flash
uint8_t readFlashStatus() {
  waitForReadyFlag();
  sendByteToFlash(0x70, COMMAND_INPUT);
  return receiveByteFromFlash();
}

// Read the ID register from the nand flash
uint64_t readFlashID() {
  uint64_t id = 0;
  waitForReadyFlag();
  sendByteToFlash(0x90, COMMAND_INPUT);
  sendByteToFlash(0x00, ADDRESS_INPUT);
  for(int i = 0; i < 5; i++){
    printf("ID ");
    printf("%i", i);
    printf(": ");
    uint8_t byte = receiveByteFromFlash();
    id |= byte << (4-i);
    printByte(byte);
  }
  return id;
}


void writeProtection() {
  waitForReadyFlag();
  setControlPins(WRITE_PROTECT);  // Write Protection
}

// Code to read 1 frame from flash
void readFrame(uint32_t frameAddr, uint8_t *readFrameBytes, uint8_t _length) {
  waitForReadyFlag();
  sendByteToFlash(0x00, COMMAND_INPUT);
  sendAddrToFlash(frameAddr, 0);
  sendByteToFlash(0x30, COMMAND_INPUT);
  waitForReadyFlag();

  for(int byteAddr = 0; byteAddr < _length; byteAddr++){
    readFrameBytes[byteAddr] = receiveByteFromFlash();  // read data byte
  }
}

// Code to write 1 frame to the flash
void writeFrame(uint32_t frameAddr, uint8_t *bytes) {
  waitForReadyFlag();
  sendByteToFlash(0x80, COMMAND_INPUT);
  sendAddrToFlash(frameAddr, 0);  // Address Input
  delay(1);
  for(int byteAddr = 0; byteAddr < 128; byteAddr++){
    sendByteToFlash(bytes[byteAddr], DATA_INPUT);
  }
  
  sendByteToFlash(0x10, COMMAND_INPUT);
}

// A blocking function which will erase a block on the flash
void eraseBlock(uint32_t blockAddr) {
  waitForReadyFlag();
  sendByteToFlash(0x60, COMMAND_INPUT);
  sendBlockAddrToFlash(blockAddr);
  sendByteToFlash(0xD0, COMMAND_INPUT);
  waitForReadyFlag();  // Blocking Function
}

void eraseALL(){
  printf("WARNING: ERASING ALL DATA (UNPLUG NAND FLASH TO ABORT)\r\n");
  for(int countDown = 10; countDown > 0; countDown--){
    printf("ERASING DATA IN: ");
    printf("%i", countDown);
    printf(" Seconds\r\n");
    delay(1);
  }
  for(uint32_t block = 0; block < 64*4096; block++){
    eraseBlock(block);
    if(block%5000 == 0){
      printf("ERASING [");
      for(int i = 0; i < 50; i++){
        if(i < block/(64*4096*0.01*2)){
          printf("#");
        } else {
          printf(" ");
        }
      }
      printf("] - ");
      int percentage = (int)(block/(64*4096*0.01));
      printf("%d", percentage);
      printf("%%\r\n");
    }
  }
  printf("ERASING COMPLETE \r\n");
}

uint16_t max(uint16_t x1, uint16_t x2){
  return (x1 > x2) ? x1 : x2;
}

uint16_t min(uint16_t x1, uint16_t x2){
  return (x1 < x2) ? x1 : x2;
}

uint16_t diff(uint16_t x1, uint16_t x2) {
  return (uint16_t)abs((int)((int)x1 - (int)x2));
}


// Function which searches for next available block and returns the first frame address of that block
uint32_t getNextAvailableFrameAddr() {
  uint16_t prevPointer = 4096;
  uint16_t pointer = prevPointer / 2;
  uint8_t _check = 0;
  
  for(int i = 0; i < 11; i++) {
    
    readFrame(max(pointer - 1, 0) * 64 * 32, &_check, 1);  // Dosn't need to be the whole frame -------
    uint16_t difference = diff(pointer, prevPointer);
    prevPointer = pointer;
    if (_check == 0xFF) {
      pointer -= difference / 2;
    } else {
      pointer += difference / 2;
    }
  }
  
  readFrame(max(pointer - 1, 0) * 64 * 32, &_check, 1);
  if (_check != 0xFF) {
    pointer += 1;
  }

  return max(pointer - 1, 0) * 64 * 32;  // pointer * no of pages in block * no of frames in a page
}

/*
void testRoutine() {
  uint8_t bytes[128];
  _memset(bytes, 0xFF, 128);

  for(uint8_t i = 0; i < 128; i ++){
    bytes[i] = i;
  }

  int wrong = 0;
  int total = 0;

  int numOfFramesToCheck = 1000;
  
  //unsigned long timeBegin = micros();

  for(int i = 0; i < numOfFramesToCheck; i++){
    writeFrame(i, bytes);  // Write frame 0
    total += 128;
    
  }
  unsigned long timeEnd = micros();

  for(int i = 0; i < numOfFramesToCheck; i++){
    uint8_t bytesRead[128];
    readFrame(i, bytesRead, 128);
    for(int j = 0; j < 128; j++){
      if(bytesRead[j] != j){
        wrong++;
      }
    }
  }
  
  //unsigned long duration = timeEnd - timeBegin;
  printf("ms delay: ");
  printf(DELAY + "\n");
  printf("Incorrect Bytes: ");
  printf(wrong + "\n");
  printf("Total Bytes: ");
  printf(total + "\n");
  printf("Percent Incorrect: ");
  printf("%f", ((float)wrong * 100) / (float)total);
  printf(" %%\n");
  printf("Time took to write: ");
  printf(duration / 1000);
  printf(" ms\n");
  printf("Time waiting while nand flash was busy: ");
 // printf(count / 1000);
  printf(" ms\n");
  printf("Next available frame addr: ");
  printf(getNextAvailableFrameAddr() + "\n");
}
*/


void initialiseFlash() {
  gpio_set_mode(data0, GPIO_MODE_OUTPUT);
  gpio_set_mode(data1, GPIO_MODE_OUTPUT);
  gpio_set_mode(data2, GPIO_MODE_OUTPUT);
  gpio_set_mode(data3, GPIO_MODE_OUTPUT);
  gpio_set_mode(data4, GPIO_MODE_OUTPUT);
  gpio_set_mode(data5, GPIO_MODE_OUTPUT);
  gpio_set_mode(data6, GPIO_MODE_OUTPUT);
  gpio_set_mode(data7, GPIO_MODE_OUTPUT);

  gpio_set_mode(ALE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CLE, GPIO_MODE_OUTPUT);
  gpio_set_mode(CE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(RE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WE,  GPIO_MODE_OUTPUT);
  gpio_set_mode(WP,  GPIO_MODE_OUTPUT);

  gpio_set_mode(RB,  GPIO_MODE_INPUT);
  
  frameAddressPointer = getNextAvailableFrameAddr();

  if (readFlashID() != 0){
    printf("Flash Working Correctly\r\n");
  }
  
}

// --------------------------------

// Calculates CRC16-CCITT Checksum
uint16_t calculateCRC(uint8_t* data, uint8_t length) {
  const uint16_t CRC_POLY = 0x1021;
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ CRC_POLY : crc << 1;
    }
  }
  return crc;
}

// HAMMING CODE HASHING
void hash(uint8_t *_input, uint8_t *_output) {
  _memset(_output, 0, 120);
  for (int i = 0; i < 8*120; i++) {
    int j = ((i%120)*8) + (i/120);
    _output[i/8] |= getBitArr(_input, j) << (7-(i%8));
  }
}


bool isPowerOfTwo(int x) {
    return (x != 0) && ((x & (x - 1)) == 0);
}

// Calculate parity bits for a given encoded data frame
void calculateParityBits(uint8_t *_input, uint8_t *_output) {
  uint8_t hashedData[120];
  _memset(hashedData, 0, 120);

  uint8_t condition[120];
  _memset(condition, 0, 120);
  for (int i = 0; i < 118; i++){
    condition[i] = _input[i];
  }
  
  hash(condition, hashedData);

  for (int _set = 0; _set < 8; _set++) {
    uint8_t _word[15];
    for (int i = 0; i < 15; i ++) {
      _word[i] = hashedData[(_set * 15) + i];
    }
    
    // Initialize parity bits to 0
    uint8_t parities = 0;
    // Calculate parity bits
    for (int i = 0; i < 8; i++) {
      // Calculate bit position of this parity bit
      int bit_pos = 1 << i;
      
      // Calculate parity for this bit position
      uint8_t parity = 0;
      int k = 0;
      for (int j = 0; j < 128; j++) { // j from 0 - 128
        if (j + 1 != 1 && isPowerOfTwo(j + 1) == 0) {
          if (bit_pos & (j + 1)) {
            parity ^= (_word[k / 8] >> (k % 8)) & 1;
          }
          k++;
        }
      }
      parities |= parity << (i%8);
    }
    _output[_set] = parities;
  }
}



void encodeParity(frameArray dataFrame, uint8_t *bytes) {
  zip(dataFrame, bytes);

  uint8_t parities[8];
  calculateParityBits(bytes, parities);
  for (int i = 0; i < 8; i++){
    bytes[118+i] = parities[i];
  }
  uint16_t CRC_Check = calculateCRC(bytes, 126);
  bytes[126] = (uint8_t)((CRC_Check >> 8) & 0xFF);
  bytes[127] = (uint8_t)(CRC_Check & 0xFF);
}


void printCapacityInfo() {
  uint32_t lastFrameUsed = getNextAvailableFrameAddr();
  printf("Used: ");
  uint32_t usedInMB = (lastFrameUsed*128)/1000000;
  printf("%i", (int)usedInMB);
  printf(" MB (");
  printf("%f", lastFrameUsed/(4096*64*32*0.01));
  printf("%%) | ");
  printf("Estimated Time Remaining (hh:mm:ss): ");
  uint32_t hours = ((4096*64*32) - lastFrameUsed)/(1000*60*60)%24;
  uint32_t minutes = ((4096*64*32) - lastFrameUsed)/(1000*60)%60;
  uint32_t seconds = (((4096*64*32) - lastFrameUsed)/(1000))%60;
  if (hours<10) printf("0");
  printf("%i", (int)hours);
  printf(":");
  if (minutes<10) printf("0");
  printf("%i", (int)minutes);
  printf(":");
  if (seconds<10) printf("0");
  printf("%i", (int)seconds);
  printf("\r\n");
}



int logFrame(frameArray _input) {
  //printf("LOGFRAME addr ");
  //printfln(frameAddressPointer);

  // frameArray to array of bytes
  if (frameAddressPointer <= 8388607) {
    uint8_t encoded[128];
    _memset(encoded, 0, 128);
    encodeParity(_input, encoded);
    //printFrame(encoded);
    writeFrame(frameAddressPointer, encoded);
    frameAddressPointer++;
  } else {
    printf("Overflow Error\r\n");  // ERROR
    return STORAGE_FULL_ERROR;
  }
  // If the pointer is near the capacity of the storage
  // (95% full, 6.5 minutes left of recording)
  if (frameAddressPointer >= 8000000) {   
    return STORAGE_FULL_WARNING;
  }
  return SUCCESS;  // Successfully written
}


frameArray recallFrame(uint32_t frameAddr) {
  uint8_t encoded[128];
  _memset(encoded, 0, 128);
  frameArray _output;
  
  // Attempts re-reading the data from the flash 10 times before it gives up
  //_output.successFlag = DATA_CORRUPTED;
  //int timeout = 10;  
  //while(_output.successFlag == DATA_CORRUPTED && timeout > 0){
  //  timeout--;
  readFrame(frameAddr, encoded, 128);
  //  if ((encoded[0] & encoded[1]) != 0xFF) {
  //    _output = decodeParity(encoded);  // CHANGE TO ALLOW FOR INT RETURN OF STATUS
  //  } else {
  //    _output = unzip(encoded);  // Don't bother decoding parity bits
  //    _output.successFlag = EMPTY;
  //  }
  //}
  _output = unzip(encoded);  // Don't bother decoding parity bits
  return _output;
}

void readALL(){
  frameArray _output;
  _output.successFlag = NONE;

  uint32_t lastFrameToRead = getNextAvailableFrameAddr();
  
  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int data_empty = 0;

  for(uint32_t i = 0; i < lastFrameToRead; i++){
    _output = recallFrame(i);
    /*int flag = _output.successFlag;
    if(flag == DATA_INTACT){
      data_intact += 1;
      printFrame(_output);
    } else if(flag == DATA_FIXED) {
      data_fixed += 1;
      printFrame(_output);
    } else if(flag == EMPTY) {
      data_empty += 1;
    } else {
      data_error += 1;
    }*/
    printFrameArray(_output);
  }
  printf("----------------------------------------------\r\n");
  printCapacityInfo();
  printf("data_empty: ");
  printf(data_empty + "\r\n");
  printf("data_intact: ");
  printf(data_intact + "\r\n");
  printf("data_fixed: ");
  printf(data_fixed + "\r\n");
  printf("data_error: ");
  printf(data_error + "\r\n");
  printf("Percent Correct Data : ");
  printf("%i", (data_intact + data_fixed)/(4096*64*(4096/128)));
  printf("%%\r\n");
}
