/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 03 March 2024
  Description: Header Drivers test routines.
*/

#include "test_routines.h"

/**
  @brief Initial Routine to run on hardware. Should trigger RGB blink sequence
  and Serial printing via LUART1.
*/
void run_test_routine()
{
  printf("================ Main_test_routine ================\r\n");
  bool on = true;
  for (;;) {
    delay_ms(1000);
    on = true;                                  // This block is executed
    gpio_write(GREEN_LED, on);                  // Every `period` milliseconds
    on = !on;                                   // Toggle LED state
    printf("LED: %d, tick: %lu\r\n", on, 1000); // Write message
  }
}

/**
  @brief Routine to test the MS5611 barometer.
*/
void run_MS5611_routine()
{
  printf("================ MS5611_routine ================\r\n");
  M5611_data _data;
  bool on = true;
  for (;;) {
    delay_ms(1000);
    gpio_write(GREEN_LED, on);
    on = !on;
    MS5611_get_data(&_data);
    printf("p: %d, t: %d, \r\n", _data.pressure, _data.temp);
  }
}

/**
  @brief Routine to test the ADXL375 accelerometer.
*/
void run_ADXL375_routine()
{
  printf("================ ADXL375_routine ================\r\n");
  ADXL375_data _data;
  uint8_t devid;
  
  for (;;) {
    delay_ms(1000);
    spi_enable_cs(SPI1, ADXL375_CS);
    spi_transmit_receive(SPI1, ADXL375_DEVID, 2, 1, &devid);
    spi_disable_cs(SPI1, ADXL375_CS);
    printf("ADXL375 Device ID: %d\r\n", devid);
  }
}

/**
  @brief Routine to test the SI446 radio module.
*/
void run_SI446_routine()
{
  printf("================ SI446_routine ================\r\n");
  int8_t ret_val = 123;
  ret_val = SI446_init(SPI1);
  printf("completed: %d \r\n ", ret_val);
}

/**
  @brief Routine to test the LSM6DS3 IMU.
*/
void run_LSM6DS3_routine()
{
  printf("================ LSM6DS3_routine ================\r\n");
  LSM6DS3_data gyro_data;
  delay_ms(50);
  lsm6ds6_init(SPI1, &gyro_data);
  
  for (;;) {
    lsm6ds6GyroReadAngle(SPI1, &gyro_data);
    printf("Gyro: %d, %d, %d, \r\n", gyro_data.x, gyro_data.y, gyro_data.z);
  }
}

/**
  @brief Routine to test the SPI communication.
*/
void spi_test_routine(SPI_TypeDef* spi)
{
  printf("================ SPI_routine ================\r\n");
  uint32_t timer = 0, period = 500;
  uint8_t ret_val = 0;
  static bool on = true; 

  for (;;)
  {
    gpio_write(GREEN_LED, on);        // Every `period` milliseconds
    on = !on;                         // Toggle LED state
    spi_transmit(spi, 2);
    ret_val = spi_read_byte(spi);
    printf("SPI: %d\r\n", ret_val);   // Write message
    delay_ms(1000);
  }
}

// 10000010 1100000 100111110
// 11010111 1100000
/**
  @brief Test that the SPI works appropriately; use Putty to check the LUART output
  @param spi Selected SPI (1, 2 or 3)
*/
/*
static inline uint16_t spi_test_routine(SPI_TypeDef *spi, uint16_t valueToSend) {
  valueToSend++;

  // Convert the integer to a byte array
  uint8_t byteBuffer[sizeof(valueToSend)];
  for (size_t i = 0; i < sizeof(valueToSend); ++i) {
    byteBuffer[i] = (uint8_t)(valueToSend >> (i * 8)) & 0xFF;
  }

  // Calculate the length of the byte array
  size_t bufferLength = sizeof(byteBuffer);

  spi_write_buf(spi, (char *)byteBuffer, bufferLength);

  // Wait for transfer to complete (until receive buffer is not empty)
  spi_ready_read(spi);

  // Read received data from SPI
  uint16_t receivedValue = spi_read_byte(spi);

  // Convert the received byte array back to an integer
  for (size_t i = 0; i < sizeof(receivedValue); ++i) {
    receivedValue |= ((uint16_t)byteBuffer[i] << (i * 8));
  }

  // Print the received integer
  printf("Received Value: %hu\r\n", receivedValue);

  return 0;
}*/

/**
  @brief Routine to test NAND Flash reading and writing.
*/
/*
void NAND_flash_test_routine()
{
  printf("==================== START WRITING ====================\r\n");
  uint8_t dataArray[128];
  _memset(dataArray, 0x0, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    dataArray[i] = i;
  }
  
  FrameArray _input = unzip(dataArray);

  int numOfFramesToTest = 100;
  for (int i = 0; i < numOfFramesToTest; i++) {
    for (uint8_t j = 0; j < 128; j ++) {
      dataArray[j] = j;
    }

    dataArray[0] = 0;
    dataArray[1] = 0;

    _input = unzip(dataArray);
    log_frame(_input);
    printf("======================== DONE ========================\r\n");
  }
  printf("==================== DONE WRITING ====================\r\n");
  
  read_all();
  print_capacity_info();
}
*/
