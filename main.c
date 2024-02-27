/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>
#include "mcu.h"
#include "STM32_init.h"
#include "stm32l4r5xx.h"
// #include "NAND_flash_driver.h"
#include "drivers/MS5611_driver.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/LSM6DS3_driver.h"
#include "drivers/SI446_driver.h"

// Flags
FlightStages flightStage = LAUNCHPAD;
volatile uint32_t s_ticks;

/**
  @brief Required for compilation
*/

void SysTick_Handler(void)
{
  s_ticks++;
}

/**
  @brief TODO
*/
void update_sensors() {}

/**
  @brief Buzzer sound
  @param onDurationMs
  @param offDurationMs
  @param noOfBeeps
*/
void STM32_beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps)
{
  for (int i = 0; i < noOfBeeps; i++) {
      gpio_write(_buzzer, HIGH);
      delay_ms(onDurationMs);
      gpio_write(_buzzer, LOW); 
      delay_ms(offDurationMs);
  }
}

/**
  @brief Buzzer sound to indicate power on
*/
void STM32_indicate_on_buzzer()
{
  STM32_beep_buzzer(100, 100, 3);
}

/**
  @brief Led light to indicate power on
*/
void STM32_indicate_on_led()
{
  STM32_led_on();
  delay_ms(200);
  STM32_led_off();
  delay_ms(100);
  STM32_led_on();
  delay_ms(200);
  STM32_led_off();
}

/*
void send_data() {
  setControlPins(WRITE_PROTECT);                  // Write Protection
  setControlPins(WRITE_PROTECT_OFF);              // Write Protection Off

  uint8_t dataArray[128];
  _memset(dataArray, 0x0, 128);

  for (uint8_t i = 0; i < 128; i ++) {
    dataArray[i] = i;
  }

  eraseBlock(0);
  eraseALL();

  writeFrame(0, dataArray);
  readFrame(10000, dataArray);
  FrameArray _input = unzip(dataArray);
  frameArray _output;

  int data_intact = 0;
  int data_fixed = 0;
  int data_error = 0;
  int startAddr = frameAddressPointer;

  int numOfFramesToTest = 100;
  for (int i = 0; i < numOfFramesToTest; i++) {
    for (uint8_t j = 0; j < 128; j ++) {
      dataArray[j] = j;
    }
    dataArray[0] = 0;
    dataArray[1] = 0;
    _input = unzip(dataArray);
    log_frame(_input);
    printf("======================== DONE ========================");
  }
  printf("==================== DONE WRITING ====================\r\n");
  read_all();
  print_capacity_info();
}
*/

/**
  @brief TODO
*/
void toggle_timeout_flag() {}

/**
  @brief Initial Routine to run on hardware. Should trigger RGB blink sequence
  and Serial printing via LUART1
*/
void run_test_routine()
{
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  pwr_vdd2_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);

  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message
    }
  }
}

/**
  @brief Routine to test the MS5611 barometer.
*/
void run_MS5611_routine()
{
  MS5611_init(SPI1);
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  pwr_vdd2_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);

  uint32_t timer = 0, period = 100;
  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      //printf("Tick: %lu\r\n", s_ticks); // Write message
      static bool on = true;            // This block is executed
      gpio_write(led_B, on);            // Every `period` milliseconds
      on = !on;
      MS5611_get_data_test(); // Write message
    }
  }
}

/**
  @brief Routine to test the ADXL375 accelerometer.
*/
void run_ADXL375_routine()
{
  printf("================ ADXL375_routine ================ \r\n");
  ADXL375_init(SPI1);

  ADXL375_data data;
  data = ADXL375_get_data();
  printf("x: %d\r\n", data.x);
  // uint32_t test = spi_transmit_receive(SPI1, ADXL375_CS, 0x23, 1, 1);
}

void SI446_Test_routine()
{
  printf("================ SI446_routine ================ \r\n");
  int8_t ret_val = 123;
  ret_val = SI446_init(SPI1);
  printf("completed: %d \r\n ", ret_val);
}

void LSM6DS3_test_routine()
{
  printf("================ LSM6DS3_routine ================ \r\n");
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  pwr_vdd2_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);

  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message

      int8_t ret_val = 123;
      ret_val = LSM6DS3_init(SPI1);
      printf("completed: %d \r\n ", ret_val);
    }
  }
}

void spi_test_routine()
{
  uint16_t led_B = PIN('H', 3);
  gpio_set_mode(led_B, GPIO_MODE_OUTPUT);
  pwr_vdd2_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);

  uint32_t timer = 0, period = 500;

  for (;;)
  {
    if (timer_expired(&timer, period, s_ticks))
    {
      static bool on = true;                         // This block is executed
      gpio_write(led_B, on);                         // Every `period` milliseconds
      on = !on;                                      // Toggle LED state
      printf("LED: %d, tick: %lu\r\n", on, s_ticks); // Write message
      //spi_write_byte(SPI1,0);
      uint8_t ret_val = 0;
      //spi_write_byte(SPI1,124);
      //ret_val = spi_read_byte(SPI1);
      printf("return: %d", ret_val);
    }
  }
}

/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/
int main(void)
{
  STM32_init_clock(RCC_CFGR_SW_HSI); // set clock to 16MHz internal HSI
  STM32_init();
  systick_init(FREQ / 1000);
  uart_init(LUART1, 9600);
  run_ADXL375_routine();
}

// TODO - Add the following to the main loop
/*
STM32_init(); // Initialise the board
printf("==================== PROGRAM START ==================\r\n");

// Initialise the drivers
printf("================ INITIALISE FC DRIVERS ===============\`r\n");
init_flash();
//erase_all();
frameAddressPointer = 0;
// init_MAXM10S(); GNSS
// init_BME280(); Temperature + Humidity
// init_MS5611(); Barometer
// init_ADXL375(); Accelerometer
// init_LSM6DS3(); Gyroscope
// init_MAX31855(); Temperature
// init_SI446(); Pad Radio

printf("================ ENTER MAIN PROCEDURE ================\r\n");

for (;;) {
  // Complete based on flight stage
  #pragma region Flight Stages
  switch (flightStage) {
  case LAUNCHPAD:
      // TODO
      // save a circular buffer of sensor readings (when launch is detected)
      // check for altitude off GPS and -> Barometer
      // check for acceleration (mostly)
      // if above threshold, change flightStage to ASCEND
      break;

  case ASCEND:
      // TODO
      // update all sensor readings at high rate
      // save sensor readings to FDR/over telemetry link
      // detect apogee based on gradient of altitude
      break;

  case APOGEE:
      // TODO
      // update all sensor readings at high rate
      // save sensor readings to FDR/over telemetry link
      // trigger recovery system
      break;

  case DESCENT:
      // TODO
      // update all sensor readings at lower rate
      // save sensor readings to FDR/over telemetry link
      // if no acceleration/costant altitude, change flightStage to LANDING
      break;

  case LANDING:
      // TODO
      // stop recording
      // change buzzer sequence
      // change LED sequence
      break;
  }
  #pragma endregion Flight Stages

  //send_data();
}

// Exit program
printf("===================== PROGRAM END ====================");
*/
