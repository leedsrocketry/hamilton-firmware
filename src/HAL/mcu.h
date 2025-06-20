/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 27 Feb 2023
  Description: Main header file for the HFC firmware; suitable for STM32L4R5
*/
#pragma once

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "debug.h"

// https://github.com/STMicroelectronics/cmsis_device_l4/blob/master/Include/system_stm32l4xx.h
#include "stm32l4r5xx.h"

extern volatile uint32_t s_ticks;
extern uint32_t FREQ;

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

#define LOW false
#define HIGH true

#pragma region Struct
typedef struct DateTime {
  uint8_t year;          // 0 - 128
  uint8_t month;         // 1 - 12
  uint8_t day;           // 1 - 32
  uint8_t hour;          // 0 - 23
  uint8_t minute;        // 0 - 59
  uint8_t second;        // 0 - 59
  uint16_t millisecond;  // 0 - 999
  uint16_t microsecond;  // 0 - 999
} DateTime;

typedef struct GNSS_Data {
  uint16_t latitude;
  uint16_t longitude;
  uint16_t altitude;
  uint16_t velocity;
} GNSS_Data;
#pragma endregion Struct

/**
  @brief Print a float
  @param name Name of the float
  @param value Value of the float
  @param print_text Print text or not
  @note marked as unused because it may not be used in any given runtime
*/
static void printf_float(char *name, float value, bool print_text) __attribute__((unused));
static void printf_float(char *name, float value, bool print_text) {
  char str[30];

  char *tmpSign = (value < 0) ? "-" : "";
  float tmpVal = (value < 0) ? -value : value;

  uint32_t tmpInt1 = (uint32_t)tmpVal;               // Get the integer (678).
  float tmpFrac = (tmpVal - (float)tmpInt1);         // Get fraction (0.0123).
  int32_t tmpInt2 = (int32_t)trunc(tmpFrac * 1000);  // Turn into integer (123).

  // Print as parts, note that you need 0-padding for fractional bit.
  // Prints in format "123.456" or "value: 123.456"
  if (print_text)
    sprintf(str, "%s: %s%ld.%03ld", name, tmpSign, tmpInt1, tmpInt2);
  else
    sprintf(str, "%s%ld.%03ld", tmpSign, tmpInt1, tmpInt2);

  // Print the string
  LOG("%s", str);
}

#pragma region System Clk
/**
  @brief The low level delay
*/
static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

static inline uint32_t get_time_us() { return TIM2->CNT; }

static inline uint32_t get_time_ms() { return s_ticks; }

/**
  @brief Delay in nanoseconds
  @param time Time in nanoseconds
*/
static inline void delay_nanoseconds(uint32_t time) { spin(time); }

/**
  @brief Delay in microseconds
  @param time Time in microseconds
*/
static inline void delay_microseconds(uint32_t time) {
  uint32_t startTime = get_time_us();
  while ((get_time_us() - startTime) < time);
}

/**
  @brief Delay in miliseconds
  @param time Time in miliseconds
*/
static inline void delay_ms(uint32_t time) {
  uint32_t initial_ticks = s_ticks;
  while (s_ticks - initial_ticks < time);  // hold until that many ticks have passed
}

static inline void delay(uint32_t time) {
  uint32_t startTime = get_time_us();
  uint32_t delayPeriod = time * 1000;
  while ((get_time_us() - startTime) < delayPeriod);
}

/**
  @brief Enable system clocks by setting frequency
  @param ticks Required frequency
*/
static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SysTick->LOAD = ticks - 1;
  SysTick->VAL = 0;
  SysTick->CTRL |= BIT(0) | BIT(1) | BIT(2);  // Enable systick, enable call back, set clk source to AHB
  s_ticks = 0;
  RCC->APB2ENR |= BIT(0);  // Enable SYSCFG
}

#pragma endregion System Clk

#pragma region GPIO
#define GPIO(bank) ((GPIO_TypeDef *)(0x48000000 + 0x400 * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

/**
  @brief Set the GPIO mode to input, output, alternate function or analog
  @param pin Selected pin
  @param mode Selected mode: GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF,
  GPIO_MODE_ANALOG
*/
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  RCC->AHB2ENR |= BIT(PINBANK(pin));        // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));          // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);    // Set new mode
}

/**
  @brief Set the GPIO to an alternate function
  @param pin Selected pin
  @param af_num Selected alternative mode
*/
static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t)af_num) << ((n & 7) * 4);
}

/**
  @brief Write to the GPIO
  @param pin Selected pin
  @param val Value to be written: True/False
*/
static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}

/**
  @brief Read from the GPIO
  @param pin Selected pin
  @return Value of the GPIO
*/
static inline bool gpio_read(uint16_t pin) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
  bool value;
  value = gpio->IDR & (1U << PINNO(pin));
  return value;
}
#pragma endregion GPIO

#pragma region UART
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3
#define LUART1 LPUART1

/**
  @brief Initialise the UART
  @param uart Selected UART (1, 2, 3 or low power)
  @param baud Baud rate
*/
static inline void uart_init(USART_TypeDef *uart, uint32_t baud) {
  uint8_t af = 8;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins

  if (uart == UART1) RCC->APB2ENR |= BIT(14);
  if (uart == UART2) RCC->APB1ENR1 |= BIT(17);
  if (uart == UART3) RCC->APB1ENR1 |= BIT(18);
  if (uart == LUART1) RCC->APB1ENR2 |= BIT(0);

#ifdef FLIGHT_COMPUTER
  // Flight Computer pins
  if (uart == UART1) af = 7, tx = PIN('A', 9), rx = PIN('A', 10);  // EXTERN USART
  if (uart == UART2) af = 7, tx = PIN('D', 5), rx = PIN('D', 6);
  if (uart == UART3) af = 7, tx = PIN('C', 10), rx = PIN('C', 11);  // GNSS RX/TX
  if (uart == UART4) af = 7, tx = PIN('D', 8), rx = PIN('D', 9);    // GNSS RX/TX
#else
  // Nucleo pins
  if (uart == UART1) af = 7, tx = PIN('A', 9), rx = PIN('A', 10);
  if (uart == UART2) af = 7, tx = PIN('A', 2), rx = PIN('A', 3);
  if (uart == UART3) af = 7, tx = PIN('B', 10), rx = PIN('B', 11);
  if (uart == LUART1) af = 8, tx = PIN('G', 7), rx = PIN('G', 8);
#endif

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;  // Disable this UART

  if (uart == LUART1 || uart == USART1) {
    uart->BRR = FREQ / baud;  // FREQ is a CPU frequency*256 when LPUART is used
  } else if (uart == USART2 || uart == USART3) {
    // uart->BRR = baud;

    uart->BRR = 0x682;

    uart->CR1 |= BIT(29);

    uart->CR2 = 0;

    uart->CR3 = 0;
    uart->CR3 |= USART_CR3_OVRDIS;
    uart->CR3 |= USART_CR3_ONEBIT;

    // uart->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    // NVIC_EnableIRQ(USART3_IRQn);
  }

  if (uart == USART3) {
    uart->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    NVIC_EnableIRQ(USART3_IRQn);  // Enable RXNE interrupt for USART3
  }

  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE Datasheet 50.8.1
}

/**
  @brief Write via UART
  @param uart Selected UART (1, 2, 3 or low power)
  @param byte Byte to be written
*/
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1);  // Ref manual STM32L4 50.8.10 USART status register (USART_ISR)
}

/**
  @brief Write to UART buffer
  @param uart Selected UART (1, 2, 3 or low power)
  @param buf Buffer
  @param len Length of the buffer
*/
static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *)buf++);
}

/**
  @brief Set UART to read
  @param uart Selected UART (1, 2, 3 or low power)
  @return True when ready
*/
static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready Ref manual 50.8.10
}

/**
  @brief Read UART
  @param uart Selected UART (1, 2, 3 or low power)
  @return Byte from UART
*/
static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  while (!uart_read_ready(uart)) {
    spin(1);  // Ref manual STM32L4 50.8.10 USART status register (USART_ISR)
  }
  return (uint8_t)(uart->RDR & 255);
}

static inline void uart_read_buf(USART_TypeDef *uart, char *results, uint8_t size) {
  uint8_t size_r = size;
  uint8_t i = 0;
  while (size_r > 0) {
    if (uart_read_ready(uart)) {
      results[i] = (uint8_t)(uart->RDR & 255);
      size_r--;
      i++;
    }
  }
}
#pragma endregion UART

#pragma region Multiplexer IC201
// Define Select pins on the multiplexer
#define A0 PIN('D', 1)
#define A1 PIN('D', 2)
#define A2 PIN('D', 3)
#define A3 PIN('D', 4)

// Map cs pins to sensors
#define CS0 0  // Accelerometer
#define CS1 1  // IMU
#define CS2 2  // Magnetometer
#define CS3 3  // Barometer
#define CS4 4  // Humidity
#define CS5 5  // SD_CARD

// Generate all switch cases for the multiplexer
static inline int set_cs(int16_t cs) {
  if (cs > 15) {
    return 1;
  } else {
    gpio_write(A0, (cs & 0x1));
    gpio_write(A1, (cs & 0x2) >> 1);
    gpio_write(A2, (cs & 0x4) >> 2);
    gpio_write(A3, (cs & 0x8) >> 3);
    return 0;
  }
}

static inline void unset_cs() {
  // get the 14 cs line low so that no sensors are active
  gpio_write(A0, LOW);
  gpio_write(A1, HIGH);
  gpio_write(A2, HIGH);
  gpio_write(A3, HIGH);
}

static inline void multiplexer_init() {
  gpio_set_mode(A0, GPIO_MODE_OUTPUT);
  gpio_set_mode(A1, GPIO_MODE_OUTPUT);
  gpio_set_mode(A2, GPIO_MODE_OUTPUT);
  gpio_set_mode(A3, GPIO_MODE_OUTPUT);
}
#pragma endregion Multiplexer IC201

#pragma region SPI
/**
  @brief Initialisation of the SPI
  @param spi Selected SPI
*/
static inline void spi_init(SPI_TypeDef *spi) {
  //  STM32L4R5 Reference manual SPI Documentation:
  //  - RM0351,  pg 78-82: Memory map and peripheral register boundary
  //  - DS10198, pg 68: Pinout
  //  - RM0351,  pg 1459: Configuration of SPI
  //  - RM0351,  pg 1484: SPI register map
  //  - RM0351,  pg 1476: SPI registers
  //  STM32L4R5 alternative functions map:
  //  https://www.st.com/resource/en/datasheet/stm32l4r5vi.pdf

  uint8_t af;
  uint16_t ss, sclk, miso, mosi;

#ifdef FLIGHT_COMPUTER
  // Flight Computer pins maybe A4 or B0 or E12 or G5 or A15
  // Note: SS not needed?
  if (spi == SPI1)
    RCC->APB2ENR |= BIT(12), af = 5, ss = PIN('A', 4), sclk = PIN('B', 3), miso = PIN('B', 4), mosi = PIN('B', 5);
  if (spi == SPI2)
    RCC->APB1ENR1 |= BIT(14), af = 5, ss = PIN('B', 12), sclk = PIN('B', 13), miso = PIN('B', 14), mosi = PIN('B', 15);

#else
  // Nucleo pins
  if (spi == SPI1)
    RCC->APB2ENR |= BIT(12), af = 5, ss = PIN('A', 4), sclk = PIN('A', 5), miso = PIN('A', 6), mosi = PIN('A', 7);
  if (spi == SPI2)
    RCC->APB1ENR1 |= BIT(14), af = 5, ss = PIN('B', 12), sclk = PIN('B', 13), miso = PIN('B', 14), mosi = PIN('B', 15);
  if (spi == SPI3)
    RCC->APB1ENR1 |= BIT(15), af = 6, ss = PIN('A', 15), sclk = PIN('C', 10), miso = PIN('C', 11), mosi = PIN('C', 12);

#endif

  // ss was originally set to GPIO_MODE_AF, which seems correct but needs to be
  // set to output to actually work? investigate !!!
  gpio_set_mode(ss, GPIO_MODE_OUTPUT);
  gpio_set_mode(sclk, GPIO_MODE_AF);
  gpio_set_mode(miso, GPIO_MODE_AF);
  gpio_set_mode(mosi, GPIO_MODE_AF);

  gpio_set_af(ss, af);
  gpio_set_af(sclk, af);
  gpio_set_af(miso, af);
  gpio_set_af(mosi, af);

  // MCU clock speed (FREQ) is 16 MHz and max MCU SPI speed is FREQ / 2.
  spi->CR1 &= ~(7U << 3);  // Clears BR (bits 5:3) to 000 which is = system clock/2
  // TODO: seems like clk is running two times faster? DEBUG
  spi->CR1 |= (3U << 3);  // Sets BR to 011, systemclk/16, so 1MHz.. but actually 2Mhz

  // CPOL (clk polarity) and CPHA (clk phase) defaults  to produce the desired
  // clock/data relationship CPOL controls the idle state value of the clock
  // when no data is being transferred.
  spi->CR1 |= BIT(0);  // Clock polarity CPOL = 1
  spi->CR1 |= BIT(1);  // Clock phase CPHA = 1

  // MCU datasheet "Select simplex or half-duplex mode by configuring
  // RXONLY or BIDIMODE and BIDIOE (RXONLY and BIDIMODE cannot be set
  // at the same time)"
  spi->CR1 &= ~BIT(10);  // full duplex
  spi->CR1 &= ~BIT(15);  // 2 line unidirectional data mode

  // Datasheet: "The MSB of a byte is transmitted first"
  spi->CR1 &= ~BIT(7);

  // Software slave management seems required
  spi->CR1 |= BIT(9);  // Manually do ss

  // Configuring the mcu as SPI master
  spi->CR1 |= BIT(2);

  // Frame size is 8 bits
  spi->CR2 |= (7U << 8);

  // Activating SS output enable
  spi->CR2 |= BIT(2);

  spi->CR2 |= BIT(12);

  // Not using TI protocol so not bothered by FRF bit
  // Not using NSSP protocol so not bothered by NSS bit
  // Not bothered by FRXTH bit. We're not reading anything on the mcu end
  // Not bothered about configuring the CRC polynomial
  // Not bothered about any DMA stuff

  // Enable the SPI!
  spi->CR1 |= BIT(6);
}

/**
  @brief Get the SPI ready for reading
  @param spi Selected SPI (1, 2 or 3)
  @return True when ready
*/
static inline int spi_ready_read(SPI_TypeDef *spi) {
  while (!(spi->SR & BIT(1)));  // Wait until transmit buffer is empty
  while (!(spi->SR & BIT(0)));  // Wait until receive buffer is not empty (RxNE, 52.4.9)
  return 1;                     // data is ready
}

static inline int spi_ready_write(SPI_TypeDef *spi) {
  while ((spi->SR & BIT(7)));  // Wait until SPI is not busy
  return 1;                    // data is ready
}

/**
  @brief Enable chip select line for spi
  @param spi Selected SPI (1, 2 or 3)
  @note currently ONLY works for spi for testing
*/
static inline void spi_enable_cs(SPI_TypeDef *spi, uint8_t cs) {
#ifdef FLIGHT_COMPUTER
  set_cs(cs);
#else  // Nucleo
  if (spi == SPI1) gpio_write(PIN('A', 4), LOW);
#endif
  // Explicitely use CS and SPI to avoid warnings, this is intended behaviour
  (void)cs;
  (void)spi;
}

/**
  @brief Enable chip select line for spi
  @param spi Selected SPI (1, 2 or 3)
  @note currently ONLY works for spi for testing
*/
static inline void spi_disable_cs(SPI_TypeDef *spi, uint8_t cs) {
#ifdef FLIGHT_COMPUTER
  unset_cs(cs);
#else  // Nucleo
  if (spi == SPI1) gpio_write(PIN('A', 4), HIGH);
#endif
  // Explicitely use CS and SPI to avoid warnings, this is intended behaviour
  (void)cs;
  (void)spi;
}

/**
  @brief Transmit single byte to and recieve a byte from SPI peripheral
  @param spi Selected SPI
  @param send_byte Byte to be sent via SPI
  @return Byte from SPI
*/
static inline uint8_t spi_transmit(SPI_TypeDef *spi, uint8_t send_byte) {
  uint8_t recieve_byte = 123;
  spi_ready_write(spi);
  *(volatile uint8_t *)&spi->DR = send_byte;

  // Since SPI is asyncronous communication, we recieve a bit as well
  spi_ready_read(spi);
  recieve_byte = *((volatile uint8_t *)&(spi->DR));
  return recieve_byte;
}

/**
  @brief Transmit multiples bytes to and recieve multiple bytes from SPI
  peripheral
  @param spi Selected SPI (1, 2 or 3)
  @param send_byte Byte to be sent via SPI
  @param transmit_size Number of bytes being sent over SPI
  @param receive_size Number of bytes being receiving over SPI
  @param result_ptr ptr to store the result
  @return not used
*/
static inline void spi_transmit_receive(SPI_TypeDef *spi, uint8_t *send_byte, uint8_t transmit_size,
                                        uint8_t receive_size, void *result_ptr) {
  spi_ready_write(spi);

  // Transmit data
  for (int i = 0; i < transmit_size; i++) {
    spi_transmit(spi, send_byte[i]);
  }

  // Receive data
  uint32_t result = 0;
  for (uint8_t rs = receive_size; rs > 0; rs--) {
    uint8_t received = spi_transmit(spi, 0x00);
    result = (result << 8) | received;
    spi_ready_write(spi);
  }

  // Store result based on receive_size
  if (receive_size == 1) {
    *(uint8_t *)result_ptr = (uint8_t)result;
  } else if (receive_size == 2) {
    *(uint16_t *)result_ptr = (uint16_t)result;
  } else if (receive_size >= 3) {
    *(uint32_t *)result_ptr = result;
  }
}

/**
  @brief Transmit single byte to and from SPI peripheral
  @param spi Selected SPI (1, 2 or 3)
  @return Byte from SPI
*/
static inline uint8_t spi_read_byte(SPI_TypeDef *spi) {
  uint8_t recieve_byte = 99;
  recieve_byte = spi_transmit(spi, 0x00);
  return recieve_byte;
}

/**
  @brief Transmit multiple bytes to and from SPI peripheral
  @param spi Selected SPI (1, 2 or 3)
  @param send_byte Byte to be sent via SPI
  @param transmit_size Number of bytes to be sent (Not currently implemented)
  @return error checking
*/
static inline uint8_t spi_write_buf(SPI_TypeDef *spi, uint8_t *send_bytes, uint8_t transmit_size) {
  for (int i = 0; i < transmit_size; i++) {
    spi_transmit(spi, send_bytes[i]);
  }
  return 0;
}

/**
  @brief Read multiple bytes from SPI peripheral
  @param spi Selected SPI (1, 2 or 3)
  @param recieve_bytes Bytes to be read over SPI
  @param receive_size Number of bytes to be read
  @note TO BE DEPRECATED
  @return value read
*/
static inline uint8_t spi_read_buf(SPI_TypeDef *spi, uint8_t *recieve_bytes, uint8_t receive_size) {
  uint8_t retval = 0;
  uint8_t i = 0;
  while (i < receive_size) {
    *(recieve_bytes + i) = spi_read_byte(spi);  // dereference to get element
    i++;
    // LOG("Received Value: %u  %u  %u \r\n", received, receive_size, result);
  }
  return retval;  // TODO error checking
}
#pragma endregion SPI

#pragma region Watchdog
/**
  @brief Set timer
  @param t Expiration time
  @param prd Period
  @param now Current time
  @return True when timer is done
*/
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

/**
  @brief Initialise the secondary power control register (Vdd2) which is needed
  for the GPIO G
*/
static inline void pwr_vdd2_init() {
  RCC->APB1ENR1 |= BIT(28);  // page 291
  PWR->CR2 |= BIT(9);        // set the IOSV bit in the PWR_CR2 page 186, 219
}

// information about watchdogs cann be found here:
// https://www.st.com/resource/en/product_training/STM32WB-WDG_TIMERS-Independent-Watchdog-IWDG.pdf

/**
  @brief Starts the watchdog timer
*/
static inline void watchdog_init() {
  // Set the IWDG_SW option bit, This is to hardware enable the watchdog instead
  // of enabling it each time like below pretty sure the option register is
  // write protected, there are steps to unlock. In reference manual: 3.4.2
  // pg141 FLASH->OPTR &= ~FLASH_OPTR_IWDG_SW;       //turn the hardware IWDG on
  // by setting bit to off FLASH->OPTR |= FLASH_OPTR_IWDG_STDBY;     //run IWDG
  // (1 turns off stand by) FLASH->OPTR |= FLASH_OPTR_IWDG_STOP;     //run IWDG
  // (1 turns off stop)

  /*The first step is to write the Key register with value 0x0000 CCCC which
    starts the watchdog. Then remove the independent watchdog register
    protection by writing 0x0000 5555 to unlock the key. Set the independent
    watchdog prescaler in the IWDG_PR register by selecting the prescaler
    divider feeding the counter clock. Write the reload register (IWDG_RLR) to
    define the value to be loaded in the watchdog counter.
  */
  IWDG->KR = 0xCCCC;
  IWDG->KR = 0x5555;
  while (IWDG->SR & ~IWDG_SR_PVU_Msk) {
  };  // prescalar can only be set when PVU bit is reset, so hold until = 0
  // IWDG->PR = 0x0001;  //Prescalar is 3 bits, 000 = /4, 001 = /8, 010 = /16,
  // 011 = /32... Divides the 32kHz clock signal
  IWDG->PR = 0x0004;
  /*
  To calculate the counter reload value to achieve the desired reset time limit
  the following formula is used: RL = (Desired_Time_ms * 32,000)/(4 * 2^PR *
  1000) -1 RL has a limit of 4095, so choose a PR to get a value less than this
  So for a 0.5s time:
  RL = (500 * 32,000)/(4 * 2^(1) * 1000) - 1 = 1999
  */
  while (IWDG->SR & ~IWDG_SR_RVU_Msk) {
  };  // reload value can only be set when RVU bit is reset, so hold until = 0
  IWDG->RLR = 0x7CF;  // 1999, value to be reloaded into the counter on reset
}

/**
  @brief Reset the watchdog timer to prevent a system reset
*/
static inline void watchdog_pat() {
  // IWDG_KR register must be written with 0x0000AAAA
  IWDG->KR = 0xAAAA;
}
#pragma endregion Watchdog
