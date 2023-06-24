/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 11 June 2023
  Description: STM32L4R5 class
*/

#include "STM32_init.h"
#include "mcu.h"


// Pins
const uint16_t _vBatt   = PIN('A', 0);  
const uint16_t _vBatt1  = PIN('A', 0); 
const uint16_t _vBatt2  = PIN('A', 0); 
const uint16_t _vBatt3  = PIN('A', 0); 
const uint16_t _buzzer  = PIN('A', 0); 
const uint16_t _blueLED = PIN('B', 7);


/**
  @brief Initialisation of the STM32L5 board
*/
void init_STM32()
{
  systick_init(FREQ / 1000);     // Tick every 1 ms
  init_peripherals();
  init_internals();
}


/**
  @brief Initialisation of the STM32L4R5 board internals (UART, SPI, Power, etc.)
*/
void init_internals()
{
  // UART
  systick_init(FREQ / 1000);  // Tick every 1 ms
  uart_init(LUART1, 115200);  // Initialise Low Power UART;
  uart_init(UART1,  115200);  // Initialise UART1;
  uart_init(UART2,  115200);  // Initialise UART2;
  uart_init(UART3,  115200);  // Initialise UART3;

  // SPI TODO

  // Additional
  pwr_vdd2_init();            // Initialise VDD2 for GPIO G
}


/**
  @brief Initialisation of the STM32L5 board externals (GPIO, ADC, etc.)
*/
void init_peripherals()
{
  // Define inputs and outputs
  gpio_set_mode(_buzzer, GPIO_MODE_OUTPUT);
  gpio_set_mode(_blueLED, GPIO_MODE_OUTPUT);

  // Initialise
  gpio_write(_blueLED, HIGH);
  gpio_write(_buzzer, LOW);
}


/**
  @brief Led on
*/
void led_on()
{
  gpio_write(_blueLED, HIGH);
}


/**
  @brief Led off
*/
void led_off()
{
  gpio_write(_blueLED, LOW);
}


/**
  @brief Buzzer sound
*/
void beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps)
{
  for (int i = 0; i < noOfBeeps; i++) {
      gpio_write(_buzzer, HIGH);
      delay(onDurationMs);
      gpio_write(_buzzer, LOW); 
      delay(offDurationMs);
  }
}


/**
  @brief Buzzer sound to indicate power on
*/
void indicate_on_buzzer()
{
  gpio_write(_buzzer, HIGH);
  delay(300);
  gpio_write(_buzzer, LOW);
  delay(300);
  gpio_write(_buzzer, HIGH);
  delay(300);
  gpio_write(_buzzer, LOW);
}


/**
  @brief Led light to indicate power on
*/
void indicate_on_led()
{
  led_on();
  delay(500);
  led_off();
  delay(500);
  led_on();
  delay(500);
  led_off();
}


/**
  @brief Check battery charge
  @note Do not run if below TODO
*/
double get_battery_capacity(uint8_t batteryNo)
{
  switch (batteryNo)
  {
      case 1:
          return _vBatt*3.3/1023-0.07;
      case 2:
          return _vBatt1*3.3/1023-0.07;
      case 3:
          return _vBatt2*3.3/1023-0.07;
      case 4:
          return _vBatt3*3.3/1023-0.07;
      default:
          return -1;
  }
}
