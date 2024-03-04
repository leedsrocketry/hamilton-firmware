/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Oliver Martin
  Created on: 11 June 2023
  Description: STM32L4R5 class
*/

#include "STM32_init.h"
#include "mcu.h"

FREQ = (int) 4000000;

void STM32_init()
{
  systick_init(FREQ / 1000);     // Tick every 1 ms
  STM32_init_peripherals();
  STM32_init_internals();
}

void STM32_init_clock(unsigned long frequency){
  if (frequency == RCC_CFGR_SW_MSI){
    // MSI range can only be set if MSI is off, or MSI is on and MSIRDY = 1
    RCC->CR |= RCC_CR_MSION;       // set to 1 for MSI on
    while ((RCC->CR & RCC_CR_MSION) && !(RCC->CR & RCC_CR_MSIRDY)); //wait until off, or on and ready
    RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk)  | RCC_CR_MSIRANGE_11; //set MSI range to 48Hz (0b1011)
    RCC->CR |= RCC_CR_MSIRGSEL;    // set to 1 to use MSI range from CR register
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_MSI; // set system clock to MSI
    FREQ = 48000000; // 48MHz
  } else if (frequency == RCC_CFGR_SW_HSI){
    RCC->CR |= RCC_CR_HSION;       // set to 1 for HSI on
    while (!(RCC->CR & RCC_CR_HSIRDY)); // wait until HSI ready
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI; // set system clock to HSI
    FREQ = 16000000; // 16MHz
  } else {
    FREQ = 4000000; // default
  }
}

void STM32_init_internals()
{
  // UART
  systick_init(FREQ / 1000);  // Tick every 1 ms
  uart_init(LUART1, 115200);  // Initialise Low Power UART;
  uart_init(UART1,  115200);  // Initialise UART1;
  uart_init(UART2,  115200);  // Initialise UART2;
  uart_init(UART3,  115200);  // Initialise UART3;

  // SPI 
  spi_init(SPI1);

  // Additional
  pwr_vdd2_init();            // Initialise VDD2 for GPIO G
}

void STM32_init_peripherals()
{
  // Initialise the multiplexer if Flight Computer is connected
  #ifdef FLIGHT_COMPUTER
    multiplexer_init();
  #endif

  // Define inputs and outputs
  gpio_set_mode(_buzzer, GPIO_MODE_OUTPUT);
  gpio_set_mode(_blueLED, GPIO_MODE_OUTPUT);

  // Initialise
  gpio_write(_blueLED, HIGH);
  gpio_write(_buzzer, LOW);
}

void STM32_led_on()
{
  gpio_write(_blueLED, HIGH);
}

void STM32_led_off()
{
  gpio_write(_blueLED, LOW);
}
