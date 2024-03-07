/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Oliver Martin
  Created on: 11 June 2023
  Description: STM32L4R5 class
*/

#include "STM32_init.h"

FREQ = (int) 4000000;

void STM32_init()
{
  systick_init(FREQ / 1000);     // Tick every 1 ms
  STM32_init_internals();
  STM32_init_peripherals();
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

void init_delay_timer(){
  //use general purpose timer 2 which is a 32bit auto-reload timer
  RCC->APB1ENR1 = RCC_APB1ENR1_TIM2EN;

  RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;

  //prescaler must make clock period = 1ns from system clock of (16MHz)
  uint32_t prescaler = FREQ/1000000 - 1; //should be 15
  TIM2->PSC = 15; 

  // Send an update event to reset the timer and apply settings.
  TIM2->EGR  |= TIM_EGR_UG;

  //reload value
  //TIM2->ARR = 999;
  
  //enable timer
  TIM2->CR1 = (1 << 0);
  
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

    // LED/BUZZER
    gpio_set_mode(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_mode(BLUE_LED, GPIO_MODE_OUTPUT);
  #endif
}

void STM32_beep_buzzer(uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfBeeps) {
  for (int i = 0; i < noOfBeeps; i++) {
    // ! because it seems as it works the other way?
    gpio_write(BUZZER, !HIGH);
    delay_ms(onDurationMs);
    gpio_write(BUZZER, !LOW); 
    delay_ms(offDurationMs);
  }
}

void STM32_flash_LED (uint32_t onDurationMs, uint32_t offDurationMs, uint16_t noOfFlash) {
  for (int i = 0; i < noOfFlash; i++) {
    gpio_write(BLUE_LED, !HIGH);
    delay_ms(onDurationMs);
    gpio_write(BLUE_LED, !LOW); 
    delay_ms(offDurationMs);
  }
}

void STM32_indicate_on() {
  STM32_beep_buzzer(200, 200, 3);
  STM32_flash_LED(200, 200, 3);
}
