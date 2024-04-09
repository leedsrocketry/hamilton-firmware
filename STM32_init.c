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
  // Set clock to 16MHz internal HSI
  STM32_init_clock(RCC_CFGR_SW_HSI);

  // Initialise the timer (tick every 1 us)
  uint32_t ticks_per_ms = FREQ / 1000;
  systick_init(ticks_per_ms);
  init_delay_timer();

  // Initialise system
  STM32_init_internals();
  STM32_init_peripherals();
}

void STM32_init_clock(unsigned long frequency) {
  if (frequency == RCC_CFGR_SW_MSI){
    //MSI range can only be set if MSI is off, or MSI is on and MSIRDY = 1
    RCC->CR |= RCC_CR_MSION;       //set to 1 for MSI on
    while ((RCC->CR & RCC_CR_MSION) && !(RCC->CR & RCC_CR_MSIRDY)); //wait until off, or on and ready
    RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk)  | RCC_CR_MSIRANGE_11; //set MSI range to 48Hz (0b1011)
    RCC->CR |= RCC_CR_MSIRGSEL;    //set to 1 to use MSI range from CR register
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_MSI; // set system clock to MSI
    FREQ = 48000000; //48MHz
  }else if (frequency == RCC_CFGR_SW_HSI){
    RCC->CR |= RCC_CR_HSION;       //set to 1 for HSI on
    while (!(RCC->CR & RCC_CR_HSIRDY)); //wait until HSI ready
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI; // set system clock to HSI
    FREQ = 16000000; //16MHz
  }else{
    FREQ = 4000000; //default
  }
}

void init_delay_timer() {
  // Use general purpose timer 2 which is a 32bit auto-reload timer
  RCC->APB1ENR1 = RCC_APB1ENR1_TIM2EN;
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;

  // Prescaler must make clock period = 1ns from system clock of (16MHz)
  uint32_t prescaler = FREQ/1000000 - 1; //should be 15
  TIM2->PSC = 15; 

  // Send an update event to reset the timer and apply settings.
  TIM2->EGR  |= TIM_EGR_UG;

  // Reload value
  //TIM2->ARR = 999;
  
  // Enable timer
  TIM2->CR1 = (1 << 0);
}

void STM32_init_internals()
{
  // UART
  //uart_init(LUART1, 9600);  // Initialise Low Power UART;
  uart_init(USART1, 115200);  // Initialise UART1;
  //uart_init(USART2, 115200);  // Initialise UART2;
  //uart_init(USART3, 115200);  // Initialise UART3;

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
    gpio_set_mode(GREEN_LED, GPIO_MODE_OUTPUT);
  #endif
}

void STM32_beep_buzzer(uint32_t on_duration_ms, uint32_t off_duration_ms, uint16_t nom_beeps) {
  for (int i = 0; i < nom_beeps; i++) {
    gpio_write(BUZZER, !HIGH);
    delay_ms(on_duration_ms);
    gpio_write(BUZZER, !LOW); 
    delay_ms(off_duration_ms);
  }
}

void STM32_flash_LED(uint32_t on_duration_ms, uint32_t off_duration_ms, uint16_t nom_flash) {
  for (int i = 0; i < nom_flash; i++) {
    gpio_write(GREEN_LED, !HIGH);
    delay_ms(on_duration_ms);
    gpio_write(GREEN_LED, !LOW); 
    delay_ms(off_duration_ms);
    
  }
}

void STM32_indicate_on() {
  for (int i = 0; i < 3; i++) {
    gpio_write(BUZZER, !HIGH);    // Turn on buzzer
    gpio_write(GREEN_LED, !LOW);  // Turn off LED
    delay_ms(200);
    gpio_write(BUZZER, !LOW);     // Turn off buzzer
    gpio_write(GREEN_LED, !HIGH); // Turn on LED
    delay_ms(200);
  }
}
