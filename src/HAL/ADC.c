/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 20 August 2024
    Description: STM32L4 ADC Driver
*/

#include "ADC.h"

/**
  @brief Initialisation of the ADC
  @param ADCx: The ADC peripheral to use
  @param adc_pin: The pin to use for the ADC
*/
static inline void ADC_init(ADC_TypeDef* ADCx, uint16_t adc_pin) {
  // Set clock source for ADC
  RCC->AHB2ENR |=  (RCC_AHB2ENR_ADCEN);         // ADC clock enable        
  RCC->CCIPR   &= ~(RCC_CCIPR_ADCSEL);          // Clear ADCSEL bits
  RCC->CCIPR   |=  (3 << RCC_CCIPR_ADCSEL_Pos); // Set ADCSEL bits to 11 (System clock selected as ADCs clock)

  // Set GPIO pin to analog mode
  gpio_set_mode(adc_pin, GPIO_MODE_ANALOG);

  // Set ADC to a single conversation sequence
  // First, set the number of channels to read during each sequence.
  // (# of channels = L + 1, so set L to 0)
  ADCx->SQR1  &= ~( ADC_SQR1_L );
  // Configure the first (and only) step in the sequence to read channel 6.
  ADCx->SQR1  &= ~( 0x1F << 6 );
  ADCx->SQR1  |=  ( 6 << 6 );
  // Configure the sampling time to 640.5 cycles.
  ADCx->SMPR1 &= ~( 0x7 << ( 6 * 3 ) );
  ADCx->SMPR1 |=  ( 0x7 << ( 6 * 3 ) );
}

/**
  @brief Perform a single ADC conversion. A value of zero means that the ADC reads 
  zero volts, and a value of 4095 means that the ADC reads the supply voltage, which
  is 3.0V in this case
  @param ADCx: The ADC peripheral to use
  @return The converted ADC value on a scale of 0 – 4095, because it is a 12-bit ADC
*/
uint16_t ADC_single_conversion(ADC_TypeDef* ADCx) {
  // Start the ADC conversion
  ADCx->CR |= (ADC_CR_ADSTART);
  // Wait for the 'End Of Conversion' (EOC) flag
  while(!(ADCx->ISR & ADC_ISR_EOC)) {};
  // Read the converted value (this also clears the EOC flag)
  uint16_t adc_val = ADCx->DR;
  // Wait for the 'End Of Sequence' flag and clear it
  while (!(ADCx->ISR & ADC_ISR_EOS)) {};
  ADCx->ISR |= (ADC_ISR_EOS);
  // Return the ADC value
  return adc_val;
}

/**
  @brief Return battery voltage in volts
  @params ADCx: The ADC peripheral to use
*/
float ADC_get_battery_voltage(ADC_TypeDef* ADCx) {
  // Perform a single ADC conversion
  uint16_t adc_val = ADC_single_conversion(ADCx);

  // Convert the ADC value to a voltage
  float voltage = (adc_val / 4095.0) * 3.0;
  return voltage;
}