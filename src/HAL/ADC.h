/*
    Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 20 August 2024
    Description: STM32L4 ADC Driver
*/

#define ADC1				    ADC1_BASE
#define ADC2				    ADC2_BASE

/****************************************************************************/
/* ADC_SMPRx ADC Sample Time Selection for Channels */
/** @defgroup adc_sample ADC Sample Time Selection values
@ingroup adc_defines

@{*/
#define ADC_SMPR_SMP_2DOT5CYC		0x0
#define ADC_SMPR_SMP_6DOT5CYC		0x1
#define ADC_SMPR_SMP_12DOT5CYC		0x2
#define ADC_SMPR_SMP_24DOT5CYC		0x3
#define ADC_SMPR_SMP_47DOT5CYC		0x4
#define ADC_SMPR_SMP_92DOT5CYC		0x5
#define ADC_SMPR_SMP_247DOT5CYC		0x6
#define ADC_SMPR_SMP_640DOT5CYC		0x7
/**@}*/
