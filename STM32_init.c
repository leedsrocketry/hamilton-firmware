/*
	Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 11 June 2023
  Description: STM32L4R5 class
*/

#include "STM32_init.h"
#include "mcu.h"

/**
  @brief TODO
*/
int initHamilton()
{
    systick_init(FREQ / 1000);     // Tick every 1 ms
    initPeripherals();
    initInternals();
    return 0;
}


/**
  @brief TODO
*/
void initInternals()
{
    // UART
    systick_init(FREQ / 1000);    // Tick every 1 ms
    uart_init(LUART1, 115200);    // Initialise Low Power UART;
    uart_init(UART1,  115200);    // Initialise UART1;
    uart_init(UART2,  115200);    // Initialise UART2;
    uart_init(UART3,  115200);    // Initialise UART3;

    // I2C TODO

    // SPI TODO

    // Additional
    pwr_vdd2_init();              // Initialise VDD2 for GPIO G
}


/**
  @brief TODO
*/
void initPeripherals()
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
void ledOn()
{
    gpio_write(_blueLED, HIGH);
}


/**
  @brief Led off
*/
void ledOff()
{
    gpio_write(_blueLED, LOW);
}


/**
  @brief Buzzer sound
*/
void beepBuzzer(int onDurationMs, int offDurationMs, int noOfBeeps)
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
void indicateOnBuzzer()
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
void indicateOnLed()
{
    ledOn();
    delay(500);
    ledOff();
    delay(500);
    ledOn();
    delay(500);
    ledOff();
}


/**
  @brief Check battery charge
  @note Do not run if below TODO
*/
float getBatteryCapacity(uint8_t batteryNo)
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