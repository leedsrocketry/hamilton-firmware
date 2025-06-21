/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 27 Feb 2023
  Description: Entry point for the HFC firmware; suitable for STM32L4R5
*/

#include <stdio.h>

#include "../segger-rtt/RTT/SEGGER_RTT.h"
#include "HAL/mcu.h"
#include "debug.h"
#include "drivers/_driver_manager.h"
#include "flight_manager.h"
#include "frame.h"
#include "lib/log.h"
#include "stm32l4r5xx.h"
#include "drivers/MAXM10S_driver.h"

volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

#define BUFFER_SIZE 62 // Example size based on your log
#define SYNC_BYTE_1 0xB5
#define SYNC_BYTE_2 0x62

volatile uint8_t usart3_buffer[BUFFER_SIZE];
volatile uint8_t usart3_buf_idx = 0;

typedef enum {
    STATE_WAIT_FOR_SYNC1,
    STATE_WAIT_FOR_SYNC2,
    STATE_RECEIVING_PAYLOAD
} UsartReceiveState_t;

volatile UsartReceiveState_t usart3_rx_state = STATE_WAIT_FOR_SYNC1;

void USART3_IRQHandler(void) __attribute__((used));
void USART3_IRQHandler(void) {
  // Check if RXNE flag is set
  if (USART3->ISR & BIT(5)) {
    uint8_t b = uart_read_byte(USART3);

    switch (usart3_rx_state) {
        case STATE_WAIT_FOR_SYNC1:
            if (b == SYNC_BYTE_1) {
                usart3_buffer[0] = b; // Store the first sync byte
                usart3_buf_idx = 1;
                usart3_rx_state = STATE_WAIT_FOR_SYNC2;
            } else {
                // If not SYNC_BYTE_1, discard and stay in this state
                usart3_buf_idx = 0; // Ensure index is reset
            }
            break;

        case STATE_WAIT_FOR_SYNC2:
            if (b == SYNC_BYTE_2) {
                usart3_buffer[1] = b; // Store the second sync byte
                usart3_buf_idx = 2;
                usart3_rx_state = STATE_RECEIVING_PAYLOAD;
            } else {
                // If not SYNC_BYTE_2, reset state and look for SYNC_BYTE_1 again
                usart3_rx_state = STATE_WAIT_FOR_SYNC1;
                usart3_buf_idx = 0;
            }
            break;

        case STATE_RECEIVING_PAYLOAD:
            if (usart3_buf_idx < BUFFER_SIZE) {
                usart3_buffer[usart3_buf_idx++] = b;
            }

            if (usart3_buf_idx >= BUFFER_SIZE) {
                // Copy the actual hex (raw bytes) to hex_str
                uint8_t hex_str[BUFFER_SIZE];
                for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
                    hex_str[i] = usart3_buffer[i];
                }
                // Pass the raw bytes to PARSE_NAV_PVT
                // PARSE_NAV_PVT(hex_str); // Uncomment when ready

                MAX10M10S_data data;
                PARSE_NAV_PVT(hex_str, &data);

                // Log each byte as a two-digit hex string
                // #ifdef DEBUG
                // char log_buf[BUFFER_SIZE * 3 + 1] = {0};
                // for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
                //     sprintf(&log_buf[i * 3], "%02X ", hex_str[i]);
                // }
                // logi("%s\r\n", log_buf);
                // #endif

                // Reset for the next message
                usart3_buf_idx = 0;
                usart3_rx_state = STATE_WAIT_FOR_SYNC1;
            }
            break;
    }
  }

  // Check for overrun error
  if (USART3->ISR & USART_ISR_ORE) {
    // Clear the overrun error flag by reading the data register again
    volatile uint16_t _temp = USART3->RDR;
    (void)_temp;
  }
}


/**
  @brief Main entry point for the Hamilton Flight Computer (HFC) firmware
*/

int main(void) {
  // STM32 setup
  STM32_init();

  logi("================ PROGRAM START ================\r\n");
  STM32_indicate_on();

  logi("============ INITIALISE NAND FLASH ============\r\n");
  init_flash();
  print_capacity_info();

#ifdef ERASE_NAND
  erase_all();
  NAND_flash_read();
  delay_ms(100);
  return 0;
#endif

#ifdef READ_NAND
  delay_ms(1000);
  print_capacity_info();
  NAND_flash_read();
  return 0;
#endif

  logi("============== INITIALISE DRIVERS =============\r\n");
  initalise_drivers();

#ifdef SENSOR_TEST
  test_sensors();
#endif

#ifdef CALIBRATE
  logi("Getting calibration data...\r\n");
  calibrate_ADXL375();
  return 0;
#endif

  logi("============== INITIALISE FLIGHT ==============\r\n");
  delay_ms(2000);
  STM32_super_beep();
  run_flight();

  return 0;
}
