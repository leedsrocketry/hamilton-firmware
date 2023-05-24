/*
	Leeds University Rocketry Organisation - LURA
    Author Name: Alexandra Posta
    Created on: 27 Feb 2023
	  Last modified on: 24 May 2023
    Description: entry point for the HFC firmware; suitable for STM32L4R5
*/

#include "mcu.h"
#include "NAND_flash_driver.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

int main(void) {
  uint16_t led = PIN('B', 7);                         // Blue LED
  systick_init(FREQ / 1000);                          // Tick every 1 ms
  gpio_set_mode(led, GPIO_MODE_OUTPUT);               // Set blue LED to output mode
  
  pwr_vdd2_init();
  uart_init(LUART1, 115200);                          // Initialise UART; 

  static bool on = 1; 
  gpio_write(led, on); 

  printf("--- PROGRAM START ---\r\n");

  initialiseFlash();
  //eraseALL();
  frameAddressPointer = 0;

  for (;;) {
    //setControlPins(WRITE_PROTECT);  // Write Protection
    //setControlPins(WRITE_PROTECT_OFF);  // Write Protection Off
    
    uint8_t dataArray[128];
    _memset(dataArray, 0x0, 128);

    for(uint8_t i = 0; i < 128; i ++){
      dataArray[i] = i;
    }
    //eraseBlock(0);
    //eraseALL();
    //writeFrame(0, dataArray);
    //readFrame(10000, dataArray);
    
    frameArray _input = unzip(dataArray);
    // frameArray _output;

    //int data_intact = 0;
    //int data_fixed = 0;
    //int data_error = 0;
  
    //int startAddr = frameAddressPointer;
    int numOfFramesToTest = 100;
    
    for(int i = 0; i < numOfFramesToTest; i++){
      for(uint8_t j = 0; j < 128; j ++){
        dataArray[j] = j;
      } 

      dataArray[0] = 0;
      dataArray[1] = 0;
      _input = unzip(dataArray);

      logFrame(_input);

      printf("-------DONE--------");
    }  
    
    printf("DONE WRITING\r\n");
    readALL();
    printCapacityInfo();
    printf("--- PROGRAM END ---"); 
  }
  return 0;
}