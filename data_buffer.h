/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: header file for the buffer that holds data under the main routine
*/

#ifndef BUFFER_H
#define BUFFER_H

#include "drivers/ADXL375_driver.h"
#include "drivers/MS5611_driver.h"
#include "mcu.h"

// Define Constants and Thresholds
#define BUFFER_SIZE       50
#define LAUNCH_THRESHOLD  50      // mbar for detecting a decrease
#define APOGEE_THRESHOLD  50      // mbar for detecting an increase
#define WINDOW_SIZE       10      // Number of readings to compute

// Buffer for data storing
typedef struct dataBuffer {
  void* readings[BUFFER_SIZE];  // Circular buffer
  int start;                          // Start index
  int end;                            // End index (where the next value is inserted)
  int count;                          // Number of elements currently in buffer
} dataBuffer;

void update_buffer(void* reading, dataBuffer* buffer);

int buffer_median(dataBuffer* buffer, int start, int end);

#endif /* BUFFER_H */
