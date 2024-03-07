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
#include <stdio.h>
#include <stdlib.h>
#include "mcu.h"

// Define Constants and Thresholds
#define BUFFER_SIZE       50
#define LAUNCH_THRESHOLD  50      // mbar for detecting a decrease
#define APOGEE_THRESHOLD  50      // mbar for detecting an increase
#define WINDOW_SIZE       10      // Number of readings to compute

// Circular Buffer for data storing
typedef struct dataBuffer {
  FrameArray frames[BUFFER_SIZE];     // Circular buffer
  int ground_ref;                     // Set of reference values for launch
  int index;                          // End index (value is inserted)
  int count;                          // Number of elements currently in buffer
} dataBuffer;

void init_buffer(dataBuffer* buffer);

void update_buffer(FrameArray frame, dataBuffer* buffer);

int buffer_median(dataBuffer* buffer, int start, int end);

#endif /* BUFFER_H */
