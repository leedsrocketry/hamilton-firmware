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
#define LAUNCH_THRESHOLD  50      // micro bar for detecting a decrease
#define GROUND_THRESHOLD  30      // micro bar for detecting ground
#define WINDOW_SIZE       10      // Number of readings to compute

// Circular Buffer for data storing
typedef struct dataBuffer {
  FrameArray frames[BUFFER_SIZE];     // Circular buffer
  FrameArray window[WINDOW_SIZE];     // Last window readings
  int ground_ref;                     // Set of reference values for launch
  int index;                          // End index (value is inserted)
  int count;                          // Number of elements currently in buffer
} dataBuffer;

void init_buffer(dataBuffer* buffer);

/**
  @brief Get the median of the data
  @param data - array of data
  @param size - size of the array
  @return median value
*/
int get_median(int data[], int size);

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void update_buffer(FrameArray* frame, dataBuffer* buffer);


/**
  @brief Get the last window of data
  @param buffer - data buffer
  @param window - array of frames that should be returned


void get_last_window(dataBuffer* buffer, FrameArray window[]);
*/
#endif /* BUFFER_H */
