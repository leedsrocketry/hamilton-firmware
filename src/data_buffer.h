/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: header file for the buffer that holds data under the main routine
*/

#ifndef BUFFER_H
#define BUFFER_H

#include <stdio.h>
#include <stdlib.h>

#include "HAL/mcu.h"
#include "debug.h"
#include "drivers/ADXL375_driver.h"
#include "drivers/MS5611_driver.h"
#include "frame_array.h"

// Define Constants and Thresholds
#define BUFFER_SIZE 50
#define LAUNCH_THRESHOLD 50   // micro bar for detecting a decrease
#define APOGEE_THRESHOLD 50   // micro bar for detecting apogee
#define DESCENT_THRESHOLD 50  // micro bar for detecting an increase
#define GROUND_THRESHOLD 100  // micro bar for detecting ground
#define WINDOW_SIZE 20        // Number of readings to compute

static float sea_level_pressure = 1013.25;  // Sea level presser in micro bar

// Circular Buffer for data storing
typedef struct dataBuffer {
  FrameArray frames[BUFFER_SIZE];  // Circular buffer
  FrameArray window[WINDOW_SIZE];  // Last window readings
  int ground_ref;                  // Set of reference values for launch
  int index;                       // End index (value is inserted)
  int count;                       // Number of elements currently in buffer
} dataBuffer;

/**
  @brief Initialize the buffer
  @param buffer - data buffer
*/
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
  @brief Calculate vertical velocity using JUST barometer pressure data
  @param data The array of barometer data
  @param dt Delta-time between each reading
*/
float get_vertical_velocity(int barometer_data[], int dt);

/**
  @brief Check if rocket is stationary using JUST barometer pressure data
  @param data The array of barometer data
  @return true if the rocket is stationary
  @note This is not a particularly good solution, was only written because
  accelerometer was not working for launch 1
*/

bool is_stationary(int data[]);

#endif /* BUFFER_H */
