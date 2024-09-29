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

#include "frame.h"
#include "HAL/NAND_flash_driver.h"

// Define Constants and Thresholds
#define BUFFER_SIZE 60
#define WINDOW_SIZE BUFFER_SIZE/2  // Number of readings to calculate computations

// Circular Buffer for data storing
typedef struct FrameBuffer {
  Frame frames[BUFFER_SIZE];  // Circular buffer
  Frame *window_0;
  Frame *window_1;
  double ground_ref;                  // Set of reference values for launch
  uint32_t index;                       // End index (value is inserted)
  uint32_t count;                       // Number of elements currently in buffer
} FrameBuffer;

/**
  @brief Initialize the buffer
  @param buffer - data buffer
*/
void init_frame_buffer(FrameBuffer* buffer);

double get_framebuffer_median(FrameBuffer* fb, uint32_t size, SensorReading sensor);

void write_framebuffer(FrameBuffer* fb);

/**
  @brief Get the median of the data
  @param data - array of data
  @param size - size of the array
  @return median value
*/
double get_median(double data[], uint32_t size);

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void update_frame_buffer(Frame* frame, FrameBuffer* buffer);

/**
  @brief Calculate vertical velocity using JUST barometer pressure data
  @param fb frame
  @param dt Delta-time between each reading
*/
float get_vertical_velocity(FrameBuffer *fb);

/**
  @brief Check if rocket is stationary using JUST barometer pressure data
  @param data The array of barometer data
  @return true if the rocket is stationary
  @note This is not a particularly good solution, was only written because
  accelerometer was not working for launch 1
*/

bool is_stationary(FrameBuffer *fb);

#endif /* BUFFER_H */
