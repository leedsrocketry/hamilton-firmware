/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: buffer that holds data under the main routine
*/

#include "data_buffer.h"

/**
  @brief Initalise buffer
  @param buffer The buffer to be initalised
*/
void init_buffer(dataBuffer* buffer) {
  buffer->ground_ref = 0;
  buffer->index = 0;
  buffer->count = 0;

  // TODO: Allocate memory for the buffer array
  // TODO: Allocate memory for the window array
}

/**
  @brief Compare function for sorting purposes
  @param a - first element
  @param b - second element
  @return comparison value
*/
int cmpfunc(const void* a, const void* b) { return (*(int*)a - *(int*)b); }

/**
  @brief Return median of array of integers
  @param data Array of integers
  @param size Size of array
  @return True when ready
*/
int get_median(int data[], int size) {
  qsort(data, (size_t)size, sizeof(int), cmpfunc);
  if (size % 2 != 0) return data[(size / 2) + 1];
  return (data[size / 2] + data[(size / 2) + 1]) / 2;
}

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void set_ground_reference(dataBuffer* buffer) {
  // Create copy of buffer data to sort
  int _data[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++) {
    _data[i] = buffer->frames[i].barometer.pressure;
  }

  // get the ground reference as median
  buffer->ground_ref = get_median(_data, WINDOW_SIZE);
}

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void update_buffer(FrameArray* frame, dataBuffer* buffer) {
  buffer->frames[buffer->index] = *frame;
  buffer->index = (buffer->index + 1) % BUFFER_SIZE;

  // Increase count and set ground reference
  if (buffer->count < BUFFER_SIZE) {
    if (buffer->count == WINDOW_SIZE) set_ground_reference(buffer);
    buffer->count++;
  }

  // Update window
  if (buffer->count > WINDOW_SIZE * 2) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      int frame_number = (buffer->index - WINDOW_SIZE + i);
      if (frame_number < 0) frame_number = BUFFER_SIZE + frame_number;
      buffer->window[i] = buffer->frames[i];
    }
  }
}

/**
  @brief Calculate vertical velocity using JUST barometer pressure data
  @param data The array of barometer data
  @param dt Delta-time between each reading
*/
float get_vertical_velocity(int data[], int dt) {
  qsort(data, WINDOW_SIZE, sizeof(int), cmpfunc);
  float altitude_change = 0.0;
  float previous_altitude, current_altitude;

  previous_altitude =
      (float)(44330.7692 *
              (1.0 -
               pow(((float)data[0] / 100.0f) / sea_level_pressure, 0.1902)));
  current_altitude =
      (float)(44330.7692 * (1.0 - pow(((float)data[WINDOW_SIZE - 1] / 100.0f) /
                                          sea_level_pressure,
                                      0.1902)));
  altitude_change = previous_altitude - current_altitude;

  // Calculate the total time covered by the readings (microseconds)
  float total_time = ((float)dt * (float)WINDOW_SIZE * 1e-6f);
  float velocity = altitude_change / total_time;

  // Return vertical velocity in m/s
  return velocity;
}

/**
  @brief Check if rocket is stationary using JUST barometer pressure data
  @param data The array of barometer data
  @return true if the rocket is stationary
  @note This is not a particularly good solution, was only written because
  accelerometer was not working for launch 1
  @note Deprecated
*/
bool is_stationary(int data[]) {
  int sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += data[i];
  }
  int mean = sum / WINDOW_SIZE;
  int variance = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    variance += (int)pow((data[i] - mean), 2);
  }
  return sqrt(variance / WINDOW_SIZE) < GROUND_THRESHOLD;
}
