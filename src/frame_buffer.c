/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: buffer that holds data under the main routine
*/

#include "frame_buffer.h"

/**
  @brief Initalise buffer
  @param buffer The buffer to be initalised
*/
void init_frame_buffer(FrameBuffer *fb)
{
  fb->ground_ref = 0;
  fb->index = 0;
  fb->count = 0;
}

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void set_ground_reference(FrameBuffer *fb)
{
  // get the ground reference as median
  fb->ground_ref = get_framebuffer_median(fb, BUFFER_SIZE, ALTITUDE);
}

/**
  @brief Fills the circular buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void update_frame_buffer(Frame *frame, FrameBuffer *fb)
{
  fb->frames[fb->index] = *frame;
  fb->index = (fb->index + 1) % BUFFER_SIZE;

  // Increase count and set ground reference
  if (fb->count < BUFFER_SIZE)
  {
    if (fb->count == WINDOW_SIZE)
    {
      set_ground_reference(fb);
    }
    fb->count++;
  }

  // // Update window
  // if (buffer->count > WINDOW_SIZE * 2) {
  //   for (int i = 0; i < WINDOW_SIZE; i++) {
  //     int frame_number = (buffer->index - WINDOW_SIZE + i);
  //     if (frame_number < 0) frame_number = BUFFER_SIZE + frame_number;
  //     buffer->window[i] = buffer->frames[i];
  //   }
  // }
}

Frame *get_window_0(FrameBuffer *fb)
{
  // Window 0: the last 20 readings
  uint32_t start_idx = (fb->index + BUFFER_SIZE - WINDOW_SIZE) % BUFFER_SIZE;
  return &(fb->frames[start_idx]);
}

uint32_t get_window_0_index(FrameBuffer *fb)
{
  uint32_t start_idx = (fb->index + BUFFER_SIZE - WINDOW_SIZE) % BUFFER_SIZE;
  return start_idx;
}

Frame *get_window_1(FrameBuffer *fb)
{
  // Window 1: the 20 readings before the last 20 readings
  uint32_t start_idx = (fb->index + BUFFER_SIZE - 2 * WINDOW_SIZE) % BUFFER_SIZE;
  return &(fb->frames[start_idx]);
}

uint32_t get_window_1_index(FrameBuffer *fb)
{
  uint32_t start_idx = (fb->index + BUFFER_SIZE - 2 * WINDOW_SIZE) % BUFFER_SIZE;
  return start_idx;
}

void write_framebuffer(FrameBuffer *fb)
{
  for (uint32_t i = 0; i < fb->count; i++)
  {
    log_frame(fb->frames[i]);
  }
}

double get_framebuffer_median(FrameBuffer *fb, uint32_t size, SensorReading reading)
{
  double _data[size]; // Adjusted to the size of the window you want to calculate the median for

  // Determine the start index based on the most recent frames
  uint32_t start_idx = (fb->index + BUFFER_SIZE - size) % BUFFER_SIZE;

  // Populate _data based on the desired sensor reading
  for (uint32_t i = 0; i < size; i++)
  {
    uint32_t idx = (start_idx + i) % BUFFER_SIZE; // Ensure correct circular indexing
    switch (reading)
    {
    case MS5611_PRESSURE:
      _data[i] = (double)fb->frames[idx].barometer.pressure;
      break;

    case MS5611_TEMP:
      _data[i] = (double)fb->frames[idx].barometer.temp;
      break;

    case ALTITUDE:
      _data[i] = fb->frames[idx].altitude;
      break;
    }
  }

  // Calculate and return the median
  return get_median(_data, size);
}

/**
  @brief Compare function for sorting purposes
  @param a - first element
  @param b - second element
  @return comparison value
*/
int cmpfunc(const void *a, const void *b) { return (*(int *)a - *(int *)b); }

/**
  @brief Return median of array of integers
  @param data Array of integers
  @param size Size of array
  @return True when ready
*/
double get_median(double data[], uint32_t size)
{
  qsort(data, (size_t)size, sizeof(int), cmpfunc);
  if (size % 2 != 0)
    return data[(size / 2) + 1];
  return (double)(data[size / 2] + data[(size / 2) + 1]) / 2;
}

bool is_stationary(FrameBuffer *fb) {
    LSM6DS3_data data[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++) {
        data[i] = fb->frames[i].imu; 
    }

    if (LSM6DS3_gyro_standard_dev(data, WINDOW_SIZE, 2200)) {
        return true;  
    } else {
        return false;
    }
}


// /**
//   @brief Check if rocket is stationary using JUST barometer pressure data
//   @param data The array of barometer data
//   @return true if the rocket is stationary
//   @note This is not a particularly good solution, was only written because
//   accelerometer was not working for launch 1
//   @note Deprecated
// */
// bool is_stationary(int32_t data[]) {
//   int32_t sum = 0;
//   for (int32_t i = 0; i < WINDOW_SIZE; i++) {
//     sum += data[i];
//   }
//   int32_t mean = sum / WINDOW_SIZE;
//   int32_t variance = 0;
//   for (int32_t i = 0; i < WINDOW_SIZE; i++) {
//     variance += (int32_t)pow((data[i] - mean), 2);
//   }
//   return sqrt(variance / WINDOW_SIZE) < GROUND_THRESHOLD;
// }