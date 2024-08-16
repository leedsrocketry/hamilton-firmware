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
void init_frame_buffer(FrameBuffer* fb) {
  fb->ground_ref = 0;
  fb->index = 0;
  fb->count = 0;
  
  fb->window_0 = &(fb->frames[0]);
  fb->window_1 = &(fb->frames[BUFFER_SIZE / 2]);
}

void write_framebuffer(FrameBuffer* fb)
{
  for(int32_t i = 0; i < fb->count; i++ )
  {
    log_frame(fb->frames[i]);
  }
}

int32_t get_framebuffer_median(FrameBuffer* fb, uint32_t size, SensorReading reading)
{
  int32_t _data[BUFFER_SIZE];
  switch(reading)
  {
    case MS5611_PRESSURE:
      // Calculate median of pressure in fb
      for(uint32_t i = 0; i < BUFFER_SIZE; i++)
      {
        _data[i] = fb->frames[i].barometer.pressure;
      }
      get_median(_data, BUFFER_SIZE);
    break;
    
    case MS5611_TEMP:
      // Calculate median of temp in fb
      for(uint32_t i = 0; i < BUFFER_SIZE; i++)
      {
        _data[i] = fb->frames[i].barometer.temp;
      }
      get_median(_data, BUFFER_SIZE);
    break;

    case ALTITUDE:
      for(uint32_t i = 0; i < BUFFER_SIZE; i++)
      {
        _data[i] = fb->frames[i].altitude;
      }
      get_median(_data, BUFFER_SIZE);
    break;
  }
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
uint32_t get_median(int32_t data[], uint32_t size) {
  qsort(data, (size_t)size, sizeof(int), cmpfunc);
  if (size % 2 != 0) return data[(size / 2) + 1];
  return (data[size / 2] + data[(size / 2) + 1]) / 2;
}

/**
  @brief Set the ground reference for the buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void set_ground_reference(FrameBuffer* fb) {
  // get the ground reference as median
  fb->ground_ref = get_framebuffer_median(fb, BUFFER_SIZE, ALTITUDE);
}

/**
  @brief Fills the circular buffer
  @param frame - one reading data frame to add
  @param buffer - data buffer
*/
void update_frame_buffer(Frame* frame, FrameBuffer* fb) {
  fb->frames[fb->index] = *frame;
  fb->index = (fb->index + 1) % BUFFER_SIZE;

  // Increase count and set ground reference
  if (fb->count < BUFFER_SIZE) {
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

float get_vertical_velocity(FrameBuffer fb, int dt) {
    float data[WINDOW_SIZE];
    float data_prev[WINDOW_SIZE];

    // fill data arrays from windows
    for(uint32_t i = 0; i < WINDOW_SIZE; i++)
    {
        data[i] = fb.window_1[i].altitude;
        data_prev[i] = fb.window_0[i].altitude;
    }

    // calc average altitudes
    float avg_current = 0;
    float avg_previous = 0;
    for(uint32_t i = 0; i < WINDOW_SIZE; i++)
    {
        avg_current += data[i];
        avg_previous += data_prev[i];
    }
    avg_current /= WINDOW_SIZE;
    avg_previous /= WINDOW_SIZE;

    // calc vertical velocity
    float velocity = (avg_current - avg_previous) / dt;

    return velocity;
}

// /**
//   @brief Calculate vertical velocity using JUST barometer pressure data
//   @param data The array of barometer data
//   @param dt Delta-time between each reading
// */
// float get_vertical_velocity(int data[], int dt) {
//   qsort(data, WINDOW_SIZE, sizeof(int), cmpfunc);
//   float altitude_change = 0.0;
//   float previous_altitude, current_altitude;

//   previous_altitude =
//       (float)(44330.7692 *
//               (1.0 -
//                pow(((float)data[0] / 100.0f) / sea_level_pressure, 0.1902)));
//   current_altitude =
//       (float)(44330.7692 * (1.0 - pow(((float)data[WINDOW_SIZE - 1] / 100.0f) /
//                                           sea_level_pressure,
//                                       0.1902)));
//   altitude_change = previous_altitude - current_altitude;

//   // Calculate the total time covered by the readings (microseconds)
//   float total_time = ((float)dt * (float)WINDOW_SIZE * 1e-6f);
//   float velocity = altitude_change / total_time;

//   // Return vertical velocity in m/s
//   return velocity;
// }

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