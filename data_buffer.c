/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: buffer that holds data under the main routine
*/

#include "data_buffer.h"

int cmpfunc(const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

void init_buffer(dataBuffer* buffer) {
  buffer->ground_ref = 0;
  buffer->index = 0;
  buffer->count = 0;

  // Allocate memory for the buffer
  //buffer->frames = (FrameArray*)malloc(BUFFER_SIZE * 128);
}

// Set the values for ground level
void set_ground_reference(dataBuffer* buffer) {
  // Create copy of buffer data to sort
  int _data[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++) {
    _data[i] = buffer->frames[i].barometer;
  }
  qsort(_data, WINDOW_SIZE, sizeof(int), cmpfunc);

  if (sizeof(_data) % 2 != 0)
    buffer->ground_ref = _data[(WINDOW_SIZE / 2) + 1];
  buffer->ground_ref = (_data[WINDOW_SIZE / 2] + _data[(WINDOW_SIZE / 2)+ 1]) / 2; 
}

// Circular Buffer logic to update data
void update_buffer(FrameArray frame, dataBuffer* buffer) {
  buffer->frames[buffer->index] = frame;
  buffer->index = (buffer->index + 1) % BUFFER_SIZE;

  // Increase count and set ground reference
  if (buffer->count < BUFFER_SIZE) {
    if (buffer->count == WINDOW_SIZE) 
      set_ground_reference(buffer);
    buffer->count++;
  }
}

int buffer_median(dataBuffer* buffer, int start, int end)
{
    int size = end - start;
    qsort(buffer->frames, size, sizeof(int), cmpfunc);

    for(int i = 0; i < size; i++)
    {
        printf("%d\r\n", buffer->frames[i]);
    }
}