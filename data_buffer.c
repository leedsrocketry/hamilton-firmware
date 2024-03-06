/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta, Evan Madurai
  Created on: 06 March 2023
  Description: buffer that holds data under the main routine
*/

#include "data_buffer.h"

// FIFO Buffer logic for data extraction
void update_buffer(void* reading, dataBuffer* buffer) {
  //printf("%p \r\n", *reading->pressure);
  //buffer->readings[buffer->end] = reading;
  //buffer->end = (buffer->end + 1) % BUFFER_SIZE;

  /*
  if (buffer->count < BUFFER_SIZE) {
    printf("1. Buffer count: %d\r\n", buffer->count);
    buffer->count++;
  } else {
    printf("2. Buffer start: %d\r\n", buffer->start);
    buffer->start = (buffer->start + 1) % BUFFER_SIZE;
  }
  */
}

int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

int buffer_median(dataBuffer* buffer, int start, int end)
{
    int size = end-start;
    qsort(buffer->readings, size, sizeof(int), cmpfunc);

    for(int i = 0; i < size; i++)
    {
        printf("%d\r\n", buffer->readings[i]);
    }
}