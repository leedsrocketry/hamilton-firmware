#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdio.h>

#include "frame.h"

typedef struct {
  Frame** buffer;         // Pointer to the buffer array
  uint32_t head;          // Index of the oldest element
  uint32_t tail;          // Index where the next element will be inserted
  uint32_t max_size;      // Maximum number of elements in the buffer
  uint32_t current_size;  // Current number of elements in the buffer
} CircularBuffer;

// Function prototypes
CircularBuffer* cb_create(uint32_t size);
void cb_destroy(CircularBuffer* cb);
uint32_t cb_is_empty(const CircularBuffer* cb);
uint32_t cb_enqueue_overwrite(CircularBuffer* cb, Frame* element);
Frame* cb_dequeue(CircularBuffer* cb);
Frame* cb_peek(const CircularBuffer* cb);
uint32_t cb_size(const CircularBuffer* cb);
void cb_clear(CircularBuffer* cb);
int32_t cb_average(CircularBuffer* cb, Frame* frame);
uint32_t cb_pressure_range(const CircularBuffer* cb);

#endif  // CIRCULAR_BUFFER_H
