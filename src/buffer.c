#include "buffer.h"

#include <stdlib.h>
#include <string.h>

#include "frame.h"

// Weights for WMA calculation of 20 elements
static int32_t weights[] = {0,  2,  4,  6,  8,  10, 12, 14, 16, 18,
                            20, 22, 24, 26, 28, 30, 32, 34, 36, 38};

CircularBuffer* cb_create(uint32_t size) {
  // Allocate memory for the circular buffer structure
  CircularBuffer* cb = malloc(sizeof(CircularBuffer));
  if (cb == NULL) {
    return NULL;
  }

  // Allocate memory for the buffer array
  cb->buffer = malloc(size * sizeof(Frame*));
  if (cb->buffer == NULL) {
    free(cb);
    return NULL;
  }

  // Initialize buffer properties
  cb->max_size = size;
  cb->current_size = 0;
  cb->head = 0;
  cb->tail = 0;

  return cb;
}

int32_t cb_average_pressure(CircularBuffer* cb) {
  // Check if the buffer is empty
  if (cb_is_empty(cb)) {
    return 0;
  }

  // Calculate the average of the elements in the buffer
  int32_t sum = 0;
  int32_t sum_of_weights = 0;

  for (uint32_t i = 0; i < cb->current_size; i++) {
    // Get the index of the current element
    uint32_t index = (cb->head + i) % cb->max_size;
    Frame* frame = cb->buffer[index];

    int32_t weight = weights[i];
    sum_of_weights += weight;
    // Apply the weight to the current element
    sum += weight * frame->barometer.pressure;
    // sum += frame->barometer.pressure;
  }

  // int32_t average = sum / (int32_t)(cb->current_size);
  int32_t average = sum / (int32_t)(sum_of_weights);

  // Store the average in the first element of the buffer
  return average;
}

int32_t cb_average_temp(CircularBuffer* cb) {
  // Check if the buffer is empty
  if (cb_is_empty(cb)) {
    return 0;
  }

  // Calculate the average of the elements in the buffer
  int32_t sum = 0;

  for (uint32_t i = 0; i < cb->current_size; i++) {
    // Get the index of the current element
    uint32_t index = (cb->head + i) % cb->max_size;
    Frame* frame = cb->buffer[index];
    int32_t weight = weights[i];
    // Apply the weight to the current element
    sum += weight * frame->barometer.temp;
    // sum += frame->barometer.temp;
  }

  int32_t average = sum / (int32_t)(cb->current_size);

  // Store the average in the first element of the buffer
  return average;
}

void cb_destroy(CircularBuffer* cb) {
  if (cb != NULL) {
    // Free the buffer array
    free(cb->buffer);
    // Free the circular buffer structure
    free(cb);
  }
}

uint32_t cb_is_empty(const CircularBuffer* cb) {
  return (cb->current_size == 0);
}

uint32_t cb_enqueue_overwrite(CircularBuffer* cb, Frame* element) {
  // If buffer is full, overwrite the oldest element
  if (cb->current_size == cb->max_size) {
    // Move head forward, remove the oldest element
    cb->head = (cb->head + 1) % cb->max_size;
    cb->current_size--;
  }

  // Insert element at tail
  cb->buffer[cb->tail] = element;

  // Move tail, wrapping around if necessary
  cb->tail = (cb->tail + 1) % cb->max_size;

  // Increment current size (if not already at max)
  if (cb->current_size < cb->max_size) {
    cb->current_size++;
  }

  return 1;
}

Frame* cb_dequeue(CircularBuffer* cb) {
  // Check if buffer is empty
  if (cb_is_empty(cb)) {
    return NULL;
  }

  Frame* element = cb->buffer[cb->head];

  cb->head = (cb->head + 1) % cb->max_size;

  cb->current_size--;

  return element;
}

Frame* cb_peek(const CircularBuffer* cb) {
  if (cb_is_empty(cb)) {
    return NULL;
  }

  return cb->buffer[cb->head];
}

uint32_t cb_size(const CircularBuffer* cb) { return cb->current_size; }

void cb_clear(CircularBuffer* cb) {
  // Reset head, tail, and current size
  cb->head = 0;
  cb->tail = 0;
  cb->current_size = 0;
}
