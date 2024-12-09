/**
 * @file ring_buffer.h
 * Implements a ring buffer for apogee detection
 */

#ifndef N4_FLIGHT_SOFTWARE_RING_BUFFER_H
#define N4_FLIGHT_SOFTWARE_RING_BUFFER_H

#include <Arduino.h>
#include <cstring>

#define SIZE_OF_BUFFER 5

typedef struct {
    float buffer[SIZE_OF_BUFFER];
    uint8_t write_index;
    uint8_t read_index;
    uint8_t buffer_length;

} ring_buffer;

void ring_buffer_init(ring_buffer*);
void ring_buffer_put(ring_buffer*, float);
float ring_buffer_get(ring_buffer* r);
uint8_t ring_buffer_available(ring_buffer* r);
uint8_t ring_buffer_full(ring_buffer* r);
uint8_t ring_buffer_empty(ring_buffer* r);
void ring_buffer_flush(ring_buffer* r);

#endif
