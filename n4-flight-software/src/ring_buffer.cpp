/**
 * @file ring_buffer.cpp
 * implements the function definitions for ring buffer
 */

#include "ring_buffer.h"


void ring_buffer_init(ring_buffer* r) {
    r->write_index = 0;
    r->read_index = 0;
    r->buffer_length = 0;
}

void ring_buffer_put(ring_buffer* r, float d) {
    if(r->write_index == SIZE_OF_BUFFER) {
        r->write_index = 0;
    }
    r->buffer[r->write_index++] = d;

    if(r->buffer_length >= SIZE_OF_BUFFER) {
        r->buffer_length = SIZE_OF_BUFFER;
    } else {
        r->buffer_length++;
    }

}

float ring_buffer_get(ring_buffer* r) {
    if(r->read_index == SIZE_OF_BUFFER) {
        r->read_index = 0;
    }
    r->buffer_length--;
    return r->buffer[r->read_index++];
}

uint8_t ring_buffer_available(ring_buffer* r) {
    return r->buffer_length;
}

uint8_t ring_buffer_empty(ring_buffer* r) {
    return (r->buffer_length == 0);
}

uint8_t ring_buffer_full(ring_buffer* r) {
    return (r->buffer_length == SIZE_OF_BUFFER);
}

void ring_buffer_flush(ring_buffer* r) {
    r->buffer_length = 0;
    r->read_index=0;
    r->write_index = 0;
    memset(r, 0, sizeof(r->buffer));
}


