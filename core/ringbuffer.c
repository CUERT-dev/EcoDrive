#include "ringbuffer.h"
#include <string.h>  // ADD THIS for memcpy

/**
 * @file
 * Implementation of ring buffer functions.
 */

void ring_buffer_init(ring_buffer_t *buffer, char *buf, size_t buf_size) {
  RING_BUFFER_ASSERT(RING_BUFFER_IS_POWER_OF_TWO(buf_size) == 1);
  buffer->buffer = buf;
  buffer->buffer_mask = buf_size - 1;
  buffer->tail_index = 0;
  buffer->head_index = 0;
}

uint8_t ring_buffer_queue(ring_buffer_t *buffer, const char *data, ring_buffer_size_t size) {
  if (size == 0) return 1;
  
  /* Calculate available space and handle overflow */
  ring_buffer_size_t available = ring_buffer_get_free_space(buffer);
  if (size > available) 
  {
    return 0;  // Buffer full
  }
  
  /* Copy data efficiently with memcpy */
  ring_buffer_size_t head = buffer->head_index;
  ring_buffer_size_t bytes_until_wrap = RING_BUFFER_SIZE(buffer) - head;
  
  if (size <= bytes_until_wrap) 
  {
    /* Single contiguous copy - no wrap around */
    memcpy(&buffer->buffer[head], data, size);
  } 
  else 
  {
    /* Two copies needed - handle buffer wrap */
    memcpy(&buffer->buffer[head], data, bytes_until_wrap);
    memcpy(buffer->buffer, data + bytes_until_wrap, size - bytes_until_wrap);
  }
  
  /* Update head index */
  buffer->head_index = ((head + size) & RING_BUFFER_MASK(buffer));  // ADD THIS LINE
  return 1;
}

ring_buffer_size_t ring_buffer_dequeue(ring_buffer_t *buffer, char *data, ring_buffer_size_t len) {
  if (ring_buffer_is_empty(buffer)) {
    return 0;
  }

  /* Calculate how much we can actually read */
  ring_buffer_size_t available = ring_buffer_num_items(buffer);
  ring_buffer_size_t bytes_to_read = (len > available) ? available : len;
  
  if (bytes_to_read == 0) return 0;
  
  /* Copy data efficiently with memcpy */
  ring_buffer_size_t tail = buffer->tail_index;
  ring_buffer_size_t bytes_until_wrap = RING_BUFFER_SIZE(buffer) - tail;
  
  if (bytes_to_read <= bytes_until_wrap) {
    /* Single contiguous copy - no wrap around */
    memcpy(data, &buffer->buffer[tail], bytes_to_read);
  } else {
    /* Two copies needed - handle buffer wrap */
    memcpy(data, &buffer->buffer[tail], bytes_until_wrap);
    memcpy(data + bytes_until_wrap, buffer->buffer, bytes_to_read - bytes_until_wrap);
  }
  
  /* Update tail index */
  buffer->tail_index = ((tail + bytes_to_read) & RING_BUFFER_MASK(buffer));
  return bytes_to_read;
}

uint8_t ring_buffer_peek(ring_buffer_t *buffer, char *data, ring_buffer_size_t index) {
  if (ring_buffer_is_empty(buffer)) {
    return 0;
  }

  /* Calculate available data */
  ring_buffer_size_t available = ring_buffer_num_items(buffer);
  if (index >= available) {
    return 0;
  }
  
  /* Calculate starting position and handle wrap-around efficiently */
  ring_buffer_size_t peek_start = ((buffer->tail_index + index) & RING_BUFFER_MASK(buffer));
  *data = buffer->buffer[peek_start];  // FIXED: Single byte peek as per original prototype
  return 1;
}

// REMOVE these lines - they're not needed since we defined them as inline in the header
extern inline uint8_t ring_buffer_is_empty(ring_buffer_t *buffer);
extern inline uint8_t ring_buffer_is_full(ring_buffer_t *buffer);
extern inline ring_buffer_size_t ring_buffer_num_items(ring_buffer_t *buffer);
extern inline ring_buffer_size_t ring_buffer_get_free_space(ring_buffer_t *buffer);