#include <inttypes.h>
#include <stddef.h>
#include <assert.h>
#include <stdint.h>  // ADD THIS
/**
 * @file
 * Prototypes and structures for the ring buffer module.
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

#define RING_BUFFER_ASSERT(x) assert(x)

/**
 * Checks if the buffer_size is a power of two.
 * Due to the design only <tt> RING_BUFFER_SIZE-1 </tt> items
 * can be contained in the buffer.
 * buffer_size must be a power of two.
*/
#define RING_BUFFER_IS_POWER_OF_TWO(buffer_size) ((buffer_size & (buffer_size - 1)) == 0)

/**
 * The type which is used to hold the size
 * and the indicies of the buffer.
 */
typedef size_t ring_buffer_size_t;

/**
 * Used as a modulo operator
 * as <tt> a % b = (a & (b − 1)) </tt>
 * where \c a is a positive index in the buffer and
 * \c b is the (power of two) size of the buffer.
 */
#define RING_BUFFER_MASK(rb) (rb->buffer_mask)

/**
 * Returns the size of the ring buffer
 */
#define RING_BUFFER_SIZE(rb) (rb->buffer_mask + 1)  // ADD THIS

/**
 * Simplifies the use of <tt>struct ring_buffer_t</tt>.
 */
typedef struct ring_buffer_t ring_buffer_t;

/**
 * Structure which holds a ring buffer.
 * The buffer contains a buffer array
 * as well as metadata for the ring buffer.
 */
struct ring_buffer_t {
  /** Buffer memory. */
  uint8_t *buffer;
  /** Buffer mask. */
  ring_buffer_size_t buffer_mask;
  /** Index of tail. */
  volatile ring_buffer_size_t tail_index;
  /** Index of head. */
  volatile ring_buffer_size_t head_index;
};

/**
 * Initializes the ring buffer pointed to by <em>buffer</em>.
 * This function can also be used to empty/reset the buffer.
 * The resulting buffer can contain <em>buf_size-1</em> bytes.
 * @param buffer The ring buffer to initialize.
 * @param buf The buffer allocated for the ringbuffer.
 * @param buf_size The size of the allocated ringbuffer.
 */
void ring_buffer_init(ring_buffer_t *buffer, uint8_t *buf, size_t buf_size);

/**
 * Adds an array of bytes to a ring buffer.
 * @param buffer The buffer in which the data should be placed.
 * @param data A pointer to the array of bytes to place in the queue.
 * @param size The size of the array.
 * @return 1 if successful; 0 if buffer full.
 */
uint8_t ring_buffer_queue(ring_buffer_t *buffer, const uint8_t *data, ring_buffer_size_t size);

/**
 * Returns the <em>len</em> oldest bytes in a ring buffer.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the array at which the data should be placed.
 * @param len The maximum number of bytes to return.
 * @return The number of bytes returned.
 */
ring_buffer_size_t ring_buffer_dequeue(ring_buffer_t *buffer, uint8_t *data, ring_buffer_size_t len);

/**
 * Peeks a ring buffer, i.e. returns an element without removing it.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the location at which the data should be placed.
 * @param index The index to peek.
 * @return 1 if data was returned; 0 otherwise.
 */
uint8_t ring_buffer_peek(ring_buffer_t *buffer, uint8_t *data, ring_buffer_size_t index);

/**
 * Returns whether a ring buffer is empty.
 * @param buffer The buffer for which it should be returned whether it is empty.
 * @return 1 if empty; 0 otherwise.
 */
inline uint8_t ring_buffer_is_empty(ring_buffer_t *buffer) {
  return (buffer->head_index == buffer->tail_index);
}

/**
 * Returns whether a ring buffer is full.
 * @param buffer The buffer for which it should be returned whether it is full.
 * @return 1 if full; 0 otherwise.
 */
inline uint8_t ring_buffer_is_full(ring_buffer_t *buffer) {
  return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK(buffer)) == RING_BUFFER_MASK(buffer);
}

/**
 * Returns the number of items in a ring buffer.
 * @param buffer The buffer for which the number of items should be returned.
 * @return The number of items in the ring buffer.
 */
inline ring_buffer_size_t ring_buffer_num_items(ring_buffer_t *buffer) {
  return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK(buffer));
}

/**
 * Returns the free space in a ring buffer.
 * @param buffer The buffer for which the free space should be returned.
 * @return The number of free bytes in the ring buffer.
 */
inline ring_buffer_size_t ring_buffer_get_free_space(ring_buffer_t *buffer) {  // FIX THIS FUNCTION
  return RING_BUFFER_SIZE(buffer) - ring_buffer_num_items(buffer) - 1;  // -1 because full condition leaves 1 byte unused
}

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H */