/**
 * @file util_ringbuf.h
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

#ifndef UTIL_RINGBUF_H
#define UTIL_RINGBUF_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief A fixed-capacity byte ring buffer over caller-owned storage.
 *
 * Storage is caller-owned (static is recommended), so this module performs no
 * dynamic allocation and stays hardware-independent. Capacity MUST be a power
 * of two so the wrap-around reduces to a cheap mask.
 *
 * Concurrency (single-core, e.g. Cortex-M): this is a single-producer /
 * single-consumer (SPSC) queue. One context may Put (the producer, typically an
 * ISR) while another context Gets (the consumer, typically a task) without a
 * lock: the producer only advances @c head and the consumer only advances
 * @c tail, and both are volatile so each side observes the other's progress.
 * Do NOT drive Put from two contexts, or Get from two contexts, without an
 * external lock.
 */
typedef struct
{
    uint8_t*          buffer;   /**< Caller-owned storage (>= capacity bytes). */
    volatile uint16_t head;     /**< Write index (producer only).             */
    volatile uint16_t tail;     /**< Read index (consumer only).              */
    uint16_t          capacity; /**< Total capacity, a power of two.          */
    uint16_t          mask;     /**< capacity - 1, for fast wrap-around.      */
} UTIL_RingBuf_s;

/**
 * @brief Initialize a ring buffer over caller-provided storage.
 * @param rb        Ring buffer to initialize.
 * @param buf       Caller-owned storage of at least @p capacity bytes.
 * @param capacity  Storage size in bytes; MUST be a power of two.
 */
void UTIL_RingBuf_Init(UTIL_RingBuf_s* rb, uint8_t* buf, uint16_t capacity);

/**
 * @brief Number of bytes currently buffered.
 * @param rb  Ring buffer.
 * @return Count of readable bytes.
 */
static inline uint16_t UTIL_RingBuf_Count(const UTIL_RingBuf_s* rb)
{
    return (uint16_t) ((rb->head - rb->tail) & rb->mask);
}

/**
 * @brief Number of free bytes available for writing.
 * @param rb  Ring buffer.
 * @return Count of writable bytes (one slot is reserved to distinguish
 *         full from empty).
 */
static inline uint16_t UTIL_RingBuf_Free(const UTIL_RingBuf_s* rb)
{
    return (uint16_t) (rb->capacity - UTIL_RingBuf_Count(rb) - 1u);
}

/**
 * @brief Test whether the buffer holds no bytes.
 * @param rb  Ring buffer.
 * @return true when empty.
 */
static inline bool UTIL_RingBuf_IsEmpty(const UTIL_RingBuf_s* rb) { return rb->head == rb->tail; }

/**
 * @brief Test whether the buffer cannot accept more bytes.
 * @param rb  Ring buffer.
 * @return true when full.
 */
static inline bool UTIL_RingBuf_IsFull(const UTIL_RingBuf_s* rb)
{
    return UTIL_RingBuf_Free(rb) == 0u;
}

/**
 * @brief Discard all buffered bytes.
 * @param rb  Ring buffer.
 * @note Not safe to call concurrently with Put/Get; use only when both the
 *       producer and consumer are quiescent.
 */
void UTIL_RingBuf_Flush(UTIL_RingBuf_s* rb);

/**
 * @brief Append one byte (producer side).
 * @param rb    Ring buffer.
 * @param byte  Byte to store.
 * @return true on success, false if the buffer was full (byte dropped).
 */
bool UTIL_RingBuf_Put(UTIL_RingBuf_s* rb, uint8_t byte);

/**
 * @brief Remove one byte (consumer side).
 * @param rb    Ring buffer.
 * @param byte  Destination for the byte read.
 * @return true on success, false if the buffer was empty.
 */
bool UTIL_RingBuf_Get(UTIL_RingBuf_s* rb, uint8_t* byte);

/**
 * @brief Append up to @p len bytes (producer side).
 *
 * Writes as many bytes as fit; excess bytes are dropped. Publishes @c head only
 * after the payload is stored, so a concurrent consumer never reads a slot that
 * has not been written yet.
 *
 * @param rb    Ring buffer.
 * @param data  Source bytes.
 * @param len   Number of bytes offered.
 * @return Number of bytes actually stored (<= @p len).
 */
uint16_t UTIL_RingBuf_PutN(UTIL_RingBuf_s* rb, const uint8_t* data, uint16_t len);

/**
 * @brief Remove up to @p len bytes (consumer side).
 * @param rb    Ring buffer.
 * @param data  Destination buffer.
 * @param len   Maximum bytes to read.
 * @return Number of bytes actually read (<= @p len).
 */
uint16_t UTIL_RingBuf_GetN(UTIL_RingBuf_s* rb, uint8_t* data, uint16_t len);

#endif /* UTIL_RINGBUF_H */
