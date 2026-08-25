/**
 * @file util_ringbuf.c
 * @author Gao Xing
 * @date 2025/7/23
 * @version 1.0
 */

#include "util_ringbuf.h"

void UTIL_RingBuf_Init(UTIL_RingBuf_s* rb, uint8_t* buf, uint16_t capacity)
{
    rb->buffer   = buf;
    rb->head     = 0;
    rb->tail     = 0;
    rb->capacity = capacity;
    rb->mask     = (uint16_t) (capacity - 1u);
}

void UTIL_RingBuf_Flush(UTIL_RingBuf_s* rb)
{
    rb->head = 0;
    rb->tail = 0;
}

bool UTIL_RingBuf_Put(UTIL_RingBuf_s* rb, uint8_t byte)
{
    if (UTIL_RingBuf_IsFull(rb))
    {
        return false;
    }
    rb->buffer[rb->head & rb->mask] = byte;
    rb->head++;
    return true;
}

bool UTIL_RingBuf_Get(UTIL_RingBuf_s* rb, uint8_t* byte)
{
    if (UTIL_RingBuf_IsEmpty(rb))
    {
        return false;
    }
    *byte = rb->buffer[rb->tail & rb->mask];
    rb->tail++;
    return true;
}

uint16_t UTIL_RingBuf_PutN(UTIL_RingBuf_s* rb, const uint8_t* data, uint16_t len)
{
    uint16_t free = UTIL_RingBuf_Free(rb);
    if (len > free)
    {
        len = free;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        rb->buffer[(rb->head + i) & rb->mask] = data[i];
    }
    rb->head = (uint16_t) (rb->head + len); /* publish payload before advancing head */
    return len;
}

uint16_t UTIL_RingBuf_GetN(UTIL_RingBuf_s* rb, uint8_t* data, uint16_t len)
{
    uint16_t avail = UTIL_RingBuf_Count(rb);
    if (len > avail)
    {
        len = avail;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = rb->buffer[(rb->tail + i) & rb->mask];
    }
    rb->tail = (uint16_t) (rb->tail + len);
    return len;
}
