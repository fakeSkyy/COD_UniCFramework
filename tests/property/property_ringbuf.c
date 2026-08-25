/**
 * @file property_ringbuf.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util_ringbuf.h"

#define PROPERTY_SEED 0x52424B31u
#define CAPACITY 32u
#define USABLE (CAPACITY - 1u)
#define RANDOM_STEPS 50000u

typedef struct
{
    uint8_t data[USABLE];
    size_t  len;
} Reference_Queue_s;

static uint32_t rng_state = PROPERTY_SEED;

static uint32_t next_random(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static void fail_at(size_t step, const char* message)
{
    fprintf(stderr, "property_ringbuf FAIL seed=0x%08X step=%zu: %s\n", PROPERTY_SEED, step,
            message);
    exit(EXIT_FAILURE);
}

static void check_state(const UTIL_RingBuf_s* rb, const Reference_Queue_s* ref, size_t step)
{
    if (UTIL_RingBuf_Count(rb) != ref->len || UTIL_RingBuf_Free(rb) != USABLE - ref->len ||
        UTIL_RingBuf_IsEmpty(rb) != (ref->len == 0u) ||
        UTIL_RingBuf_IsFull(rb) != (ref->len == USABLE))
    {
        fail_at(step, "occupancy observers disagree with reference queue");
    }

    for (size_t i = 0u; i < ref->len; i++)
    {
        uint16_t index = (uint16_t) ((rb->tail + i) & rb->mask);
        if (rb->buffer[index] != ref->data[i])
        {
            fail_at(step, "logical FIFO contents disagree with reference queue");
        }
    }
}

static bool ref_put(Reference_Queue_s* ref, uint8_t value)
{
    if (ref->len == USABLE)
    {
        return false;
    }
    ref->data[ref->len++] = value;
    return true;
}

static bool ref_get(Reference_Queue_s* ref, uint8_t* value)
{
    if (ref->len == 0u)
    {
        return false;
    }
    *value = ref->data[0];
    memmove(ref->data, ref->data + 1, --ref->len);
    return true;
}

static uint16_t ref_put_n(Reference_Queue_s* ref, const uint8_t* data, uint16_t len)
{
    size_t accepted = len;
    if (accepted > USABLE - ref->len)
    {
        accepted = USABLE - ref->len;
    }
    memcpy(ref->data + ref->len, data, accepted);
    ref->len += accepted;
    return (uint16_t) accepted;
}

static uint16_t ref_get_n(Reference_Queue_s* ref, uint8_t* data, uint16_t len)
{
    size_t removed = len;
    if (removed > ref->len)
    {
        removed = ref->len;
    }
    memcpy(data, ref->data, removed);
    memmove(ref->data, ref->data + removed, ref->len - removed);
    ref->len -= removed;
    return (uint16_t) removed;
}

int main(void)
{
    UTIL_RingBuf_s    rb;
    uint8_t           storage[CAPACITY];
    Reference_Queue_s ref       = {{0}, 0u};
    bool              saw_wrap  = false;
    bool              saw_full  = false;
    bool              saw_flush = false;
    bool              saw_bulk  = false;

    UTIL_RingBuf_Init(&rb, storage, CAPACITY);

    for (size_t step = 0u; step < RANDOM_STEPS; step++)
    {
        uint8_t  input[48];
        uint8_t  actual[48];
        uint8_t  expected[48];
        uint16_t len = (uint16_t) (next_random() % (sizeof(input) + 1u));
        for (size_t i = 0u; i < sizeof(input); i++)
        {
            input[i] = (uint8_t) next_random();
        }
        memset(actual, 0xA5, sizeof(actual));
        memset(expected, 0xA5, sizeof(expected));

        switch (next_random() % 7u)
        {
        case 0u:
        {
            uint8_t value = (uint8_t) next_random();
            if (UTIL_RingBuf_Put(&rb, value) != ref_put(&ref, value))
            {
                fail_at(step, "Put return differs from reference");
            }
            break;
        }
        case 1u:
        {
            uint8_t actual_byte   = 0xA5u;
            uint8_t expected_byte = 0xA5u;
            bool    actual_ok     = UTIL_RingBuf_Get(&rb, &actual_byte);
            bool    expected_ok   = ref_get(&ref, &expected_byte);
            if (actual_ok != expected_ok || actual_byte != expected_byte)
            {
                fail_at(step, "Get result differs from reference");
            }
            break;
        }
        case 2u:
        {
            uint16_t actual_n   = UTIL_RingBuf_PutN(&rb, input, len);
            uint16_t expected_n = ref_put_n(&ref, input, len);
            saw_bulk            = true;
            if (actual_n != expected_n)
            {
                fail_at(step, "PutN count differs from reference");
            }
            break;
        }
        case 3u:
        {
            uint16_t actual_n   = UTIL_RingBuf_GetN(&rb, actual, len);
            uint16_t expected_n = ref_get_n(&ref, expected, len);
            saw_bulk            = true;
            if (actual_n != expected_n || memcmp(actual, expected, sizeof(actual)) != 0)
            {
                fail_at(step, "GetN bytes or untouched suffix differ from reference");
            }
            break;
        }
        case 4u:
            UTIL_RingBuf_Flush(&rb);
            ref.len   = 0u;
            saw_flush = true;
            break;
        case 5u:
            while (ref.len < USABLE)
            {
                uint8_t value = (uint8_t) next_random();
                if (!UTIL_RingBuf_Put(&rb, value) || !ref_put(&ref, value))
                {
                    fail_at(step, "forced full setup failed");
                }
            }
            saw_full = UTIL_RingBuf_IsFull(&rb) && !UTIL_RingBuf_Put(&rb, 0xEEu);
            break;
        default:
            break;
        }

        if (rb.head >= CAPACITY || rb.tail >= CAPACITY)
        {
            saw_wrap = true;
        }
        check_state(&rb, &ref, step);
    }

    if (!saw_wrap || !saw_full || !saw_flush || !saw_bulk)
    {
        fail_at(RANDOM_STEPS, "required wrap/full/flush/bulk coverage was not reached");
    }

    printf("property_ringbuf PASS seed=0x%08X steps=%u\n", PROPERTY_SEED, RANDOM_STEPS);
    return EXIT_SUCCESS;
}
