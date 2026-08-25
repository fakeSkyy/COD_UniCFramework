/**
 * @file fuzz_ringbuf.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "util_ringbuf.h"

#define CAPACITY 64u
#define USABLE (CAPACITY - 1u)

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    uint8_t        storage[CAPACITY];
    uint8_t        reference[USABLE];
    size_t         reference_len = 0u;
    size_t         cursor        = 0u;
    UTIL_RingBuf_s rb;

    UTIL_RingBuf_Init(&rb, storage, CAPACITY);
    while (cursor < size)
    {
        uint8_t  operation = data[cursor++];
        uint16_t requested = cursor < size ? (uint16_t) (data[cursor++] % 80u) : 0u;
        size_t   available = size - cursor;
        uint16_t supplied  = requested < available ? requested : (uint16_t) available;

        switch (operation % 5u)
        {
        case 0u:
            if (supplied > 0u)
            {
                bool expected = reference_len < USABLE;
                assert(UTIL_RingBuf_Put(&rb, data[cursor]) == expected);
                if (expected)
                {
                    reference[reference_len++] = data[cursor];
                }
                cursor++;
            }
            break;
        case 1u:
        {
            uint8_t value    = 0u;
            bool    expected = reference_len > 0u;
            assert(UTIL_RingBuf_Get(&rb, &value) == expected);
            if (expected)
            {
                assert(value == reference[0]);
                memmove(reference, reference + 1, --reference_len);
            }
            break;
        }
        case 2u:
        {
            size_t expected = supplied;
            if (expected > USABLE - reference_len)
            {
                expected = USABLE - reference_len;
            }
            assert(UTIL_RingBuf_PutN(&rb, data + cursor, supplied) == expected);
            memcpy(reference + reference_len, data + cursor, expected);
            reference_len += expected;
            cursor += supplied;
            break;
        }
        case 3u:
        {
            uint8_t output[80];
            size_t  expected = requested < reference_len ? requested : reference_len;
            assert(UTIL_RingBuf_GetN(&rb, output, requested) == expected);
            assert(memcmp(output, reference, expected) == 0);
            memmove(reference, reference + expected, reference_len - expected);
            reference_len -= expected;
            break;
        }
        default:
            UTIL_RingBuf_Flush(&rb);
            reference_len = 0u;
            break;
        }

        assert(UTIL_RingBuf_Count(&rb) == reference_len);
        assert(UTIL_RingBuf_Free(&rb) == USABLE - reference_len);
    }

    return 0;
}
