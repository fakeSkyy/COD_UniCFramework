/**
 * @file fuzz_crc.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "util_crc.h"

static uint8_t reference_crc8(const uint8_t* data, size_t len, uint8_t init)
{
    uint8_t crc = init;
    for (size_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (unsigned bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 1u) != 0u ? (uint8_t) ((crc >> 1) ^ 0x8Cu) : (uint8_t) (crc >> 1);
        }
    }
    return crc;
}

static uint16_t reference_crc16(const uint8_t* data, size_t len, uint16_t init)
{
    uint16_t crc = init;
    for (size_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (unsigned bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 1u) != 0u ? (uint16_t) ((crc >> 1) ^ 0x8408u) : (uint16_t) (crc >> 1);
        }
    }
    return crc;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    uint8_t        init8       = size > 0u ? data[0] : 0u;
    uint16_t       init16      = size > 2u ? (uint16_t) data[1] | ((uint16_t) data[2] << 8) : 0u;
    const uint8_t* payload     = size > 3u ? data + 3u : data + size;
    size_t         payload_len = size > 3u ? size - 3u : 0u;

    assert(UTIL_CRC8_Calc(payload, payload_len, init8) ==
           reference_crc8(payload, payload_len, init8));
    assert(UTIL_CRC16_Calc(payload, payload_len, init16) ==
           reference_crc16(payload, payload_len, init16));

    if (payload_len > 0u && payload_len <= 4096u)
    {
        uint8_t* frame = malloc(payload_len + 2u);
        if (frame != NULL)
        {
            memcpy(frame, payload, payload_len);
            assert(UTIL_CRC16_Append(frame, payload_len + 2u));
            assert(UTIL_CRC16_Verify(frame, payload_len + 2u));
            free(frame);
        }
    }
    return 0;
}
