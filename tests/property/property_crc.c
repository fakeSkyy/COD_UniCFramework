/**
 * @file property_crc.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util_crc.h"

#define PROPERTY_SEED 0x43524331u
#define ITERATIONS 4000u
#define MAX_PAYLOAD 512u

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

static void fail_at(size_t iteration, const char* property)
{
    fprintf(stderr, "property_crc FAIL seed=0x%08X iteration=%zu: %s\n", PROPERTY_SEED, iteration,
            property);
    exit(EXIT_FAILURE);
}

int main(void)
{
    uint8_t payload[MAX_PAYLOAD + 2u];
    uint8_t frame[MAX_PAYLOAD + 2u];

    for (size_t iteration = 0u; iteration < ITERATIONS; iteration++)
    {
        size_t   len    = next_random() % (MAX_PAYLOAD + 1u);
        uint8_t  init8  = (uint8_t) next_random();
        uint16_t init16 = (uint16_t) next_random();
        for (size_t i = 0u; i < len; i++)
        {
            payload[i] = (uint8_t) next_random();
        }

        uint8_t  expected8  = reference_crc8(payload, len, init8);
        uint16_t expected16 = reference_crc16(payload, len, init16);
        if (UTIL_CRC8_Calc(payload, len, init8) != expected8)
        {
            fail_at(iteration, "CRC-8 differs from bitwise reference");
        }
        if (UTIL_CRC16_Calc(payload, len, init16) != expected16)
        {
            fail_at(iteration, "CRC-16 differs from bitwise reference");
        }

        size_t   split     = len == 0u ? 0u : next_random() % (len + 1u);
        uint8_t  chained8  = UTIL_CRC8_Calc(payload, split, init8);
        uint16_t chained16 = UTIL_CRC16_Calc(payload, split, init16);
        if (UTIL_CRC8_Calc(payload + split, len - split, chained8) != expected8 ||
            UTIL_CRC16_Calc(payload + split, len - split, chained16) != expected16)
        {
            fail_at(iteration, "fragmented calculation differs from whole payload");
        }

        if (len > 0u)
        {
            memcpy(frame, payload, len);
            frame[len] = 0xA5u;
            if (!UTIL_CRC8_Append(frame, len + 1u) ||
                frame[len] != reference_crc8(payload, len, UTIL_CRC8_INIT) ||
                !UTIL_CRC8_Verify(frame, len + 1u))
            {
                fail_at(iteration, "CRC-8 append/verify property failed");
            }

            memcpy(frame, payload, len);
            frame[len]      = 0x5Au;
            frame[len + 1u] = 0xA5u;
            if (!UTIL_CRC16_Append(frame, len + 2u) || !UTIL_CRC16_Verify(frame, len + 2u))
            {
                fail_at(iteration, "CRC-16 append/verify property failed");
            }
            uint16_t wire_crc = (uint16_t) frame[len] | ((uint16_t) frame[len + 1u] << 8);
            if (wire_crc != reference_crc16(payload, len, UTIL_CRC16_INIT))
            {
                fail_at(iteration, "CRC-16 wire bytes differ from bitwise reference");
            }
        }
    }

    printf("property_crc PASS seed=0x%08X iterations=%u\n", PROPERTY_SEED, ITERATIONS);
    return EXIT_SUCCESS;
}
