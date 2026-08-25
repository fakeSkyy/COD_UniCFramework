/**
 * @file property_remote.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev_remote.h"
#include "remote_test_stub.h"

#define PROPERTY_SEED 0x44425553u
#define FRAME_COUNT 256u
#define ENDPOINT_FRAME_COUNT 2u
#define LOST_TICKS 1000u

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

static void fail_at(size_t offset, const char* property)
{
    fprintf(stderr, "property_remote FAIL seed=0x%08X stream_offset=%zu: %s\n", PROPERTY_SEED,
            offset, property);
    exit(EXIT_FAILURE);
}

static int16_t random_between(int32_t low, int32_t high)
{
    return (int16_t) (low + (int32_t) (next_random() % (uint32_t) (high - low + 1)));
}

static void encode_frame(const DEV_Remote_Input_s* input, uint8_t frame[DEV_REMOTE_FRAME_LEN])
{
    uint16_t raw[5];
    for (size_t i = 0u; i < 5u; i++)
    {
        raw[i] = (uint16_t) (input->ch[i] + 1024);
    }

    memset(frame, 0, DEV_REMOTE_FRAME_LEN);
    frame[0] = (uint8_t) raw[0];
    frame[1] = (uint8_t) ((raw[0] >> 8) | (raw[1] << 3));
    frame[2] = (uint8_t) ((raw[1] >> 5) | (raw[2] << 6));
    frame[3] = (uint8_t) (raw[2] >> 2);
    frame[4] = (uint8_t) ((raw[2] >> 10) | (raw[3] << 1));
    frame[5] =
        (uint8_t) ((raw[3] >> 7) | ((uint16_t) input->sw[0] << 4) | ((uint16_t) input->sw[1] << 6));
    frame[6]  = (uint8_t) input->mouse_x;
    frame[7]  = (uint8_t) ((uint16_t) input->mouse_x >> 8);
    frame[8]  = (uint8_t) input->mouse_y;
    frame[9]  = (uint8_t) ((uint16_t) input->mouse_y >> 8);
    frame[10] = (uint8_t) input->mouse_z;
    frame[11] = (uint8_t) ((uint16_t) input->mouse_z >> 8);
    frame[12] = input->mouse_l ? 1u : 0u;
    frame[13] = input->mouse_r ? 1u : 0u;
    frame[14] = (uint8_t) input->key_bits;
    frame[15] = (uint8_t) (input->key_bits >> 8);
    frame[16] = (uint8_t) raw[4];
    frame[17] = (uint8_t) (raw[4] >> 8);
}

static void check_input(const DEV_Remote_s* dev, const DEV_Remote_Input_s* expected, size_t offset)
{
    const DEV_Remote_Input_s* actual = DEV_Remote_GetInput(dev);
    if (memcmp(actual->ch, expected->ch, sizeof(actual->ch)) != 0 ||
        memcmp(actual->sw, expected->sw, sizeof(actual->sw)) != 0 ||
        actual->mouse_x != expected->mouse_x || actual->mouse_y != expected->mouse_y ||
        actual->mouse_z != expected->mouse_z || actual->key_bits != expected->key_bits ||
        actual->mouse_l != expected->mouse_l || actual->mouse_r != expected->mouse_r)
    {
        fail_at(offset, "decoded snapshot differs from expected legal frame");
    }

    if (DEV_Remote_IsLinkLost(dev) ||
        DEV_Remote_IsKeyDown(dev, DEV_KEY_W) != ((expected->key_bits & 1u) != 0u) ||
        DEV_Remote_IsKeyDown(dev, DEV_KEY_MOUSE_L) != expected->mouse_l)
    {
        fail_at(offset, "link or key state differs from latest frame");
    }
}

/**
 * @brief Exercise every accepted DBUS field endpoint before randomized streaming.
 */
static void check_endpoint_vectors(DEV_Remote_s* dev, UART_Instance_s* uart)
{
    const DEV_Remote_Input_s vectors[ENDPOINT_FRAME_COUNT] = {
        {
            .ch = {-DEV_REMOTE_CH_MAX, -DEV_REMOTE_CH_MAX, -DEV_REMOTE_CH_MAX, -DEV_REMOTE_CH_MAX,
                   -DEV_REMOTE_CH_MAX},
            .sw = {1u, 3u},
            .mouse_x  = -32000,
            .mouse_y  = -32000,
            .mouse_z  = -32000,
            .key_bits = 0x0001u,
            .mouse_l  = true,
            .mouse_r  = false,
        },
        {
            .ch       = {DEV_REMOTE_CH_MAX, DEV_REMOTE_CH_MAX, DEV_REMOTE_CH_MAX, DEV_REMOTE_CH_MAX,
                         DEV_REMOTE_CH_MAX},
            .sw       = {3u, 1u},
            .mouse_x  = 32000,
            .mouse_y  = 32000,
            .mouse_z  = 32000,
            .key_bits = 0x8000u,
            .mouse_l  = false,
            .mouse_r  = true,
        },
    };

    for (size_t i = 0u; i < ENDPOINT_FRAME_COUNT; i++)
    {
        uint8_t frame[DEV_REMOTE_FRAME_LEN];
        encode_frame(&vectors[i], frame);
        if (!RemoteTestStub_Feed(uart, frame, DEV_REMOTE_FRAME_LEN))
        {
            fail_at(i * DEV_REMOTE_FRAME_LEN, "endpoint vector did not use registered callback");
        }
        DEV_Remote_Tick(dev);
        check_input(dev, &vectors[i], (i + 1u) * DEV_REMOTE_FRAME_LEN);
        if (DEV_Remote_GetFrameCount(dev) != i + 1u || DEV_Remote_GetErrorCount(dev) != 0u)
        {
            fail_at((i + 1u) * DEV_REMOTE_FRAME_LEN,
                    "endpoint vector was not accepted exactly once");
        }
    }
}

int main(void)
{
    uint8_t            stream[FRAME_COUNT * DEV_REMOTE_FRAME_LEN];
    DEV_Remote_Input_s expected[FRAME_COUNT];
    UART_Instance_s    uart;

    for (size_t frame_index = 0u; frame_index < FRAME_COUNT; frame_index++)
    {
        DEV_Remote_Input_s* input = &expected[frame_index];
        for (size_t channel = 0u; channel < 5u; channel++)
        {
            input->ch[channel] = random_between(-DEV_REMOTE_CH_MAX, DEV_REMOTE_CH_MAX);
        }
        input->sw[0]    = (uint8_t) (1u + next_random() % 3u);
        input->sw[1]    = (uint8_t) (1u + next_random() % 3u);
        input->mouse_x  = random_between(-32000, 32000);
        input->mouse_y  = random_between(-32000, 32000);
        input->mouse_z  = (int16_t) next_random();
        input->key_bits = (uint16_t) next_random();
        input->mouse_l  = (next_random() & 1u) != 0u;
        input->mouse_r  = (next_random() & 1u) != 0u;
        encode_frame(input, stream + frame_index * DEV_REMOTE_FRAME_LEN);
    }

    RemoteTestStub_Reset(&uart);
    DEV_Remote_s* dev = DEV_Remote_Create(&uart, 2u, 5u, LOST_TICKS);
    if (dev == NULL || !DEV_Remote_IsLinkLost(dev))
    {
        fail_at(0u, "device creation or initial lost state failed");
    }

    check_endpoint_vectors(dev, &uart);

    size_t offset              = 0u;
    size_t previously_complete = 0u;
    while (offset < sizeof(stream))
    {
        size_t remaining = sizeof(stream) - offset;
        size_t chunk     = 1u + next_random() % (DEV_REMOTE_FRAME_LEN * 4u);
        if (chunk > remaining)
        {
            chunk = remaining;
        }
        if (!RemoteTestStub_Feed(&uart, stream + offset, (uint16_t) chunk))
        {
            fail_at(offset, "bottom stub had no registered receive callback");
        }
        offset += chunk;
        DEV_Remote_Tick(dev);

        size_t complete = offset / DEV_REMOTE_FRAME_LEN;
        if (complete > previously_complete)
        {
            check_input(dev, &expected[complete - 1u], offset);
            if (DEV_Remote_GetFrameCount(dev) != ENDPOINT_FRAME_COUNT + complete)
            {
                fail_at(offset, "accepted frame count differs from complete stream frames");
            }
            previously_complete = complete;
        }
    }

    if (DEV_Remote_GetErrorCount(dev) != 0u ||
        DEV_Remote_GetFrameCount(dev) != ENDPOINT_FRAME_COUNT + FRAME_COUNT)
    {
        fail_at(offset, "legal stream produced parser errors or lost frames");
    }

    for (size_t tick = 1u; tick < LOST_TICKS; tick++)
    {
        DEV_Remote_Tick(dev);
        if (DEV_Remote_IsLinkLost(dev))
        {
            fail_at(offset, "link became lost before timeout tick N");
        }
    }
    DEV_Remote_Tick(dev);
    if (!DEV_Remote_IsLinkLost(dev) || DEV_Remote_GetInput(dev)->sw[0] != 0u ||
        DEV_Remote_GetInput(dev)->ch[0] != 0)
    {
        fail_at(offset, "timeout tick N did not publish the neutral lost-link state");
    }

    printf("property_remote PASS seed=0x%08X frames=%u endpoint_frames=%u\n", PROPERTY_SEED,
           FRAME_COUNT, ENDPOINT_FRAME_COUNT);
    return EXIT_SUCCESS;
}
