/**
 * @file resource_probe.c
 * @author COD Framework Team
 * @date 2026/8/24
 * @version 1.0
 */

#include <stdint.h>
#include <stdio.h>

#include "util_ahrs.h"
#include "util_pid.h"
#include "util_ringbuf.h"

static UTIL_RingBuf_s ring_buffer;
static uint8_t        ring_storage[64];
static UTIL_PID_s     pid;
static UTIL_AHRS_s    ahrs;
static float          ahrs_storage[UTIL_AHRS_BUF_SIZE];

int main(void)
{
    uint8_t byte = 0u;
    UTIL_RingBuf_Init(&ring_buffer, ring_storage, (uint16_t) sizeof(ring_storage));
    if (!UTIL_RingBuf_Put(&ring_buffer, 42u) || !UTIL_RingBuf_Get(&ring_buffer, &byte))
    {
        return 2;
    }

    UTIL_PID_Cfg_s pid_config = {0};
    pid_config.kp             = 1.0f;
    pid_config.ki             = 0.1f;
    pid_config.kd             = 0.01f;
    if (!UTIL_PID_Init(&pid, &pid_config, UTIL_PID_POSITION))
    {
        return 3;
    }
    float output = UTIL_PID_Step(&pid, 1.0f, 0.25f, 0.001f);

    if (!UTIL_AHRS_Init(&ahrs, ahrs_storage, 9.794f))
    {
        return 4;
    }
    const float gyro[3]  = {0.0f, 0.0f, 0.01f};
    const float accel[3] = {0.0f, 0.0f, 9.794f};
    if (!UTIL_AHRS_Update(&ahrs, gyro, accel, 0.001f))
    {
        return 5;
    }

    printf("%u %.6f %.6f\n", (unsigned) byte, (double) output, (double) UTIL_AHRS_GetYaw(&ahrs));
    return 0;
}
