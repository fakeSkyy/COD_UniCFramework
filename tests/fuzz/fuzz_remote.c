/**
 * @file fuzz_remote.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include <stddef.h>
#include <stdint.h>

#include "dev_remote.h"
#include "remote_test_stub.h"

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    UART_Instance_s uart;
    RemoteTestStub_Reset(&uart);
    DEV_Remote_s* dev = DEV_Remote_Create(&uart, 2u, 5u, 20u);
    if (dev == NULL)
    {
        return 0;
    }

    size_t cursor = 0u;
    while (cursor < size)
    {
        size_t chunk = 1u + data[cursor] % (DEV_REMOTE_FRAME_LEN * 4u);
        cursor++;
        if (chunk > size - cursor)
        {
            chunk = size - cursor;
        }
        if (chunk > 0u)
        {
            (void) RemoteTestStub_Feed(&uart, data + cursor, (uint16_t) chunk);
            cursor += chunk;
        }
        DEV_Remote_Tick(dev);
        (void) DEV_Remote_GetInput(dev);
        (void) DEV_Remote_IsLinkLost(dev);
        (void) DEV_Remote_GetFrameCount(dev);
        (void) DEV_Remote_GetErrorCount(dev);
    }
    return 0;
}
