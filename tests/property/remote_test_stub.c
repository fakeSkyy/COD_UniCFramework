/**
 * @file remote_test_stub.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */

#include "remote_test_stub.h"

#include <stddef.h>
#include <string.h>

#include "dev_remote.h"
#include "plat_memory.h"

#define REMOTE_TEST_ARENA_SIZE 4096u

typedef union
{
    max_align_t align;
    uint8_t     bytes[REMOTE_TEST_ARENA_SIZE];
} Remote_Test_Arena_u;

static Remote_Test_Arena_u arena;
static bool                arena_used;

void* PLAT_malloc(size_t size)
{
    if (arena_used || size > sizeof(arena.bytes))
    {
        return NULL;
    }

    arena_used = true;
    memset(arena.bytes, 0, sizeof(arena.bytes));
    return arena.bytes;
}

void PLAT_free(void* ptr)
{
    (void) ptr;
    arena_used = false;
}

void PLAT_UART_OnReceive(UART_Instance_s* uart, PLAT_UART_RxCallback cb) { uart->rx_cb = cb; }

bool PLAT_UART_StartReceive(UART_Instance_s* uart, uint8_t* buf, uint16_t size)
{
    return uart != NULL && buf != NULL && size == DEV_REMOTE_RX_BUF_SIZE;
}

void RemoteTestStub_Reset(UART_Instance_s* uart)
{
    memset(uart, 0, sizeof(*uart));
    memset(&arena, 0, sizeof(arena));
    arena_used = false;
}

bool RemoteTestStub_Feed(UART_Instance_s* uart, const uint8_t* data, uint16_t len)
{
    if (uart->rx_cb == NULL)
    {
        return false;
    }

    uart->rx_cb(uart, data, len);
    return true;
}
