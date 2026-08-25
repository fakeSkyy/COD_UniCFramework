/**
 * @file util_assert.h
 * @author Gao Xing
 * @date 2025/5/29
 * @version 1.0
 */

#ifndef UTIL_ASSERT_H
#define UTIL_ASSERT_H

#ifdef NDEBUG

#define UTIL_ASSERT(expr) ((void) 0)

#else

#include "SEGGER_RTT.h"

#define UTIL_ASSERT(expr)                                                                          \
    do                                                                                             \
    {                                                                                              \
        if (!(expr))                                                                               \
        {                                                                                          \
            SEGGER_RTT_printf(0,                                                                   \
                              "\r\n*** ASSERT FAILED ***\r\n"                                      \
                              "  expression: %s\r\n"                                               \
                              "  file:       %s\r\n"                                               \
                              "  line:       %d\r\n",                                              \
                              #expr, __FILE__, __LINE__);                                          \
            __asm volatile("BKPT #0");                                                             \
            while (1)                                                                              \
            {                                                                                      \
            }                                                                                      \
        }                                                                                          \
    } while (0)

#endif /* NDEBUG */

#endif /* UTIL_ASSERT_H */
