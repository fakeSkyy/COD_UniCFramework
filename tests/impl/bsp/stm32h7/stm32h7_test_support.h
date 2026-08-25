/**
 * @file stm32h7_test_support.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef STM32H7_TEST_SUPPORT_H
#define STM32H7_TEST_SUPPORT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include "mock_impl_memory.h"
#include "mock_stm32h7xx_hal.h"
#include "test_support.h"

/** Storage aligned for any implementation context used by a host suite. */
typedef union
{
    max_align_t align;
    uint8_t     bytes[512];
} STM32H7_Test_Storage_u;

/** Map one emulated memory-mapped peripheral or flash window. */
static inline void* STM32H7_Test_Map(uintptr_t address, size_t len, int fill)
{
    void* mapped = mmap((void*) address, len, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

    TEST_ASSERT_EQUAL_PTR((void*) address, mapped);
    memset(mapped, fill, len);
    return mapped;
}

/** Start both generated mocks with an empty CMock allocation arena. */
static inline void STM32H7_Test_MockInit(void)
{
    mock_stm32h7xx_hal_Init();
    mock_impl_memory_Init();
}

/* CTest launches every selected case in a fresh process. This isolates the
 * production backends' append-only routing tables without test-only reset hooks. */
#define STM32H7_TEST_MAIN_BEGIN()                                                                  \
    int main(int argc, char** argv)                                                                \
    {                                                                                              \
        bool matched = false;                                                                      \
        UNITY_BEGIN();

#define STM32H7_RUN_TEST(test_name)                                                                \
    do                                                                                             \
    {                                                                                              \
        if (argc == 1 || strcmp(argv[1], #test_name) == 0)                                         \
        {                                                                                          \
            RUN_TEST(test_name);                                                                   \
            matched = true;                                                                        \
        }                                                                                          \
    } while (0)

#define STM32H7_TEST_MAIN_END()                                                                    \
    int unity_result = UNITY_END();                                                                \
    return matched ? unity_result : 2;                                                             \
    }

/** Verify all expectations before releasing CMock's shared allocation arena. */
static inline void STM32H7_Test_MockVerify(void)
{
    mock_stm32h7xx_hal_Verify();
    mock_impl_memory_Verify();
    mock_stm32h7xx_hal_Destroy();
    mock_impl_memory_Destroy();
}

#endif /* STM32H7_TEST_SUPPORT_H */
