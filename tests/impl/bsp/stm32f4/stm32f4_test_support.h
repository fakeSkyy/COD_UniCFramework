/**
 * @file stm32f4_test_support.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef STM32F4_TEST_SUPPORT_H
#define STM32F4_TEST_SUPPORT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include "mock_impl_memory.h"
#include "mock_stm32f4xx_hal.h"
#include "test_support.h"

typedef union
{
    max_align_t align;
    uint8_t     bytes[512];
} STM32F4_Test_Storage_u;

static inline void* STM32F4_Test_Map(uintptr_t address, size_t len, int fill)
{
    void* mapped = mmap((void*) address, len, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    TEST_ASSERT_EQUAL_PTR((void*) address, mapped);
    memset(mapped, fill, len);
    return mapped;
}

static inline void STM32F4_Test_MockInit(void)
{
    mock_stm32f4xx_hal_Init();
    mock_impl_memory_Init();
}

static inline void STM32F4_Test_MockVerify(void)
{
    mock_stm32f4xx_hal_Verify();
    mock_impl_memory_Verify();
    mock_stm32f4xx_hal_Destroy();
    mock_impl_memory_Destroy();
}

/* Each selected case runs in a fresh process, isolating append-only production
 * registries and static context pools without adding test hooks to production. */
#define STM32F4_TEST_MAIN_BEGIN()                                                                  \
    int main(int argc, char** argv)                                                                \
    {                                                                                              \
        bool matched = false;                                                                      \
        UNITY_BEGIN();

#define STM32F4_RUN_TEST(test_name)                                                                \
    do                                                                                             \
    {                                                                                              \
        if (argc == 1 || strcmp(argv[1], #test_name) == 0)                                         \
        {                                                                                          \
            RUN_TEST(test_name);                                                                   \
            matched = true;                                                                        \
        }                                                                                          \
    } while (0)

#define STM32F4_TEST_MAIN_END()                                                                    \
    int unity_result = UNITY_END();                                                                \
    return matched ? unity_result : 2;                                                             \
    }

#endif /* STM32F4_TEST_SUPPORT_H */
