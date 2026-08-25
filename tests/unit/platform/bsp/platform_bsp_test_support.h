/**
 * @file platform_bsp_test_support.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef PLATFORM_BSP_TEST_SUPPORT_H
#define PLATFORM_BSP_TEST_SUPPORT_H

#include "mock_plat_memory.h"
#include "mock_platform_bsp_backend.h"
#include "unity.h"

static inline void PlatformBsp_Test_MockInit(void)
{
    mock_plat_memory_Init();
    mock_platform_bsp_backend_Init();
}

static inline void PlatformBsp_Test_MockVerify(void)
{
    mock_plat_memory_Verify();
    mock_platform_bsp_backend_Verify();
    mock_plat_memory_Destroy();
    mock_platform_bsp_backend_Destroy();
}

#endif /* PLATFORM_BSP_TEST_SUPPORT_H */
