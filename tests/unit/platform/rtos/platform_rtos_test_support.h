/**
 * @file platform_rtos_test_support.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef PLATFORM_RTOS_TEST_SUPPORT_H
#define PLATFORM_RTOS_TEST_SUPPORT_H

#include "mock_prtos_impl_memory.h"
#include "mock_prtos_impl_mutex.h"
#include "mock_prtos_impl_sem.h"
#include "mock_prtos_impl_task.h"
#include "mock_prtos_platform_rtos_backend.h"
#include "unity.h"

static inline void PlatformRtos_Test_MockInit(void)
{
    mock_prtos_impl_memory_Init();
    mock_prtos_impl_mutex_Init();
    mock_prtos_impl_sem_Init();
    mock_prtos_impl_task_Init();
    mock_prtos_platform_rtos_backend_Init();
}

static inline void PlatformRtos_Test_MockVerify(void)
{
    mock_prtos_impl_memory_Verify();
    mock_prtos_impl_mutex_Verify();
    mock_prtos_impl_sem_Verify();
    mock_prtos_impl_task_Verify();
    mock_prtos_platform_rtos_backend_Verify();
    mock_prtos_impl_memory_Destroy();
    mock_prtos_impl_mutex_Destroy();
    mock_prtos_impl_sem_Destroy();
    mock_prtos_impl_task_Destroy();
    mock_prtos_platform_rtos_backend_Destroy();
}

#endif /* PLATFORM_RTOS_TEST_SUPPORT_H */
