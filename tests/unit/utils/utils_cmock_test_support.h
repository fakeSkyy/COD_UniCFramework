/**
 * @file utils_cmock_test_support.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef UTILS_CMOCK_TEST_SUPPORT_H
#define UTILS_CMOCK_TEST_SUPPORT_H

#include "mock_SEGGER_RTT.h"
#include "mock_plat_mutex.h"
#include "mock_plat_task.h"
#include "test_support.h"

static inline void Utils_CMock_Init(void)
{
    mock_SEGGER_RTT_Init();
    mock_plat_mutex_Init();
    mock_plat_task_Init();
}

static inline void Utils_CMock_Verify(void)
{
    mock_SEGGER_RTT_Verify();
    mock_plat_mutex_Verify();
    mock_plat_task_Verify();
    mock_SEGGER_RTT_Destroy();
    mock_plat_mutex_Destroy();
    mock_plat_task_Destroy();
}

#endif /* UTILS_CMOCK_TEST_SUPPORT_H */
