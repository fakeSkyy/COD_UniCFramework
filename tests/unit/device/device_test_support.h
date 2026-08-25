/**
 * @file device_test_support.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef DEVICE_TEST_SUPPORT_H
#define DEVICE_TEST_SUPPORT_H

#include "mock_device_platform.h"

static inline void DEVICE_CMock_Init(void) { mock_device_platform_Init(); }

static inline void DEVICE_CMock_Verify(void) { mock_device_platform_Verify(); }

static inline void DEVICE_CMock_Destroy(void) { mock_device_platform_Destroy(); }

#endif /* DEVICE_TEST_SUPPORT_H */
