/**
 * @file rtos_test_support.h
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#ifndef RTOS_TEST_SUPPORT_H
#define RTOS_TEST_SUPPORT_H

#include <setjmp.h>
#include <stddef.h>
#include <stdint.h>

#include "mock_FreeRTOS.h"
#include "mock_semphr.h"
#include "mock_task.h"
#include "stm32h7xx.h"
#include "unity.h"

typedef union
{
    max_align_t align;
    uint8_t     bytes[512];
} RTOS_Test_Storage_u;

extern jmp_buf rtos_test_fatal_jump;

void        RTOS_Test_ResetHost(void);
void        RTOS_Test_ArmFatalJump(void);
void        RTOS_Test_SetStackPointers(uint32_t msp, uint32_t psp);
const char* RTOS_Test_GetOutput(void);
unsigned    RTOS_Test_GetDisableCount(void);
unsigned    RTOS_Test_GetDSBCount(void);
unsigned    RTOS_Test_GetISBCount(void);

static inline void RTOS_Test_MockInit(void)
{
    mock_FreeRTOS_Init();
    mock_task_Init();
    mock_semphr_Init();
    RTOS_Test_ResetHost();
}

static inline void RTOS_Test_MockVerify(void)
{
    mock_FreeRTOS_Verify();
    mock_task_Verify();
    mock_semphr_Verify();
    mock_FreeRTOS_Destroy();
    mock_task_Destroy();
    mock_semphr_Destroy();
}

#endif /* RTOS_TEST_SUPPORT_H */
