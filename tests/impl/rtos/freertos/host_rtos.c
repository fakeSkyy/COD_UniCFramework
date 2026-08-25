/**
 * @file host_rtos.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static SCB_Type host_scb;
SCB_Type*       SCB = &host_scb;

jmp_buf rtos_test_fatal_jump;

static uint32_t host_msp;
static uint32_t host_psp;
static unsigned disable_count;
static unsigned dsb_count;
static unsigned isb_count;
static int      fatal_jump_armed;
static char     output[4096];

static void append_format(const char* format, va_list args)
{
    size_t used = strlen(output);

    if (used < sizeof output - 1u)
    {
        (void) vsnprintf(output + used, sizeof output - used, format, args);
    }
}

void RTOS_Test_ResetHost(void)
{
    memset(&host_scb, 0, sizeof host_scb);
    host_msp         = 0u;
    host_psp         = 0u;
    disable_count    = 0u;
    dsb_count        = 0u;
    isb_count        = 0u;
    fatal_jump_armed = 0;
    output[0]        = '\0';
}

void RTOS_Test_ArmFatalJump(void) { fatal_jump_armed = 1; }

void RTOS_Test_SetStackPointers(uint32_t msp, uint32_t psp)
{
    host_msp = msp;
    host_psp = psp;
}

const char* RTOS_Test_GetOutput(void) { return output; }
unsigned    RTOS_Test_GetDisableCount(void) { return disable_count; }
unsigned    RTOS_Test_GetDSBCount(void) { return dsb_count; }
unsigned    RTOS_Test_GetISBCount(void) { return isb_count; }

uint32_t RTOS_Test_GetMSP(void) { return host_msp; }
uint32_t RTOS_Test_GetPSP(void) { return host_psp; }

void RTOS_Test_DisableIRQ(void)
{
    ++disable_count;
    if (fatal_jump_armed != 0)
    {
        fatal_jump_armed = 0;
        longjmp(rtos_test_fatal_jump, 1);
    }
}

void RTOS_Test_DSB(void) { ++dsb_count; }
void RTOS_Test_ISB(void) { ++isb_count; }

void RTOS_Test_Log(const char* tag, const char* format, ...)
{
    (void) snprintf(output, sizeof output, "[%s] ", (tag != NULL) ? tag : "?");

    va_list args;
    va_start(args, format);
    append_format(format, args);
    va_end(args);
}

int SEGGER_RTT_printf(unsigned buffer_index, const char* format, ...)
{
    (void) buffer_index;

    va_list args;
    va_start(args, format);
    append_format(format, args);
    va_end(args);
    return 0;
}

void UTIL_Log_Write(int level, const char* tag, const char* format, ...)
{
    (void) level;
    (void) snprintf(output, sizeof output, "[%s] ", (tag != NULL) ? tag : "?");

    va_list args;
    va_start(args, format);
    append_format(format, args);
    va_end(args);
}
