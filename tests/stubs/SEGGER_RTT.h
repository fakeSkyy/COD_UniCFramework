/**
 * @file SEGGER_RTT.h
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#include <stdarg.h>
#include <stddef.h>

/* Host stub for the SEGGER RTT API, so util_log and util_assert can be tested
 * off-target. Only the three entry points 06_utils actually calls are here.
 *
 * The implementation in SEGGER_RTT_stub.c records what was written instead of
 * printing it, which is what lets a test assert that a log line was emitted at
 * all — the real question for util_log, whose whole job is deciding whether to
 * emit. */

int SEGGER_RTT_printf(unsigned BufferIndex, const char* sFormat, ...);

/* SEGGER's signature takes a va_list POINTER, not a va_list — util_log passes one
 * through because its own arguments arrive as a va_list. Getting this wrong
 * compiles and then reads the wrong stack. */
int      SEGGER_RTT_vprintf(unsigned BufferIndex, const char* sFormat, va_list* pParamList);
unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s);

/* Test-only inspection of what the stub captured. */
void        RTT_StubReset(void);
unsigned    RTT_StubWriteCount(void);
const char* RTT_StubLastText(void);

const char* RTT_StubText(unsigned call_index);
#endif /* SEGGER_RTT_H */
