/**
 * @file SEGGER_RTT_stub.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "SEGGER_RTT.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RTT_STUB_MAX_WRITES 32u
#define RTT_STUB_TEXT_BYTES 512u

static char     s_text[RTT_STUB_MAX_WRITES][RTT_STUB_TEXT_BYTES];
static unsigned s_writes;

static char* next_text(void)
{
    unsigned index = s_writes;
    if (index >= RTT_STUB_MAX_WRITES)
    {
        index = RTT_STUB_MAX_WRITES - 1u;
    }

    s_writes++;
    s_text[index][0] = '\0';
    return s_text[index];
}

void RTT_StubReset(void)
{
    memset(s_text, 0, sizeof s_text);
    s_writes = 0u;
}

unsigned RTT_StubWriteCount(void) { return s_writes; }

const char* RTT_StubText(unsigned call_index)
{
    return (call_index < s_writes && call_index < RTT_STUB_MAX_WRITES) ? s_text[call_index] : "";
}

const char* RTT_StubLastText(void)
{
    if (s_writes == 0u)
    {
        return "";
    }

    unsigned index = s_writes - 1u;
    if (index >= RTT_STUB_MAX_WRITES)
    {
        index = RTT_STUB_MAX_WRITES - 1u;
    }
    return s_text[index];
}

int SEGGER_RTT_printf(unsigned BufferIndex, const char* sFormat, ...)
{
    (void) BufferIndex;

    char*   text = next_text();
    va_list ap;
    va_start(ap, sFormat);
    const int n = vsnprintf(text, RTT_STUB_TEXT_BYTES, sFormat, ap);
    va_end(ap);
    return n;
}

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char* sFormat, va_list* pParamList)
{
    (void) BufferIndex;
    return vsnprintf(next_text(), RTT_STUB_TEXT_BYTES, sFormat, *pParamList);
}

unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes)
{
    (void) BufferIndex;

    char*          text = next_text();
    const unsigned n =
        (NumBytes < RTT_STUB_TEXT_BYTES - 1u) ? NumBytes : (unsigned) (RTT_STUB_TEXT_BYTES - 1u);
    memcpy(text, pBuffer, n);
    text[n] = '\0';
    return NumBytes;
}

unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s)
{
    return SEGGER_RTT_Write(BufferIndex, s, (unsigned) strlen(s));
}
