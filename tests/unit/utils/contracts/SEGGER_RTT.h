#ifndef UTILS_CMOCK_SEGGER_RTT_H
#define UTILS_CMOCK_SEGGER_RTT_H

#include <stdarg.h>
#include <stddef.h>

int      SEGGER_RTT_printf(unsigned buffer_index, const char* format, ...);
int      SEGGER_RTT_vprintf(unsigned buffer_index, const char* format, va_list* args);
unsigned SEGGER_RTT_WriteString(unsigned buffer_index, const char* text);

#endif /* UTILS_CMOCK_SEGGER_RTT_H */
