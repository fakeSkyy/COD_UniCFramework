/**
 * @file util_log.c
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#include "util_log.h"

#include <stdarg.h>

#include "SEGGER_RTT.h"

/* ==========================================================================
 * Binding the levels to a transport
 * ==========================================================================
 *
 * SEGGER RTT, on up-buffer 0. It is the only log path this firmware has and the
 * only one that works in the situations a log matters most: a fault handler with
 * interrupts masked, or bring-up before any peripheral is configured. RTT is a
 * memory write plus a debugger poll — no peripheral, no clock, no DMA.
 *
 * @par Why this file is in 06_utils and includes a vendor header
 * 06_utils may not depend on layers above it, and it does not: SEGGER RTT is in
 * 05_vender, which is downward. util_assert.h already reaches for it the same
 * way. What would be wrong is depending on 03_platform or 04_impl, because those
 * are the layers that log.
 *
 * @par Why not route through a platform UART instead
 * A log has to work when the thing being debugged is the UART. Going through
 * PLAT_UART would also make 06_utils depend on 03_platform, which the layering
 * forbids — and it would put a log call inside the code path it is reporting on.
 * ==========================================================================
 */

/** @brief RTT up-buffer this module writes to. Buffer 0 always exists. */
#define LOG_CHANNEL 0u

/**
 * @brief Fixed-width level tags.
 *
 * Padded to the same width so the tag and message columns line up in a terminal;
 * scanning a log for the one line that matters is much easier when the text
 * starts at a fixed offset. Indexed by UTIL_Log_Level_e, so the order here is
 * load-bearing.
 */
static const char* const level_tag[] = {
    "E", /* UTIL_LOG_ERROR */
    "W", /* UTIL_LOG_WARN  */
    "I", /* UTIL_LOG_INFO  */
};

/**
 * @brief Runtime threshold, initialised to the compiled-in ceiling.
 *
 * A plain int, not UTIL_Log_Level_e, so that UTIL_LOG_LEVEL_NONE (-1) is
 * representable: the enum has no member for it, because it is a threshold rather
 * than a severity anything is logged at. Storing an out-of-range value in an enum
 * object is where a compiler becomes free to fold the comparison against it.
 *
 * Not volatile and not atomic. It is a single word, so a read cannot observe a
 * half-written value on this architecture, and the only cost of racing with
 * SetLevel is that one message is judged against the old threshold — which is
 * indistinguishable from that message having been logged a moment earlier.
 * Paying for synchronisation to order log lines against a threshold change would
 * buy nothing.
 */
static int log_level = UTIL_LOG_LEVEL;

void UTIL_Log_SetLevel(UTIL_Log_Level_e level) { log_level = (int) level; }

UTIL_Log_Level_e UTIL_Log_GetLevel(void) { return (UTIL_Log_Level_e) log_level; }

bool UTIL_Log_Enabled(UTIL_Log_Level_e level) { return (int) level <= log_level; }

void UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...)
{
    if ((int) level > log_level || fmt == NULL)
    {
        return;
    }

    /* Bounds-checked rather than trusted: the macros only ever pass the three
     * real levels, but this is a public function and an out-of-range level would
     * otherwise index past level_tag. */
    const unsigned idx = (unsigned) level;
    const char*    lvl = (idx < sizeof level_tag / sizeof level_tag[0]) ? level_tag[idx] : "?";

    /* Prefix and body are two separate writes, which is a deliberate trade.
     *
     * One write would need this function to own a buffer big enough for the
     * longest line, and to build the combined format string at runtime — a
     * fixed-size static buffer shared between contexts (so an interrupt logging
     * mid-task would corrupt it), or several hundred bytes of stack in a
     * function reachable from a fault handler whose stack may be nearly gone.
     *
     * Two writes cost one extra RTT lock acquisition. The interleaving risk that
     * buys is real but bounded: the prefix and body of one line can be separated
     * only by an interrupt that logs, and both halves still arrive, in order,
     * with nothing lost. A corrupted shared buffer loses the whole line and the
     * other one too. */
    SEGGER_RTT_printf(LOG_CHANNEL, "[%s][%s] ", lvl, (tag != NULL) ? tag : "?");

    va_list ap;
    va_start(ap, fmt);

    /* vprintf, not a second printf: the arguments arrived as a va_list and there
     * is no portable way to forward them to a variadic function.
     *
     * None of the three RTT calls here is checked, and deliberately: the buffer is
     * configured NO_BLOCK_SKIP, so a full buffer discards the line rather than
     * failing, and a logger that reported its own failures would need somewhere to
     * report them to. */
    SEGGER_RTT_vprintf(LOG_CHANNEL, fmt, &ap);

    va_end(ap);

    /* CRLF, not LF: RTT Viewer and most serial terminals treat a bare LF as a
     * line feed without a carriage return and stair-step the output. */
    SEGGER_RTT_WriteString(LOG_CHANNEL, "\r\n");
}
