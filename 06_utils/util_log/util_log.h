/**
 * @file util_log.h
 * @author Gao Xing
 * @date 2026/8/11
 * @version 1.0
 */

#ifndef UTIL_LOG_H
#define UTIL_LOG_H

#include <stdbool.h>

/* ==========================================================================
 * Logging, at three levels, with the disabled levels compiled out
 * ==========================================================================
 *
 * Every log site in the framework used to be a bare SEGGER_RTT_printf with a
 * hand-written prefix. That works, but it settles three things at each call site
 * that ought to be settled once: which transport carries the message, whether
 * this message is worth its cycles, and what a line looks like.
 *
 * @par Why the threshold is a preprocessor constant
 * So that a disabled site leaves the image completely — no call, no format
 * string, and no evaluation of the arguments. A UTIL_LOG_INFO whose argument is
 * an expensive query costs nothing in a build that compiled INFO out. A runtime
 * check could not give that: the string would still occupy flash and the
 * arguments would still be computed before the call decided to discard them.
 *
 * On this target each format string is tens of bytes of flash, and there are
 * enough log sites for that to matter.
 *
 * @par The two levels, and why both exist
 * UTIL_LOG_LEVEL is the ceiling, fixed at build time. UTIL_Log_SetLevel is a
 * runtime threshold underneath it, for quieting a phase without rebuilding.
 * Nothing at runtime can raise output above the ceiling — those sites no longer
 * exist.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Levels                                                                   */
/* ========================================================================= */

/* Plain integers, not an enum, because #if arithmetic is what selects the
 * compiled-in set and the preprocessor cannot see an enum — every enumerator
 * would evaluate to 0 there and the filter would silently keep everything. This
 * is the whole reason the levels are spelled this way; do not "tidy" them into an
 * enum. UTIL_Log_Level_e below exists for the function signatures, and its
 * members are defined from these so the two cannot drift.
 *
 * Ascending by verbosity, so UTIL_LOG_LEVEL reads as "the least severe level
 * still compiled in" and a site is kept when its own level is <= that.
 *
 * NONE is -1 rather than a value above INFO: it has to compare as *more* severe
 * than ERROR for `<=` to exclude everything, and a value above INFO would instead
 * admit everything — which is exactly the inversion this comment exists to stop
 * someone reintroducing. It is deliberately not a member of UTIL_Log_Level_e,
 * because it is a threshold rather than a severity anything can be logged at. */

#define UTIL_LOG_LEVEL_NONE (-1) /**< Threshold only: silences every level.      */
#define UTIL_LOG_LEVEL_ERROR 0   /**< Something failed; the caller could not go on. */
#define UTIL_LOG_LEVEL_WARN 1    /**< Something was wrong but was handled.       */
#define UTIL_LOG_LEVEL_INFO 2    /**< Normal progress worth seeing at bring-up.  */

/**
 * @brief Least severe level compiled in. Anything more verbose is removed entirely.
 *
 * Set it in the build to strip sites from a release image without editing a
 * single call:
 *
 *     -DUTIL_LOG_LEVEL=UTIL_LOG_LEVEL_WARN     # drop every INFO site
 *     -DUTIL_LOG_LEVEL=UTIL_LOG_LEVEL_ERROR    # keep only failures
 *     -DUTIL_LOG_LEVEL=UTIL_LOG_LEVEL_NONE     # drop all logging
 *
 * Defaults to INFO: during bring-up the cost of a message is far below the cost
 * of not having it.
 */
#ifndef UTIL_LOG_LEVEL
#define UTIL_LOG_LEVEL UTIL_LOG_LEVEL_INFO
#endif

/**
 * @brief Severity, for the runtime entry points.
 *
 * Members come from the macros above so the two spellings cannot disagree. There
 * is no NONE member by design — see the note there; pass UTIL_LOG_LEVEL_NONE to
 * UTIL_Log_SetLevel when silencing output.
 */
typedef enum
{
    UTIL_LOG_ERROR = UTIL_LOG_LEVEL_ERROR,
    UTIL_LOG_WARN  = UTIL_LOG_LEVEL_WARN,
    UTIL_LOG_INFO  = UTIL_LOG_LEVEL_INFO,
} UTIL_Log_Level_e;

/* ========================================================================= */
/*  Call sites                                                               */
/* ========================================================================= */

/* Each macro expands to nothing when its level is more verbose than the ceiling —
 * to `((void) 0)` rather than to an empty function call or a discarded
 * expression, so the arguments are not evaluated.
 *
 * The do/while(0) wrapper on the enabled form is what lets
 * `if (x) UTIL_LOG_I("t", "..."); else ...` compile; a bare braced block would
 * swallow the else. */

/**
 * @brief Log a failure: an operation did not complete, and the caller knows it.
 *
 * @param tag  Short subsystem name, e.g. "can" or "imu". Keep it lowercase and
 *             stable — it is what makes the output greppable.
 * @param ...  Format string and arguments. No `%f`; see UTIL_Log_Write.
 */
#if (UTIL_LOG_LEVEL_ERROR <= UTIL_LOG_LEVEL)
#define UTIL_LOG_E(tag, ...)                                                                       \
    do                                                                                             \
    {                                                                                              \
        UTIL_Log_Write(UTIL_LOG_ERROR, (tag), __VA_ARGS__);                                        \
    } while (0)
#else
#define UTIL_LOG_E(tag, ...) ((void) 0)
#endif

/**
 * @brief Log a recovered problem: wrong, but compensated for.
 *
 * The level for a clamped argument, a retried transfer, a reading that arrived
 * out of range. If the caller could not proceed it is an error, not a warning.
 *
 * @param tag  Short subsystem name.
 * @param ...  Format string and arguments.
 */
#if (UTIL_LOG_LEVEL_WARN <= UTIL_LOG_LEVEL)
#define UTIL_LOG_W(tag, ...)                                                                       \
    do                                                                                             \
    {                                                                                              \
        UTIL_Log_Write(UTIL_LOG_WARN, (tag), __VA_ARGS__);                                         \
    } while (0)
#else
#define UTIL_LOG_W(tag, ...) ((void) 0)
#endif

/**
 * @brief Log normal progress.
 *
 * @par Not on a control path
 * A log site inside a 1 kHz loop will itself cause the deadline miss it was added
 * to investigate: the transport formats and copies a string with interrupts
 * masked. Count the events and report the count from a low-rate task instead.
 *
 * @param tag  Short subsystem name.
 * @param ...  Format string and arguments.
 */
#if (UTIL_LOG_LEVEL_INFO <= UTIL_LOG_LEVEL)
#define UTIL_LOG_I(tag, ...)                                                                       \
    do                                                                                             \
    {                                                                                              \
        UTIL_Log_Write(UTIL_LOG_INFO, (tag), __VA_ARGS__);                                         \
    } while (0)
#else
#define UTIL_LOG_I(tag, ...) ((void) 0)
#endif

/* ========================================================================= */
/*  Runtime entry points                                                     */
/* ========================================================================= */

/**
 * @brief Emit one line. Called by the macros above; rarely called directly.
 *
 * Prefixes the level and the tag, appends CRLF, and writes to the transport. One
 * call is one line, so concurrent callers interleave whole lines rather than
 * fragments.
 *
 * @par What the format string supports — and what it does not
 * The RTT back end has its own formatter, not the C library's: `%d`, `%u`, `%x`,
 * `%X`, `%c`, `%s`, `%p`, `%%`, with width, zero-pad, `-`, `+` and `#`.
 *
 * **There is no `%f`.** This framework is float-only, so that is the omission
 * worth remembering. A `%f` does not merely print badly: the formatter consumes
 * the wrong number of argument bytes, so every conversion after it in the same
 * call is garbage too. Scale to an integer and name the scale:
 *
 *     UTIL_LOG_I("pid", "kp x1000 = %d", (int) (kp * 1000.0f));
 *
 * The `format(printf, ...)` attribute below means the compiler still checks
 * argument counts and integer types against the string — it does not know about
 * the missing `%f`, so `-Wformat` will happily accept one.
 *
 * @par Context and loss
 * Safe from a task, from an interrupt, and before the scheduler: the transport
 * masks interrupts to a configured priority for the write rather than taking an
 * RTOS lock, so this needs no scheduler. When the transport buffer is full the
 * message is dropped rather than blocking — a log call never stalls its caller,
 * which also makes output lossy under load, deliberately.
 *
 * @param level  Severity. Emitted when at or below the runtime threshold.
 * @param tag    Short subsystem name. NULL prints as "?".
 * @param fmt    Format string, with no trailing newline.
 */
void UTIL_Log_Write(UTIL_Log_Level_e level, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * @brief Lower, or restore, the runtime threshold.
 *
 * Bounded by UTIL_LOG_LEVEL — sites above the ceiling were removed at compile
 * time and no runtime setting brings them back. A value above the ceiling is
 * accepted and simply has no further effect.
 *
 * Starts at UTIL_LOG_LEVEL, so a fresh image emits everything it contains.
 *
 * @param level  New threshold. UTIL_LOG_LEVEL_NONE silences all output.
 */
void UTIL_Log_SetLevel(UTIL_Log_Level_e level);

/**
 * @brief The current runtime threshold.
 * @return Threshold as last set, or UTIL_LOG_LEVEL if never set.
 */
UTIL_Log_Level_e UTIL_Log_GetLevel(void);

/**
 * @brief Whether a message at @p level would be emitted right now.
 *
 * For the case the macros cannot cover: skipping the *computation* behind a log
 * rather than the log itself.
 *
 *     if (UTIL_Log_Enabled(UTIL_LOG_INFO))
 *     {
 *         size_t n = expensive_scan();
 *         UTIL_LOG_I("mon", "%u slots used", (unsigned) n);
 *     }
 *
 * @param level  Severity to test.
 * @return true when a write at that level would produce output.
 */
bool UTIL_Log_Enabled(UTIL_Log_Level_e level);

#endif /* UTIL_LOG_H */
