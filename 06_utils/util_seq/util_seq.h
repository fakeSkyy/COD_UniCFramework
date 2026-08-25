/**
 * @file util_seq.h
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#ifndef UTIL_SEQ_H
#define UTIL_SEQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ==========================================================================
 * One timeline, several kinds of output
 * ==========================================================================
 *
 * Turns a list of timed values into "what should be on the output right now".
 * A buzzer plays a note per frame, a WS2812 a colour per frame; neither the
 * timeline nor the interpolation cares which, so both come from here.
 *
 * @par Why the caller drives it rather than the player blocking
 * A player that slept through each frame would own its task for the whole
 * sequence, so one task could drive exactly one device. Stepping from the
 * caller's own periodic tick means a single task can advance any number of
 * sequences, and the task stays free to do other work between steps. The status
 * indicator's pattern loop was written the blocking way and is what this
 * replaces.
 *
 * @par Why an absolute timeline rather than a countdown
 * Step compares the caller's clock against when the current frame started,
 * instead of decrementing a counter once per call. A late call therefore eats
 * into that one frame rather than stretching the whole sequence, and the total
 * duration does not change with how often Step is called. Advancing the frame
 * start by the frame's own length — not to "now" — is what keeps a late step
 * from pushing every later frame back.
 *
 * @par Concurrency
 * One instance must be stepped from one context only; there is no locking. Two
 * devices means two instances, which is also how they are played together: the
 * caller starts both, and each is advanced by whichever task owns it.
 * ==========================================================================
 */

/**
 * @brief Channels carried by one frame.
 *
 * Three for an RGB pixel, one for a pitch, and one spare. A frame is 12 bytes at
 * this width, so a 19-frame pattern costs 228 — small enough that raising it is
 * cheap, but every sequence in the image pays for it.
 */
#define UTIL_SEQ_CHANNELS 4u

/**
 * @brief One step of a sequence: a set of values held for a duration.
 *
 * @par Why channels are 16-bit when a colour needs 8
 * A buzzer pitch does not fit in a byte — 262 Hz for middle C, several kHz for
 * an alarm — and splitting it across two byte channels breaks interpolation:
 * the two halves would ramp independently, so 262 to 523 Hz would pass through
 * 264 Hz where the correct value is near 390. Sliding a pitch would come out as
 * a staircase. Paying a byte per channel on colours is the cheaper mistake.
 */
typedef struct
{
    uint16_t ch[UTIL_SEQ_CHANNELS]; /**< Interpreted by the device.            */
    uint16_t ms;                    /**< Duration; 0 ends the sequence.        */
    bool     ramp;                  /**< Interpolate towards the next frame.   */
} UTIL_Seq_Frame_s;

/**
 * @brief Playback state for one sequence. Treat every field as private.
 *
 * Caller-owned storage, no allocation — the same arrangement as every other
 * module in this layer.
 */
typedef struct
{
    const UTIL_Seq_Frame_s* frames;                 /**< Sequence, or NULL when idle.  */
    uint32_t                frame_start_ms;         /**< When the current frame began. */
    uint32_t                total_ms;               /**< Sum of every frame's ms.       */
    uint16_t                index;                  /**< Current frame.                */
    uint16_t                out[UTIL_SEQ_CHANNELS]; /**< Interpolated output.  */
    bool                    loop;
    bool                    playing;
} UTIL_Seq_s;

/* ========================================================================= */
/*  Lifecycle                                                               */
/* ========================================================================= */

/**
 * @brief Put an instance in the idle state.
 *
 * Call once before the first Play. Leaves every output channel at zero, which a
 * device reads as "off" for both a colour and a pitch.
 *
 * @param s  Instance to initialise.
 */
void UTIL_Seq_Init(UTIL_Seq_s* s);

/**
 * @brief Start a sequence, replacing whatever was playing.
 *
 * The frame array must outlive playback — the player holds the pointer rather
 * than copying, so a sequence built on the caller's stack is a dangling read.
 * A static or file-scope array is the intended use.
 *
 * The first frame's values are loaded immediately, so Out is valid as soon as
 * this returns rather than only after the first Step.
 *
 * @param s       Instance.
 * @param frames  Frame array, terminated by a frame whose @c ms is 0.
 * @param loop    True to restart at frame 0 when the sequence ends.
 * @param now_ms  Caller's clock, the same source Step will be given.
 * @return true when playback started; false when @p s or @p frames is NULL, or
 *         the sequence is empty (its first frame already terminates it). On
 *         false the instance is left idle with zeroed output, so an unchecked
 *         caller gets silence rather than a stuck frame.
 */
bool UTIL_Seq_Play(UTIL_Seq_s* s, const UTIL_Seq_Frame_s* frames, bool loop, uint32_t now_ms);

/**
 * @brief Stop playback and zero the output.
 *
 * The output is zeroed rather than frozen, because a caller that writes Out
 * unconditionally would otherwise hold the last colour or pitch indefinitely —
 * a stop that leaves the buzzer sounding is the bug this prevents.
 *
 * @param s  Instance. NULL is ignored.
 */
void UTIL_Seq_Stop(UTIL_Seq_s* s);

/* ========================================================================= */
/*  Playback                                                                */
/* ========================================================================= */

/**
 * @brief Advance the timeline and recompute the output.
 *
 * Call periodically. The period only bounds how finely a ramp is resolved and
 * how promptly a frame boundary is noticed — it does not affect the sequence's
 * total duration, which comes from the frames.
 *
 * A single call may cross several frames when the gap since the last one exceeds
 * a frame's duration, so a sequence stays on schedule after a long preemption
 * rather than replaying frame by frame.
 *
 * @param s       Instance.
 * @param now_ms  Caller's clock, monotonic milliseconds. Wrapping is handled.
 * @return true while a sequence is playing. On false the output holds the last
 *         frame's values, which is what lets a pattern end lit rather than
 *         snapping dark on its final millisecond.
 */
bool UTIL_Seq_Step(UTIL_Seq_s* s, uint32_t now_ms);

/**
 * @brief The current output values.
 *
 * @param s  Instance.
 * @return Pointer to UTIL_SEQ_CHANNELS values, valid until the next Step, Play
 *         or Stop. NULL only when @p s is NULL — an idle player returns zeros,
 *         so a caller need not special-case "not playing".
 */
const uint16_t* UTIL_Seq_Out(const UTIL_Seq_s* s);

/**
 * @brief Whether a sequence is still running.
 * @param s  Instance.
 * @return true while playing; false when idle, stopped, or finished.
 */
bool UTIL_Seq_IsPlaying(const UTIL_Seq_s* s);

#endif /* UTIL_SEQ_H */
