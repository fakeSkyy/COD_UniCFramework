/**
 * @file util_seq.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include "util_seq.h"

/* ========================================================================= */
/*  Frame access                                                             */
/* ========================================================================= */

/**
 * @brief Whether the frame at @p index terminates the sequence.
 *
 * @param s      Instance, known non-NULL and playing.
 * @param index  Frame to test.
 * @return true when the frame's duration is zero.
 */
static bool is_end(const UTIL_Seq_s* s, uint16_t index) { return s->frames[index].ms == 0u; }

/**
 * @brief Copy one frame's channel values to the output.
 *
 * @param s      Instance.
 * @param index  Frame to load.
 */
static void load(UTIL_Seq_s* s, uint16_t index)
{
    for (unsigned c = 0u; c < UTIL_SEQ_CHANNELS; c++)
    {
        s->out[c] = s->frames[index].ch[c];
    }
}

/** @brief Zero every output channel. Reads as off for both a colour and a pitch. */
static void silence(UTIL_Seq_s* s)
{
    for (unsigned c = 0u; c < UTIL_SEQ_CHANNELS; c++)
    {
        s->out[c] = 0u;
    }
}

/**
 * @brief Index of the frame a ramp on @p index moves towards, or @p index itself.
 *
 * A ramp needs a next frame to reach. The frame after the last one is the
 * terminator, whose channels are all zero — reading it would fade every
 * one-shot pattern to black regardless of what it meant. So a one-shot's final
 * frame targets itself, which makes the interpolation a no-op and holds the
 * value. A looping sequence targets frame 0 instead, and that wrap-around is
 * exactly what makes a breathing light continuous across cycles.
 *
 * @param s      Instance, known non-NULL and playing.
 * @param index  Frame the ramp starts from.
 * @return Frame index to interpolate towards.
 */
static uint16_t ramp_target(const UTIL_Seq_s* s, uint16_t index)
{
    if (!is_end(s, (uint16_t) (index + 1u)))
    {
        return (uint16_t) (index + 1u);
    }

    return s->loop ? 0u : index;
}

/**
 * @brief Write the output for a ramping frame, @p elapsed into it.
 *
 * @param s        Instance.
 * @param index    Frame being played.
 * @param elapsed  Milliseconds into that frame, already below its duration.
 */
static void interpolate(UTIL_Seq_s* s, uint16_t index, uint32_t elapsed)
{
    const uint16_t target = ramp_target(s, index);
    const uint16_t span   = s->frames[index].ms;

    for (unsigned c = 0u; c < UTIL_SEQ_CHANNELS; c++)
    {
        const int32_t from = (int32_t) s->frames[index].ch[c];
        const int32_t to   = (int32_t) s->frames[target].ch[c];

        /* Both operands widened to signed BEFORE subtracting. Taking the
         * difference in uint16_t would turn a descending ramp into a huge
         * positive step — 1000 to 0 would read as 64536 — so the output would
         * leap instead of falling. This is the one line here that is easy to get
         * wrong and silent when wrong. */
        const int32_t delta = to - from;

        /* The product is computed in int64_t, not int32_t. delta spans
         * [-65535, 65535] and elapsed is bounded only by span (a uint16_t), so
         * the worst case |delta * elapsed| is 65535 * 65534 =~ 4.29e9 - almost
         * double INT32_MAX. A 32-bit product there is signed overflow, which is
         * undefined behaviour, not just a wrong answer: it was observed to
         * produce off-by-one results near the failing range rather than
         * anything that looked obviously broken. int64_t costs one wider
         * multiply-divide, on a path called a few times per channel per frame
         * from a 25 ms tick - not worth trading correctness for. */
        s->out[c] =
            (uint16_t) (from + (int32_t) (((int64_t) delta * (int64_t) elapsed) / (int64_t) span));
    }
}

/**
 * @brief Sum every frame's duration up to the terminator.
 *
 * Walked once at Play rather than every Step, so a long stall on a looping
 * sequence can later be reduced modulo this value instead of being walked one
 * frame per iteration.
 *
 * @param frames  Sequence, already known non-empty.
 * @return Sum of ms across all frames before the terminator.
 */
static uint32_t total_duration(const UTIL_Seq_Frame_s* frames)
{
    uint32_t total = 0u;
    uint16_t i     = 0u;

    while (frames[i].ms != 0u)
    {
        total += frames[i].ms;
        i++;
    }

    return total;
}

/* ========================================================================= */
/*  Lifecycle                                                               */
/* ========================================================================= */

void UTIL_Seq_Init(UTIL_Seq_s* s)
{
    if (s == NULL)
    {
        return;
    }

    s->frames         = NULL;
    s->frame_start_ms = 0u;
    s->total_ms       = 0u;
    s->index          = 0u;
    s->loop           = false;
    s->playing        = false;

    silence(s);
}

bool UTIL_Seq_Play(UTIL_Seq_s* s, const UTIL_Seq_Frame_s* frames, bool loop, uint32_t now_ms)
{
    if (s == NULL)
    {
        return false;
    }

    /* Reset before validating the sequence, so a rejected Play leaves an idle
     * player rather than one still holding whatever was running. */
    UTIL_Seq_Init(s);

    if (frames == NULL || frames[0].ms == 0u)
    {
        return false;
    }

    s->frames         = frames;
    s->frame_start_ms = now_ms;
    s->total_ms       = total_duration(frames);
    s->loop           = loop;
    s->playing        = true;

    /* Loaded here rather than waiting for the first Step, so the output is valid
     * as soon as this returns. */
    load(s, 0u);

    return true;
}

void UTIL_Seq_Stop(UTIL_Seq_s* s) { UTIL_Seq_Init(s); }

/* ========================================================================= */
/*  Playback                                                                */
/* ========================================================================= */

bool UTIL_Seq_Step(UTIL_Seq_s* s, uint32_t now_ms)
{
    if (s == NULL || !s->playing)
    {
        return false;
    }

    /* Unsigned subtraction, so a wrap of the caller's millisecond counter reads
     * as a small elapsed time rather than a huge one. A naive (now < start) test
     * would stall or skip a sequence at that boundary, once every 49.7 days. */
    uint32_t elapsed = (uint32_t) (now_ms - s->frame_start_ms);

    /* A non-looping sequence is already bounded by its frame count — the while
     * loop below cannot run more times than there are frames. A looping one is
     * not: an elapsed gap of up to 2^32-1 ms against a loop that is milliseconds
     * long would otherwise walk the loop up to 2^32-1 times. Reducing by whole
     * cycles first bounds every path to at most one pass over the frames.
     * total_ms is never 0 for a sequence Play accepted (it rejects frames[0].ms
     * == 0), but the guard keeps the division obviously safe rather than
     * "safe because a caller checked something upstream". */
    if (s->loop && s->total_ms != 0u)
    {
        uint32_t skip = (elapsed / s->total_ms) * s->total_ms;

        /* Advancing frame_start_ms by the same span keeps (now - frame_start_ms)
         * equal to the reduced elapsed, so the current frame and offset within
         * it are unchanged — only the number of cycles left to walk drops. */
        s->frame_start_ms += skip;
        elapsed -= skip;
    }

    /* A while, not an if: one call may span several frames when the gap since the
     * last one exceeds a frame's duration, and advancing one frame per call would
     * leave the sequence permanently behind after a long preemption. */
    while (elapsed >= s->frames[s->index].ms)
    {
        /* The frame's own length, not "now" — advancing to now would give the
         * next frame a full duration starting late, stretching the sequence by
         * the overshoot every time. */
        s->frame_start_ms += s->frames[s->index].ms;
        elapsed -= s->frames[s->index].ms;

        s->index++;

        if (!is_end(s, s->index))
        {
            continue;
        }

        if (!s->loop)
        {
            /* Loaded explicitly: the frame just ended may never have reached the
             * unconditional load below if its own expiry was detected in this
             * same pass, and the output must hold it rather than the frame
             * before, so a pattern ends lit instead of snapping dark or stalling
             * on its second-to-last frame. */
            s->playing = false;
            load(s, (uint16_t) (s->index - 1u));
            return false;
        }

        s->index = 0u;
    }

    if (s->frames[s->index].ramp)
    {
        interpolate(s, s->index, elapsed);
    }
    else
    {
        load(s, s->index);
    }

    return true;
}

const uint16_t* UTIL_Seq_Out(const UTIL_Seq_s* s) { return (s != NULL) ? s->out : NULL; }

bool UTIL_Seq_IsPlaying(const UTIL_Seq_s* s) { return (s != NULL) && s->playing; }
