/**
 * @file dev_buzzer.h
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 */

#ifndef DEV_BUZZER_H
#define DEV_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#include "plat_pwm.h"
#include "util_seq.h"

/** @brief Frequency value marking a rest (silence) in a tone sequence. */
#define DEV_BUZZER_REST 0u

/** @brief Sentinel ending a tone sequence, in place of an explicit length. */
#define DEV_BUZZER_END                                                                             \
    {                                                                                              \
        0u, 0u                                                                                     \
    }

/**
 * @brief One step of a tone sequence: a pitch held for a duration.
 *
 * A @c freq_hz of DEV_BUZZER_REST sounds nothing for @p ms, which is how gaps
 * between repeated notes of the same pitch are expressed — without a rest, two
 * consecutive equal pitches are indistinguishable from one long note.
 *
 * Durations are in milliseconds and converted to ticks on playback, so a melody
 * definition does not depend on the caller's tick rate.
 */
typedef struct
{
    uint16_t freq_hz; /**< Pitch in Hz, or DEV_BUZZER_REST for silence.      */
    uint16_t ms;      /**< Duration in milliseconds. Zero ends the sequence. */
} DEV_Buzzer_Tone_s;

typedef struct DEV_Buzzer_s DEV_Buzzer_s;

/**
 * @brief Create a buzzer device on a PWM output.
 *
 * The PWM instance must already be wired to the physical buzzer channel by the
 * board setup; this layer only drives frequency and duty. The output is left
 * silent until something is played.
 *
 * @param pwm       PWM instance (must not be NULL).
 * @param tick_hz   Rate at which DEV_Buzzer_Tick will be called, in Hz. Used to
 *                  convert note durations from milliseconds; must be non-zero.
 *                  The internal clock accumulates ticks and divides once (floor),
 *                  so it is exact for any rate that divides 1000 evenly (100, 200,
 *                  250, 500, 1000, ...); other rates converge to the true elapsed
 *                  time as ticks accumulate but can read up to ~1 ms low on any
 *                  single tick, which only matters for a note a handful of ticks
 *                  long.
 * @param volume    Duty cycle in percent while sounding, clamped to [0,100]. A
 *                  passive buzzer is loudest near 50; lower values attenuate it.
 * @return Pointer to the created device, or NULL on invalid arguments or
 *         allocation failure.
 */
DEV_Buzzer_s* DEV_Buzzer_Create(PWM_Instance_s* pwm, uint32_t tick_hz, float volume);

/**
 * @brief Advance playback by one tick. Call at the rate passed to Create.
 *
 * This is what makes playback non-blocking: a melody advances one step per tick
 * from a periodic task instead of busy-waiting through its total duration. Not
 * calling it leaves the current note sounding indefinitely.
 *
 * @param buz  Buzzer device.
 */
void DEV_Buzzer_Tick(DEV_Buzzer_s* buz);

/**
 * @brief Sound a single tone for a fixed duration.
 *
 * Replaces whatever is currently playing. The tone stops on its own after
 * @p ms, so no paired stop call is needed.
 *
 * @param buz      Buzzer device.
 * @param freq_hz  Pitch in Hz; DEV_BUZZER_REST is silence.
 * @param ms       Duration in milliseconds. Zero is a no-op.
 */
void DEV_Buzzer_Beep(DEV_Buzzer_s* buz, uint16_t freq_hz, uint16_t ms);

/**
 * @brief Play a tone sequence.
 *
 * The sequence is referenced, not copied, so @p tones must stay valid until
 * playback finishes — a @c static const array is the intended form. It must end
 * with a zero-duration entry (DEV_BUZZER_END).
 *
 * Replaces whatever is currently playing.
 *
 * @param buz    Buzzer device.
 * @param tones  Sequence to play (must not be NULL).
 * @param loop   When true, restart from the beginning after the last note.
 */
void DEV_Buzzer_Play(DEV_Buzzer_s* buz, const DEV_Buzzer_Tone_s* tones, bool loop);

/**
 * @brief Play a UTIL_Seq frame array.
 *
 * The general form: frames carry a pitch in channel 0 and support ramping, so a
 * slide between two pitches is one frame rather than a hand-written run of
 * short notes.
 *
 * @par Why this exists alongside DEV_Buzzer_Play
 * DEV_Buzzer_Tone_s and UTIL_Seq_Frame_s do not share a layout, and converting
 * between them needs a destination buffer this device does not own. So the two
 * entry points stay separate: Play is the convenient form for a plain melody,
 * this is the one that can ramp and that shares its timeline model with the
 * status indicator.
 *
 * Replaces whatever was playing. The frame array must outlive playback — the
 * player holds the pointer rather than copying it.
 *
 * @param buz     Buzzer device.
 * @param frames  Frames, channel 0 a pitch in Hz, terminated by @c ms == 0.
 *                A pitch of DEV_BUZZER_REST is silence.
 * @param loop    True to repeat.
 */
void DEV_Buzzer_PlaySeq(DEV_Buzzer_s* buz, const UTIL_Seq_Frame_s* frames, bool loop);

/**
 * @brief Silence the buzzer and abandon any sequence in progress.
 * @param buz  Buzzer device.
 */
void DEV_Buzzer_Stop(DEV_Buzzer_s* buz);

/**
 * @brief Query whether a tone or sequence is currently sounding.
 *
 * Useful to avoid interrupting an alert that is still playing; a looping
 * sequence never reports idle.
 *
 * @param buz  Buzzer device.
 * @return true while playback is active.
 */
bool DEV_Buzzer_IsPlaying(const DEV_Buzzer_s* buz);

/**
 * @brief Set the duty cycle used while sounding.
 * @param buz     Buzzer device.
 * @param volume  Duty in percent, clamped to [0,100]. Takes effect on the next
 *                note, or immediately if one is sounding.
 */
void DEV_Buzzer_SetVolume(DEV_Buzzer_s* buz, float volume);

/* ------------------------------------------------------------------------- */
/*  Note frequencies                                                         */
/* ------------------------------------------------------------------------- */
/*  Equal-temperament pitches, rounded to whole Hz — the timer's achievable    */
/*  frequencies are quantised far more coarsely than this rounding anyway.     */

#define DEV_NOTE_C4 262u
#define DEV_NOTE_D4 294u
#define DEV_NOTE_E4 330u
#define DEV_NOTE_F4 349u
#define DEV_NOTE_G4 392u
#define DEV_NOTE_A4 440u
#define DEV_NOTE_B4 494u

#define DEV_NOTE_C5 523u
#define DEV_NOTE_D5 587u
#define DEV_NOTE_E5 659u
#define DEV_NOTE_F5 698u
#define DEV_NOTE_G5 784u
#define DEV_NOTE_A5 880u
#define DEV_NOTE_B5 988u

#define DEV_NOTE_C6 1047u
#define DEV_NOTE_D6 1175u
#define DEV_NOTE_E6 1319u

#endif /* DEV_BUZZER_H */
