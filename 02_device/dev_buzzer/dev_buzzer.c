/**
 * @file dev_buzzer.c
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 *
 * Buzzer device: pitch and duration over a PWM output.
 *
 * Playback is advanced by DEV_Buzzer_Tick from a periodic task rather than by
 * blocking delays. The legacy implementation played its melody with a chain of
 * HAL_Delay calls totalling 4.2 s, which under an RTOS stalls the calling task
 * for that whole time — and with the scheduler running, HAL_Delay's SysTick
 * timebase makes it unreliable as well.
 *
 * A passive buzzer is driven at a fixed duty (nominally 50%) and its pitch comes
 * from the PWM frequency, so this module only ever sets frequency and duty; the
 * timer's PSC/ARR arithmetic lives in the PWM backend.
 */

#include "dev_buzzer.h"

#include "plat_memory.h"

struct DEV_Buzzer_s
{
    PWM_Instance_s* pwm;
    uint32_t        tick_hz;
    float           volume;

    /* Sequence being played, or NULL for a single tone / idle. */
    const DEV_Buzzer_Tone_s* seq;
    uint16_t                 index; /**< Position within seq.               */
    bool                     loop;

    uint32_t remaining;  /**< Ticks left on the current note.               */
    uint16_t current_hz; /**< Pitch currently programmed, 0 when silent.     */
    bool     playing;

    /**
     * @brief The UTIL_Seq path, used by DEV_Buzzer_PlaySeq.
     *
     * Its own playing flag is what Tick dispatches on, so the two paths cannot
     * both drive the output: starting either one stops the other.
     */
    UTIL_Seq_s seq_player;

    /**
     * @brief Ticks elapsed since the current sequence started; see Tick.
     *
     * Reset to 0 by PlaySeq rather than counted for the device's whole lifetime, so
     * this can stay a 32-bit tick count multiplied by 1000 without overflowing: a
     * lifetime counter would wrap that multiplication at 1 kHz after ~49 days of
     * continuous playback, and a sequence is bounded in length, so resetting the base
     * per-sequence is cheaper than widening to 64 bits and never loses range.
     */
    uint32_t seq_ticks;
};

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

static float clamp_volume(float volume)
{
    if (volume < 0.0f)
    {
        return 0.0f;
    }
    if (volume > 100.0f)
    {
        return 100.0f;
    }
    return volume;
}

/**
 * @brief Convert a duration in milliseconds to whole ticks.
 *
 * Rounds up, and never yields zero for a non-zero duration: a note shorter than
 * one tick still has to sound for a tick, otherwise it would be dropped
 * silently and the melody would lose beats.
 */
static uint32_t ms_to_ticks(const DEV_Buzzer_s* buz, uint16_t ms)
{
    if (ms == 0u)
    {
        return 0u;
    }

    uint32_t ticks = ((uint32_t) ms * buz->tick_hz + 999u) / 1000u;
    return (ticks == 0u) ? 1u : ticks;
}

/**
 * @brief Drive the output at @p freq_hz, or silence it when @p freq_hz is a rest.
 *
 * Silence is produced with zero duty rather than by stopping the timer: the
 * counter keeps running, so the next note starts on the frequency it is given
 * instead of on whatever phase a restart would leave behind.
 *
 * @par Skipping a no-op reprogram
 * A no-change call is a no-op: PLAT_PWM_SetFreqAndDuty parks the compare at 0 and
 * forces an update event before writing the new PSC/ARR, which is an audible click
 * on a buzzer even when the frequency it lands on is the one already playing. The
 * sequence path (Tick) calls this every tick regardless of whether the frame's pitch
 * changed, so without this check a held note would click at the tick rate instead of
 * sounding continuously. A genuine change — including silence to a pitch, or a pitch
 * to silence — still always reprograms.
 */
static void output_set(DEV_Buzzer_s* buz, uint16_t freq_hz)
{
    if (freq_hz == buz->current_hz)
    {
        return;
    }

    if (freq_hz == DEV_BUZZER_REST)
    {
        PLAT_PWM_SetDutyPercent(buz->pwm, 0.0f);
        buz->current_hz = 0u;
        return;
    }

    /* Frequency and duty are set together so the compare value is derived from
     * the new period, not the previous one. */
    PLAT_PWM_SetFreqAndDuty(buz->pwm, freq_hz, buz->volume);
    buz->current_hz = freq_hz;
}

/**
 * @brief Load the note at the current sequence position.
 * @return true if a note was loaded, false if the sequence ended.
 */
static bool load_current(DEV_Buzzer_s* buz)
{
    const DEV_Buzzer_Tone_s* tone = &buz->seq[buz->index];

    /* Zero duration terminates the sequence. */
    if (tone->ms == 0u)
    {
        return false;
    }

    buz->remaining = ms_to_ticks(buz, tone->ms);
    output_set(buz, tone->freq_hz);
    return true;
}

/**
 * @brief Silence the output and clear playback state.
 */
static void halt(DEV_Buzzer_s* buz)
{
    output_set(buz, DEV_BUZZER_REST);
    PLAT_PWM_Stop(buz->pwm);

    UTIL_Seq_Stop(&buz->seq_player);

    buz->seq       = NULL;
    buz->index     = 0;
    buz->remaining = 0;
    buz->loop      = false;
    buz->playing   = false;
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

DEV_Buzzer_s* DEV_Buzzer_Create(PWM_Instance_s* pwm, uint32_t tick_hz, float volume)
{
    if (pwm == NULL || tick_hz == 0u)
    {
        return NULL;
    }

    DEV_Buzzer_s* buz = PLAT_malloc(sizeof(DEV_Buzzer_s));
    if (buz == NULL)
    {
        return NULL;
    }

    buz->pwm        = pwm;
    buz->tick_hz    = tick_hz;
    buz->volume     = clamp_volume(volume);
    buz->seq        = NULL;
    buz->index      = 0;
    buz->loop       = false;
    buz->remaining  = 0;
    buz->current_hz = 0;
    buz->playing    = false;

    UTIL_Seq_Init(&buz->seq_player);
    buz->seq_ticks = 0u;

    /* Leave the output quiet; the timer is started on the first note. */
    PLAT_PWM_SetDutyPercent(pwm, 0.0f);

    return buz;
}

void DEV_Buzzer_Tick(DEV_Buzzer_s* buz)
{
    if (buz == NULL || !buz->playing)
    {
        return;
    }

    /* Milliseconds derive from an accumulated tick count rather than a per-tick
     * quotient: 1000u / tick_hz truncates to 0 for any tick_hz above 1000 (the
     * elapsed clock would never advance and a note would hang forever) and rounds
     * down otherwise, understating elapsed time at every rate that does not divide
     * 1000 evenly — e.g. tick_hz=300 read as a 3 ms step instead of 3.33, playing
     * ~10% slow. ticks * 1000 / tick_hz is exact for every rate; seq_ticks resets
     * per sequence (see the struct comment) so the multiplication cannot overflow. */
    buz->seq_ticks++;
    const uint32_t now_ms = (buz->seq_ticks * 1000u) / buz->tick_hz;

    if (UTIL_Seq_IsPlaying(&buz->seq_player))
    {
        if (!UTIL_Seq_Step(&buz->seq_player, now_ms))
        {
            halt(buz);
            return;
        }

        output_set(buz, UTIL_Seq_Out(&buz->seq_player)[0]);
        return;
    }

    if (buz->remaining > 0u)
    {
        buz->remaining--;
    }

    if (buz->remaining > 0u)
    {
        return; /* current note still sounding */
    }

    /* Single tone: nothing follows it. */
    if (buz->seq == NULL)
    {
        halt(buz);
        return;
    }

    buz->index++;

    if (load_current(buz))
    {
        return;
    }

    /* Sequence exhausted. */
    if (!buz->loop)
    {
        halt(buz);
        return;
    }

    buz->index = 0;
    if (!load_current(buz))
    {
        /* An empty sequence would otherwise spin here every tick. */
        halt(buz);
    }
}

void DEV_Buzzer_Beep(DEV_Buzzer_s* buz, uint16_t freq_hz, uint16_t ms)
{
    if (buz == NULL || ms == 0u)
    {
        return;
    }

    /* Tick checks the sequence player first, so leaving it running here would
     * make this call silently ineffective — the old sequence would keep
     * driving the output instead of this tone. */
    UTIL_Seq_Stop(&buz->seq_player);

    buz->seq       = NULL;
    buz->index     = 0;
    buz->loop      = false;
    buz->remaining = ms_to_ticks(buz, ms);
    buz->playing   = true;

    PLAT_PWM_Start(buz->pwm);
    output_set(buz, freq_hz);
}

void DEV_Buzzer_Play(DEV_Buzzer_s* buz, const DEV_Buzzer_Tone_s* tones, bool loop)
{
    if (buz == NULL || tones == NULL)
    {
        return;
    }

    buz->seq     = tones;
    buz->index   = 0;
    buz->loop    = loop;
    buz->playing = true;

    UTIL_Seq_Stop(&buz->seq_player);

    PLAT_PWM_Start(buz->pwm);

    if (!load_current(buz))
    {
        halt(buz); /* sequence was empty */
    }
}

void DEV_Buzzer_PlaySeq(DEV_Buzzer_s* buz, const UTIL_Seq_Frame_s* frames, bool loop)
{
    if (buz == NULL)
    {
        return;
    }

    /* Stops the legacy path and silences the output, so the two cannot overlap. */
    halt(buz);

    /* Ticks are counted from the start of this sequence (see the struct comment on
     * seq_ticks), so the sequence always starts at time 0 on its own clock. */
    buz->seq_ticks = 0u;

    if (!UTIL_Seq_Play(&buz->seq_player, frames, loop, buz->seq_ticks))
    {
        return;
    }

    buz->playing = true;

    PLAT_PWM_Start(buz->pwm);
    output_set(buz, UTIL_Seq_Out(&buz->seq_player)[0]);
}

void DEV_Buzzer_Stop(DEV_Buzzer_s* buz)
{
    if (buz != NULL)
    {
        halt(buz);
    }
}

bool DEV_Buzzer_IsPlaying(const DEV_Buzzer_s* buz) { return (buz != NULL) && buz->playing; }

void DEV_Buzzer_SetVolume(DEV_Buzzer_s* buz, float volume)
{
    if (buz == NULL)
    {
        return;
    }

    buz->volume = clamp_volume(volume);

    /* Apply at once if a pitch is sounding, so a volume change is audible
     * without waiting for the next note. */
    if (buz->playing && buz->current_hz != DEV_BUZZER_REST)
    {
        PLAT_PWM_SetDutyPercent(buz->pwm, buz->volume);
    }
}
