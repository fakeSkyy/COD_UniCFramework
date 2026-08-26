/**
 * @file app_indicator.c
 * @author Gao Xing
 * @date 2026/8/17
 * @version 1.0
 */

#include "app_indicator.h"

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "dev_buzzer.h"
#include "dev_ws2812.h"
#include "plat_task.h"
#include "util_log.h"
#include "util_seq.h"

/* ==========================================================================
 * One LED, several things to say
 * ==========================================================================
 *
 * The status LED reports whichever raised condition ranks highest, and falls back to
 * the heartbeat when nothing is raised. The heartbeat is not a special case in the
 * code — it is the lowest-ranked row of the same table as everything else.
 *
 * @par Why one task and not one per indication
 * There is one LED. Two tasks driving it would interleave their patterns into
 * something that reads as neither, and coordinating them would need a mutex whose
 * only job is to serialise access to a thing that should have had one owner. So this
 * task owns the LED outright, and subsystems report *conditions* through
 * App_Indicator_Set rather than blink patterns. What a raised condition looks like is
 * this file's business, which is also what keeps the colour and timing decisions in
 * one place instead of spread across every detector.
 *
 * @par Why a pattern is data rather than a function
 * Every pattern here is the same shape: flash N times at some colour, with some on
 * time, then hold dark for the rest of a fixed beat. Writing that once and varying
 * four numbers means a new condition cannot get its timing subtly wrong, and the beat
 * stays exactly 1 s for all of them — which matters because a beat that changed
 * length with the condition would make "the light stopped" ambiguous with "the light
 * is showing something slower".
 *
 * A pattern that genuinely does not fit this shape — a fade, a two-colour
 * alternation — is the point at which the table becomes the wrong structure. It is
 * not there yet.
 *
 * @par Why the state is one word
 * A bitmask, set and cleared without a critical section. Detectors run wherever they
 * run: a CAN dropout is noticed in an interrupt, a low battery in an ADC task. A
 * single-word read-modify-write is not atomic on this core, so raising two conditions
 * from two contexts could in principle lose one — see the note on the mask below for
 * why that is acceptable here and what would change if it were not.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Beat                                                                     */
/* ========================================================================= */

/**
 * @brief One whole beat, so every pattern repeats once a second.
 *
 * Shared by all conditions deliberately: the beat is the unit someone watching the
 * board learns to expect, so only what happens *inside* it should change. A pattern
 * with its own period would make a stopped indicator hard to tell from a slow one.
 */
#define INDICATOR_PERIOD_MS 1000u

/** @brief How long the LED is on for each flash, and the gap between flashes. */
#define INDICATOR_ON_MS 50u
#define INDICATOR_GAP_MS 50u

/**
 * @brief Most flashes any pattern may ask for.
 *
 * The flashes have to fit inside one beat with dark time left over, which is what
 * bounds this: at 50 ms on and 50 ms gap a flash costs 100 ms, so nine would leave
 * 100 ms of tail and ten would leave none. Nine is also well past what anyone counts
 * reliably by eye.
 */
#define INDICATOR_MAX_FLASHES 9u

_Static_assert(INDICATOR_PERIOD_MS > (INDICATOR_MAX_FLASHES * INDICATOR_ON_MS) +
                                         ((INDICATOR_MAX_FLASHES - 1u) * INDICATOR_GAP_MS),
               "the longest pattern does not fit inside one beat");

/** @brief Largest fault code that can be blinked out; see App_Indicator_SetFault. */
#define INDICATOR_FAULT_CODE_MAX INDICATOR_MAX_FLASHES

/**
 * @brief How often the task wakes to advance the pattern, milliseconds.
 *
 * 25 ms, which divides the on time, the gap, and the shortest tail any pattern
 * produces (150 ms at nine flashes). A tick that did not divide them would put a
 * flash boundary between two wake-ups and make that flash a tick too long — 20 ms
 * looks like the obvious choice and is wrong for exactly this reason: 50 % 20 is
 * 10, so every flash would end late by turns.
 *
 * 40 wake-ups per beat.
 *
 * @par Why the body no longer sleeps through each flash
 * It used to run the pattern with consecutive PLAT_Task_DelayUntil calls, which
 * meant the task was occupied for the whole beat and could drive exactly one
 * device. Waking on a fixed tick and asking the player what to output leaves the
 * task free between ticks. The cost is more frequent wake-ups doing less each —
 * accepted deliberately.
 *
 * @par The millisecond assumption this relies on, and why it is not asserted here
 * beat_step feeds PLAT_Task_TickNow() to UTIL_Seq_Step, which wants milliseconds.
 * plat_task.h documents a tick as "a scheduler period, not a fixed unit" — the
 * two only coincide because this build's FreeRTOSConfig.h sets
 * configTICK_RATE_HZ to 1000. That macro is a vendor (impl-layer) symbol; this
 * file is application code and, per the composition-root rule, must not name
 * one — board_devices.c is the only translation unit allowed to see both
 * platform and vendor headers. So the one-tick-equals-one-ms assumption cannot
 * be given a _Static_assert in this file without breaking that rule. The
 * nearest place that could state it is 04_impl/rtos/freertos/task/impl_task.c,
 * which already includes FreeRTOS.h and is where PLAT_Task_TickNow is actually
 * implemented as xTaskGetTickCount() — out of scope for this change, and noted
 * in the task report instead.
 */
#define INDICATOR_TICK_MS 25u

_Static_assert(INDICATOR_ON_MS % INDICATOR_TICK_MS == 0u &&
                   INDICATOR_GAP_MS % INDICATOR_TICK_MS == 0u,
               "the tick must divide the on time and the gap, or a flash is mistimed");

/**
 * @brief Frames one beat can need, worst case.
 *
 * Nine flashes is nine lit frames, eight gaps between them, one tail of dark,
 * and the terminator: 19. Derived from the flash ceiling rather than written
 * down, so raising that ceiling resizes this too.
 */
#define INDICATOR_MAX_FRAMES (INDICATOR_MAX_FLASHES + (INDICATOR_MAX_FLASHES - 1u) + 2u)

/* ========================================================================= */
/*  Patterns                                                                 */
/* ========================================================================= */

/**
 * @brief What one condition looks like.
 */
typedef struct
{
    uint8_t     r;       /**< Colour, 0..255 per channel.                */
    uint8_t     g;       /**<                                            */
    uint8_t     b;       /**<                                            */
    uint8_t     flashes; /**< Flashes per beat; 0 uses the fault code.    */
    const char* name;    /**< For the log line when the condition wins.  */
} indicator_pattern_s;

/**
 * @brief One row per condition, indexed by App_Indicator_Condition_e.
 *
 * Designated initialisers rather than positional ones, so inserting a condition in
 * the middle of the enum cannot silently shift every pattern by one — the rows follow
 * the names, not the order they are written in.
 *
 * Brightness is kept low (80, not 255) because this LED is looked at directly from a
 * few centimetres and full brightness is genuinely unpleasant; it also draws less
 * from whatever rail the board is on.
 *
 * Colours are chosen to survive being seen badly: green and red are unmistakable, and
 * amber is far enough from both. A pattern distinguished from another only by its
 * flash count is fine; one distinguished only by hue is not, which is why the flash
 * counts differ too.
 */
static const indicator_pattern_s patterns[INDICATOR_CONDITION_COUNT] = {
    /* Two flashes rather than one: it makes the beat unmistakable at a glance and
     * distinguishes a live board from a slow blink or a reflection. */
    [INDICATOR_HEARTBEAT] = {.r = 0u, .g = 80u, .b = 0u, .flashes = 2u, .name = "alive"},

    [INDICATOR_CAN_LOST] = {.r = 80u, .g = 40u, .b = 0u, .flashes = 3u, .name = "CAN lost"},

    /* Blue, and the only condition that uses it: a device that stopped answering
     * must not be confused with a bus that is merely quiet, because the recovery
     * differs — check the log for which one. */
    [INDICATOR_DEVICE_LOST] = {.r = 0u, .g = 0u, .b = 80u, .flashes = 5u, .name = "device lost"},

    [INDICATOR_LOW_BATTERY] = {.r = 80u, .g = 20u, .b = 0u, .flashes = 4u, .name = "low battery"},

    /* flashes = 0 means "use the fault code", which is the one pattern whose length
     * is decided at run time. */
    [INDICATOR_FAULT] = {.r = 80u, .g = 0u, .b = 0u, .flashes = 0u, .name = "fault"},
};

/* ========================================================================= */
/*  Alert tone                                                               */
/* ========================================================================= */

/**
 * @brief Duty cycle for the alert tone, percent.
 *
 * 60, louder than mid-scale: this is meant to be noticed once, not lived with — see
 * DEV_Buzzer_Create's documentation of what volume means for a passive buzzer.
 */
#define INDICATOR_BUZZER_VOLUME 60.0f

/**
 * @brief Rate DEV_Buzzer_Tick is called at, matching the beat's own wake rate.
 *
 * Derived from INDICATOR_TICK_MS rather than written as a separate number, so the
 * two can never desync: DEV_Buzzer_Create converts note durations using this value,
 * and if it disagreed with how often Tick is actually called every note would run
 * at the wrong length. 1000u first keeps this in integer arithmetic — the buzzer's
 * own rounding for a rate that does not divide 1000 evenly is documented on
 * DEV_Buzzer_Create, and INDICATOR_TICK_MS does divide it here (1000 / 25 = 40).
 */
#define INDICATOR_BUZZER_TICK_HZ (1000u / INDICATOR_TICK_MS)

/**
 * @brief The one alert: three short beeps, then silence.
 *
 * Deliberately not a siren. A continuous tone masks whatever alert comes after it
 * and is intolerable to work next to, so this plays once and stops — App_Indicator
 * only ever calls DEV_Buzzer_PlaySeq with loop = false for it. Three beeps is
 * short enough not to overstay and long enough to be unmistakable against a single
 * accidental blip.
 *
 * Frame durations are whole multiples of INDICATOR_TICK_MS, asserted below one at a
 * time: this table is played by the same task that steps the LED at that rate, and
 * a duration that did not divide it would end a note early or late by however much
 * it missed by, the same hazard INDICATOR_ON_MS/INDICATOR_GAP_MS avoid for the LED.
 *
 * The durations are named constants rather than literals inside the initialiser so
 * the _Static_assert below can check each one directly: indexing into alert_seq
 * itself is not an integer constant expression in strict C (only some compilers'
 * extensions fold it), so the array's own elements cannot be asserted on portably.
 */
#define INDICATOR_ALERT_BEEP_MS 100u
#define INDICATOR_ALERT_GAP_MS 75u

static const UTIL_Seq_Frame_s alert_seq[] = {
    {.ch = {DEV_NOTE_A5, 0u, 0u, 0u}, .ms = INDICATOR_ALERT_BEEP_MS, .ramp = false},
    {.ch = {0u, 0u, 0u, 0u}, .ms = INDICATOR_ALERT_GAP_MS, .ramp = false},
    {.ch = {DEV_NOTE_A5, 0u, 0u, 0u}, .ms = INDICATOR_ALERT_BEEP_MS, .ramp = false},
    {.ch = {0u, 0u, 0u, 0u}, .ms = INDICATOR_ALERT_GAP_MS, .ramp = false},
    {.ch = {DEV_NOTE_A5, 0u, 0u, 0u}, .ms = INDICATOR_ALERT_BEEP_MS, .ramp = false},
    {.ms = 0u} /* terminator */
};

_Static_assert(INDICATOR_ALERT_BEEP_MS % INDICATOR_TICK_MS == 0u &&
                   INDICATOR_ALERT_GAP_MS % INDICATOR_TICK_MS == 0u,
               "every alert frame must be a whole multiple of the indicator tick");

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

/**
 * @brief The one LED in the chain, and its encoded waveform.
 *
 * One WS2812, so DEV_WS2812_BUF_BYTES(1) — 24 bytes of waveform plus the 100 latch
 * bytes that commit it. The macro is used rather than a literal because forgetting
 * the latch allowance is the mistake it exists to prevent: Init would refuse the
 * buffer, but only at run time.
 *
 * @par If this ever becomes DMA-driven
 * The board entry uses SPI_XFER_IT, so the buffer only has to be readable by the CPU
 * and ordinary `.bss` is fine. Switching that entry to SPI_XFER_DMA would make this
 * buffer unreachable by the DMA controller — `.bss` is in DTCM on this part — and the
 * SPI backend would then refuse the transfer rather than send nothing. The fix at
 * that point is a linker section in AXI SRAM, not a change here.
 */
static DEV_WS2812_s led_dev;
static uint8_t      led_buf[DEV_WS2812_BUF_BYTES(1u)];

/** @brief Whether the LED is usable; false when its Init failed. */
static bool led_ready;

/**
 * @brief The buzzer, or NULL when it never came up.
 *
 * NULL either because Board_BuzzerPWM() itself returned NULL (that peripheral did
 * not come up) or because DEV_Buzzer_Create rejected it — same handling either
 * way, everywhere this is used: a missing buzzer degrades the indicator to LED-only
 * rather than making it fail, mirroring how led_ready already covers a missing LED.
 */
static DEV_Buzzer_s* buzzer;

/**
 * @brief Which conditions are raised, one bit per App_Indicator_Condition_e.
 *
 * @par Why volatile and why that is not the same as safe
 * volatile stops the compiler caching it across the task's loop, which is necessary
 * because interrupts write it. It does not make the read-modify-write in
 * App_Indicator_Set atomic: on Cortex-M7 that is a load, an OR, and a store, and an
 * interrupt landing between the load and the store would have its own bit overwritten.
 *
 * Accepted rather than guarded, because of what is lost when it happens: one
 * condition stays invisible on an LED until the next time its detector raises it, and
 * detectors are expected to be level-triggered (App_Indicator_Set is idempotent
 * precisely so they can call it every cycle). A critical section here would be
 * disabling interrupts to change what colour a light is.
 *
 * If a detector is ever genuinely edge-triggered — raises once and never again — this
 * becomes a real dropped report, and the fix is a critical section in Set, not a
 * change here.
 */
static volatile uint32_t raised;

/** @brief Flash count for INDICATOR_FAULT, set by App_Indicator_SetFault. */
static volatile uint8_t fault_code;

/**
 * @brief Absolute deadline the beat is timed from.
 *
 * Seeded by bring-up and advanced by every phase, so the 1 Hz rate holds regardless
 * of how long the transfers and any preemption took. A relative delay would add the
 * body's own execution time to every beat and drift.
 */
static uint32_t cursor;

/** @brief Condition shown on the previous beat, so a change is logged once. */
static App_Indicator_Condition_e shown = INDICATOR_HEARTBEAT;

/**
 * @brief Flash count of the pattern currently loaded.
 *
 * Compared alongside @c shown because INDICATOR_FAULT's length comes from a
 * run-time code: the condition can stay the same while the pattern it should
 * blink changes, and without this the new code would not take effect until some
 * other condition intervened.
 */
static unsigned shown_flashes;

/**
 * @brief The pattern being played, and the frames it plays from.
 *
 * The buffer is written by build_pattern and read by the player, so it must
 * outlive every Step — file scope rather than a local, which would be a dangling
 * pointer the moment build_pattern returned.
 */
static UTIL_Seq_s       player;
static UTIL_Seq_Frame_s frames[INDICATOR_MAX_FRAMES];

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief Light the LED at a colour, or turn it off.
 *
 * A failed transmit is ignored rather than logged: at several flashes a second a
 * persistent failure would flood the log, and the visible symptom — a dark LED — is
 * already the report.
 *
 * @param r  Colour when lighting; pass 0,0,0 to go dark.
 * @param g  As above.
 * @param b  As above.
 */
static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_ready)
    {
        return;
    }

    DEV_WS2812_SetPixel(&led_dev, 0u, r, g, b);
    DEV_WS2812_Show(&led_dev);
}

/* ========================================================================= */
/*  Pattern                                                                  */
/* ========================================================================= */

/**
 * @brief Fill the frame buffer with one beat of a condition's pattern.
 *
 * Flashes at the pattern's colour, then holds dark for whatever is left of the
 * beat — computed rather than written down, so changing the flash count or the
 * on time keeps the 1 Hz rate instead of drifting off it.
 *
 * @param p        Pattern to render.
 * @param flashes  Flashes this beat, already clamped to INDICATOR_MAX_FLASHES.
 */
static void build_pattern(const indicator_pattern_s* p, unsigned flashes)
{
    unsigned n = 0u;

    for (unsigned i = 0u; i < flashes; i++)
    {
        frames[n].ch[0] = p->r;
        frames[n].ch[1] = p->g;
        frames[n].ch[2] = p->b;
        frames[n].ms    = INDICATOR_ON_MS;
        frames[n].ramp  = false;
        n++;

        /* The gap goes between flashes, not after the last one — that dark time
         * is the tail below, and emitting both would overrun the beat. */
        if (i + 1u < flashes)
        {
            frames[n].ch[0] = 0u;
            frames[n].ch[1] = 0u;
            frames[n].ch[2] = 0u;
            frames[n].ms    = INDICATOR_GAP_MS;
            frames[n].ramp  = false;
            n++;
        }
    }

    const uint32_t used = (flashes * INDICATOR_ON_MS) + ((flashes - 1u) * INDICATOR_GAP_MS);

    /* The clamp on flashes plus the _Static_assert on the period are what keep
     * this subtraction from wrapping to a ~49-day frame. */
    frames[n].ch[0] = 0u;
    frames[n].ch[1] = 0u;
    frames[n].ch[2] = 0u;
    frames[n].ms    = (uint16_t) (INDICATOR_PERIOD_MS - used);
    frames[n].ramp  = false;
    n++;

    frames[n].ms = 0u; /* terminator */
}

/* ========================================================================= */
/*  Selection                                                                */
/* ========================================================================= */

/**
 * @brief The highest-ranked raised condition, or INDICATOR_HEARTBEAT if none is.
 *
 * Scans downward and stops at the first hit, so rank is expressed by enum order
 * alone — adding a condition needs no change here.
 */
static App_Indicator_Condition_e active(void)
{
    const uint32_t mask = raised;

    for (unsigned c = (unsigned) INDICATOR_CONDITION_COUNT; c-- > 1u;)
    {
        if ((mask & (1u << c)) != 0u)
        {
            return (App_Indicator_Condition_e) c;
        }
    }

    return INDICATOR_HEARTBEAT;
}

/* ========================================================================= */
/*  Beat                                                                     */
/* ========================================================================= */

/**
 * @brief Prepare the status LED and seed the beat timeline.
 *
 * @par Why a failure here does not stop the task
 * The LED is a diagnostic. A board where it is not populated should still run its real
 * work, so this reports the problem once and the beat then keeps its timing with
 * nothing to show — which is better than refusing to run the very task that proves the
 * scheduler is alive.
 *
 * One LED, so one pixel's worth of buffer. The chain length is the argument to change
 * if the board ever grows a strip; nothing else here depends on it.
 */
static void beat_init(void)
{
    /* Catches a condition added to the enum without a row here. The table's length
     * cannot catch it — a designated initialiser leaves the missing slot zeroed,
     * and a zeroed row is a legal one meaning "use the fault code". A NULL name is
     * the one field no real row leaves unset, so it is what makes the omission
     * visible. Logged rather than asserted: a mis-blinking LED is not worth
     * refusing to boot over, and the line says exactly which rank is missing. */
    for (unsigned i = 0u; i < INDICATOR_CONDITION_COUNT; i++)
    {
        if (patterns[i].name == NULL)
        {
            UTIL_LOG_E("led", "condition %u has no pattern row; it will blink as a fault", i);
        }
    }

    led_ready = DEV_WS2812_Init(&led_dev, Board_StatusLed(), led_buf, sizeof led_buf, 1u);

    if (!led_ready)
    {
        /* Once, not per beat. */
        UTIL_LOG_W("led", "status LED unavailable; indicator runs without it");
    }
    else
    {
        /* Registered with the driver's zero timeout, so it never expires: a strip
         * is written and never read, and silence proves nothing about it. It is
         * here for the report — a device list that omits the LED makes "is
         * everything registered" harder to check than it needs to be. */
        (void) DEV_Watchdog_Register(&led_dev.wd, NULL);
    }

    /* Board_BuzzerPWM() returns NULL when that peripheral did not come up, and
     * DEV_Buzzer_Create returns NULL on top of that for bad arguments or an
     * allocation failure — passing a NULL pwm through is one of those bad
     * arguments, so this needs no separate branch for it. Either way buzzer stays
     * NULL and every other buzzer call in this file already checks for that,
     * same as led_ready covers a missing LED: a degraded indicator, not a dead
     * one. */
    buzzer =
        DEV_Buzzer_Create(Board_BuzzerPWM(), INDICATOR_BUZZER_TICK_HZ, INDICATOR_BUZZER_VOLUME);

    if (buzzer == NULL)
    {
        UTIL_LOG_W("led", "buzzer unavailable; faults will not be sounded");
    }

    UTIL_Seq_Init(&player);

    /* Left at a count no real pattern has, so the first beat_step always builds
     * rather than comparing against a stale value. */
    shown_flashes = 0u;

    /* Seeded last, immediately before the first beat. A cursor left at zero would put
     * the first deadline in the distant past, and the pattern would then run flat out
     * with no delay until it caught up. */
    cursor = PLAT_Task_TickNow();
}

/**
 * @brief Advance whichever condition is active by one tick.
 *
 * Rebuilds the pattern only when what should be shown changes, then always
 * steps the player and writes its output — so a steady condition runs from one
 * continuous timeline instead of restarting its beat every tick, and the LED is
 * refreshed at INDICATOR_TICK_MS resolution rather than once per whole beat.
 *
 * The condition is sampled once per tick rather than latched for a whole beat,
 * but a change only takes effect at the *pattern* level: switching colour mid
 * flash would produce a beat that is neither pattern and reads as a glitch, so
 * build_pattern is only called again when cond or its flash count differs from
 * what is already loaded — the current beat finishes on the old pattern.
 */
static void beat_step(void)
{
    const App_Indicator_Condition_e cond = active();
    const indicator_pattern_s*      p    = &patterns[cond];

    /* A zero flash count means the pattern blinks a run-time code — currently
     * only INDICATOR_FAULT. Read once into a local because it is volatile and
     * an interrupt could change it between the clamp and the build. */
    const uint8_t code = fault_code;

    unsigned flashes = (p->flashes != 0u) ? p->flashes : code;

    if (flashes == 0u)
    {
        flashes = 1u;
    }
    if (flashes > INDICATOR_MAX_FLASHES)
    {
        flashes = INDICATOR_MAX_FLASHES;
    }

    /* Rebuilt only when what is being shown changes, so a steady condition keeps
     * one continuous timeline instead of restarting its beat every tick. The
     * flash count is part of that comparison because a fault code can change
     * without the condition changing. */
    if (cond != shown || flashes != shown_flashes)
    {
        shown         = cond;
        shown_flashes = flashes;

        UTIL_LOG_I("led", "indicating: %s", p->name);

        build_pattern(p, flashes);
        (void) UTIL_Seq_Play(&player, frames, true, PLAT_Task_TickNow());
    }

    /* PLAT_Task_TickNow() is a scheduler-tick count, not documented as
     * milliseconds — see the comment on INDICATOR_TICK_MS for why this build's
     * configTICK_RATE_HZ of 1000 makes the two numerically the same, and why
     * that assumption cannot be asserted from this file. */
    (void) UTIL_Seq_Step(&player, PLAT_Task_TickNow());

    const uint16_t* out = UTIL_Seq_Out(&player);

    led_set((uint8_t) out[0], (uint8_t) out[1], (uint8_t) out[2]);

    /* Once per iteration, unconditionally — DEV_Buzzer_Tick documents that skipping
     * a call leaves whatever note is sounding stuck indefinitely, and this is the
     * only place in the program that runs at the rate DEV_Buzzer_Create was told to
     * expect. NULL when the buzzer never came up; every DEV_Buzzer_* call in this
     * file guards for that the same way led_set already does for led_ready. */
    if (buzzer != NULL)
    {
        DEV_Buzzer_Tick(buzzer);
    }

    /* Return value ignored, which is deliberate here and nowhere else: a missed
     * deadline in this task means something more important was running, which is
     * exactly what its low priority is for. Reporting it would log noise about
     * the system working as designed. */
    (void) PLAT_Task_DelayUntil(&cursor, INDICATOR_TICK_MS);
}

/* ========================================================================= */
/*  Task                                                                     */
/* ========================================================================= */

/**
 * @brief Stack and control block for this module's task.
 *
 * Sized here rather than in the task list because the depth follows from what the body
 * does: it sets one pixel and sleeps — no float, no logging in the loop except the one
 * line when the shown condition changes — so the 1 KB platform floor is enough. Both
 * must outlive the task, hence file scope: PLAT_Task_Create does not allocate.
 *
 * Confirm with PLAT_Task_StackFree rather than trusting this estimate.
 */
static uint8_t stack[1024];
static Task_s  task;

/**
 * @brief The task body: bring up the LED, then beat forever.
 *
 * Never returns. A task body that falls off its end is deleting itself, which faults
 * on a port not built for it.
 *
 * Unlike most bodies this one has nothing to park for: bring-up cannot fail in a way
 * that makes the task pointless, since an indicator with no LED still keeps the timing
 * and beat_init has already logged the reason.
 *
 * @param arg  Unused.
 */
static void body(void* arg)
{
    (void) arg;

    beat_init();

    for (;;)
    {
        beat_step();
    }
}

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

bool App_Indicator_StartTask(uint8_t priority)
{
    return PLAT_Task_Create(&task, body, NULL, "indicator", stack, sizeof stack, priority);
}

void App_Indicator_Set(App_Indicator_Condition_e cond, bool on)
{
    /* Rejected rather than masked into range: a caller passing something out of range
     * has a bug, and silently lighting the wrong condition would hide it. The
     * heartbeat is refused for a different reason — it is the absence of every other
     * condition, so raising it would be meaningless and clearing it would be a way to
     * turn the LED off entirely. */
    if (cond <= INDICATOR_HEARTBEAT || cond >= INDICATOR_CONDITION_COUNT)
    {
        return;
    }

    const uint32_t bit = 1u << (unsigned) cond;

    if (on)
    {
        raised |= bit;
    }
    else
    {
        raised &= ~bit;
    }
}

void App_Indicator_SetFault(uint8_t code)
{
    if (code == 0u)
    {
        App_Indicator_Set(INDICATOR_FAULT, false);
        return;
    }

    /* Clamped rather than refused: a fault worth reporting should still light the LED
     * even if its code is too large to blink, and the exact value belongs in a log
     * line anyway. */
    fault_code = (code > INDICATOR_FAULT_CODE_MAX) ? INDICATOR_FAULT_CODE_MAX : code;

    App_Indicator_Set(INDICATOR_FAULT, true);

    /* Sounded here rather than exposing a separate App_Indicator_SoundAlert: every
     * caller of this function already has something worth an audible alert — that
     * is what a fault code means — so a second entry point would only give callers
     * a way to light the LED without the sound, which is not a case anyone needs.
     * DEV_Buzzer_PlaySeq replaces whatever is already sounding rather than queuing,
     * so calling this again while the alert is still playing restarts it instead of
     * layering two, which is what a caller re-asserting the same fault every cycle
     * (detectors are level-triggered, same as App_Indicator_Set) would otherwise do.
     * loop = false: one alert, then silence — a siren would mask whatever comes
     * next and is intolerable to sit next to. NULL-checked, same as every other
     * buzzer call in this file. */
    if (buzzer != NULL)
    {
        DEV_Buzzer_PlaySeq(buzzer, alert_seq, false);
    }
}

App_Indicator_Condition_e App_Indicator_Active(void) { return active(); }
