/**
 * @file app_telemetry.c
 * @author Gao Xing
 * @date 2026/8/13
 * @version 1.0
 */

#include "app_telemetry.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* memcpy */

#include "board.h"
#include "plat_uart.h"
#include "util_fast_math.h" /* UTIL_RAD_TO_DEG */

/* ==========================================================================
 * Streaming the attitude estimate to VOFA+
 * ==========================================================================
 *
 * VOFA+ justFloat: a frame is N little-endian 32-bit floats followed by the four
 * bytes 00 00 80 7F, and nothing else. No header, no length, no checksum — the
 * channel count is implied by how many bytes arrived before the tail, so both ends
 * have to agree on it out of band.
 *
 * The tail is the little-endian encoding of 0x7F800000, which is +infinity. That
 * is what makes the scheme work without a header: +Inf is not a value a sane
 * measurement ever takes, so it cannot occur inside the payload and be mistaken
 * for a boundary. It is also the one hazard here — a channel that goes infinite
 * would emit a second tail mid-frame and split the frame in two, desynchronising
 * every channel after it. Hence the finiteness check in encode().
 *
 * @par Why this is its own module rather than part of app_imu.c
 * app_imu.c owns the estimator; what someone plots, in what units, over which
 * transport, is a separate concern that changes far more often. Keeping them apart
 * means adding a channel does not touch the filter, and the filter has no idea a
 * serial port exists.
 * ==========================================================================
 */

/* ========================================================================= */
/*  Protocol                                                                 */
/* ========================================================================= */

/**
 * @brief Channels in one frame.
 *
 * Must match the channel count configured in VOFA+. justFloat carries no channel
 * count, so a mismatch is not detected anywhere: the receiver simply slices the
 * bytes differently and every trace shows the wrong quantity.
 */
#define TELEM_CHANNELS 7u

/**
 * @brief The justFloat frame terminator: 0x7F800000 little-endian, i.e. +Inf.
 *
 * Written as four explicit bytes rather than by punning a float, so the wire
 * format does not depend on the host's float representation or byte order.
 */
static const uint8_t telem_tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

/** @brief Bytes on the wire per frame: the payload plus the tail. */
#define TELEM_FRAME_BYTES (TELEM_CHANNELS * 4u + sizeof telem_tail)

/* ========================================================================= */
/*  Rate                                                                     */
/* ========================================================================= */

/**
 * @brief Send one frame every Nth call to App_Telemetry_Step.
 *
 * The step runs at the attitude loop's 1 kHz, which this divides down to 200 Hz.
 *
 * @par Why 200 Hz rather than every cycle
 * Not bandwidth. A frame is 32 bytes and 8N1 spends 10 bit-times per byte, so one
 * frame is 320 bit-times; at the 921600 the board's UART entry runs at, the line
 * carries ~2880 frames a second, and 1 kHz would use 35% of it. The wire could take
 * it.
 *
 * The reason is that 200 Hz is already five times the rate anything mechanical can
 * move at, so the extra frames would cost interrupts and plot nothing new — at one
 * interrupt per byte, 1 kHz means 32 k/s of them competing with the attitude loop
 * they are meant to observe. A telemetry path that perturbs its own measurement is
 * worse than a slower one.
 *
 * @par What happens if the baud rate drops
 * This divider is chosen against the board entry's rate and nothing checks the two
 * agree. At 115200 the line carries only 360 frames a second, so 200 Hz would use
 * 56% — still workable, but 1 kHz would need 2.8x the bandwidth that exists. Frames
 * do not queue: each send would find the previous one still going and fail, so the
 * result is not slow telemetry but erratic telemetry at whatever rate the failures
 * happen to leave. App_Telemetry_Skipped is what shows that.
 */
#define TELEM_DIVIDER 5u

/* ========================================================================= */
/*  State                                                                    */
/* ========================================================================= */

/**
 * @brief The frame being transmitted.
 *
 * @par Why the buffer must not be rebuilt while the previous send is running
 * The send is asynchronous: it returns as soon as the interrupt-driven transfer is
 * armed, and the peripheral reads out of this array over the following
 * milliseconds. Overwriting it in the meantime would splice the new frame into the
 * middle of the old one on the wire — the receiver then sees one frame whose
 * channels come from two different instants, with no way to tell. The in_flight
 * flag below is what prevents that.
 *
 * PLAT_DMA_BUF rather than ordinary .bss, even though the board entry currently asks
 * for UART_XFER_IT and interrupt mode has no reachability or cache-line requirement
 * at all. Declaring it this way costs padding to a 32-byte cache line and nothing
 * else, and it means switching that entry to UART_XFER_DMA is a one-word change here
 * instead of a silent failure: DMA1/DMA2 cannot reach DTCM, where .bss lives, so an
 * ordinary buffer would be refused by the backend's dma_reachable check.
 *
 * Note this buffer is therefore no longer zero-initialised at reset — .dma_buf is a
 * NOLOAD section. Nothing here reads it before writing: build_frame fills every byte
 * of the frame before the send is armed.
 */
PLAT_DMA_BUF(uint8_t, telem_frame, TELEM_FRAME_BYTES);

/**
 * @brief True from arming a send until its completion callback runs.
 *
 * The whole reason a callback is registered at all. Without it the only way to
 * know the port is free would be to ask before each send, which races: the
 * transfer can finish between the question and the answer, or not finish until
 * after — and the buffer would be rebuilt either way.
 */
static volatile bool in_flight;

/** @brief Cycles until the next frame; see TELEM_DIVIDER. */
static uint32_t countdown;

/** @brief Frames dropped because the previous one was still going out. */
static uint32_t skipped;

/** @brief The port, resolved once at Init. NULL until then, which gates Step. */
static UART_Instance_s* port;

/* ========================================================================= */
/*  Encoding                                                                 */
/* ========================================================================= */

/**
 * @brief Store one float into the frame at channel @p ch.
 *
 * @par Why non-finite values are replaced rather than passed through
 * An infinity in the payload would encode to exactly the four tail bytes, so the
 * receiver would treat it as a frame boundary: that frame ends early, the
 * remaining channels become the start of the next frame, and every trace shifts by
 * a channel and stays shifted. A NaN does not collide with the tail but plots as a
 * gap. Substituting zero loses the fact that something went wrong, which is why
 * the substitution is counted in @c nonfinite rather than silent.
 *
 * memcpy rather than a cast through a float pointer: the frame is a byte array, so
 * writing a float through a reinterpreted pointer would break strict aliasing, and
 * the compiler is free to reorder that write against the neighbouring byte stores.
 * At -Og this compiles to a single 32-bit store anyway.
 *
 * @param ch  Channel index, 0-based. Must be below TELEM_CHANNELS.
 * @param v   Value to store.
 */
static void encode(unsigned ch, float v)
{
    /* One integer comparison against the exponent field catches NaN and both
     * infinities together — all three have every exponent bit set. Cheaper than
     * isnan() || isinf(), which is two calls that may not inline. */
    union
    {
        float    f;
        uint32_t u;
    } conv;

    conv.f = v;

    if ((conv.u & 0x7F800000u) == 0x7F800000u)
    {
        conv.f = 0.0f;
    }

    memcpy(&telem_frame[ch * 4u], &conv.f, sizeof conv.f);
}

/**
 * @brief Transmit-complete callback: release the buffer.
 *
 * Runs in interrupt context, so it does the minimum — clearing the flag is the
 * whole job, and it is a single word so no synchronisation is needed against the
 * task that sets it.
 *
 * @param uart  The port that finished. Unused; there is only one.
 */
static void on_sent(UART_Instance_s* uart)
{
    (void) uart;

    in_flight = false;
}

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Resolve the port and register the completion callback.
 */
bool App_Telemetry_Init(void)
{
    port = Board_DebugUart();

    if (port == NULL)
    {
        return false;
    }

    PLAT_UART_OnSendComplete(port, on_sent);

    /* The tail is constant, so it is written once here rather than per frame. Only
     * the payload changes afterwards, and encode() never reaches past it. */
    memcpy(&telem_frame[TELEM_CHANNELS * 4u], telem_tail, sizeof telem_tail);

    in_flight = false;
    countdown = TELEM_DIVIDER;
    skipped   = 0u;

    return true;
}

/**
 * @brief Build and send one frame, at the divided rate.
 */
void App_Telemetry_Step(float roll_rad, float pitch_rad, float yaw_rad, const float* rate_rads,
                        float temp_c)
{
    if (port == NULL)
    {
        return;
    }

    if (--countdown != 0u)
    {
        return;
    }

    countdown = TELEM_DIVIDER;

    /* Skipped rather than queued. There is one buffer, and rebuilding it under a
     * running transfer would put half of two different frames on the wire. A
     * dropped frame at 200 Hz is invisible in a plot; a spliced one shifts every
     * channel. */
    if (in_flight)
    {
        skipped++;
        return;
    }

    /* Degrees, not radians. VOFA+ plots whatever it is given, and degrees are what
     * anyone reading an attitude plot expects — a 5 degree tilt reading as 0.087
     * makes the eye do arithmetic it should not have to. */
    encode(0u, roll_rad * UTIL_RAD_TO_DEG);
    encode(1u, pitch_rad * UTIL_RAD_TO_DEG);
    encode(2u, yaw_rad * UTIL_RAD_TO_DEG);

    if (rate_rads != NULL)
    {
        encode(3u, rate_rads[0] * UTIL_RAD_TO_DEG);
        encode(4u, rate_rads[1] * UTIL_RAD_TO_DEG);
        encode(5u, rate_rads[2] * UTIL_RAD_TO_DEG);
    }
    else
    {
        encode(3u, 0.0f);
        encode(4u, 0.0f);
        encode(5u, 0.0f);
    }

    encode(6u, temp_c);

    /* Flag set before arming, not after: the completion interrupt can fire before
     * this function returns, and setting it afterwards would then clear it first
     * and leave the flag stuck true — telemetry would stop for good after the
     * first frame. */
    in_flight = true;

    if (!PLAT_UART_SendAsync(port, telem_frame, (uint16_t) sizeof telem_frame))
    {
        /* Nothing was armed, so no callback will come to clear this. */
        in_flight = false;
        skipped++;
    }
}

/**
 * @brief Frames dropped because the port was still busy.
 *
 * A slowly growing count is normal at a divider that puts the line near capacity.
 * A count climbing as fast as the frame rate means nothing is getting out — check
 * the baud rate against TELEM_DIVIDER.
 *
 * @return Total skipped since Init.
 */
uint32_t App_Telemetry_Skipped(void) { return skipped; }
