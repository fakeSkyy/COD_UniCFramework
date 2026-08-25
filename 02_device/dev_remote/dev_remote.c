/**
 * @file dev_remote.c
 * @author Gao Xing
 * @date 2026/7/31
 * @version 1.0
 *
 * DBUS remote-control receiver (DJI DR16 class).
 *
 * The transmitter sends an 18-byte frame roughly every 14 ms over a 100000-baud
 * line. Frames are captured by idle-line framing in the UART backend, decoded in
 * interrupt context into a double buffer, and published to the application by
 * DEV_Remote_Tick.
 *
 * Concurrency: the ISR writes the back buffer and sets a flag; Tick swaps the
 * buffers under a short critical section. Readers therefore never observe a
 * half-decoded frame, and every getter within one tick sees the same instant.
 */

#include "dev_remote.h"

#include <string.h>

#include "plat_memory.h"

/* ========================================================================= */
/*  Frame layout                                                             */
/* ========================================================================= */

/** @brief Raw channel span: 11 bits, so 0..2047. */
#define RC_CH_MASK 0x07FFu

/** @brief Channel value corresponding to a centred stick. */
#define RC_CH_CENTRE 1024

/**
 * @brief Sanity bound for a mouse axis.
 *
 * The DR16 reports mouse velocity as a signed 16-bit value but never exceeds
 * this magnitude; anything larger indicates a corrupted frame.
 */
#define RC_MOUSE_MAX 32000

/* ========================================================================= */
/*  Device state                                                             */
/* ========================================================================= */

/**
 * @brief Debounce bookkeeping for one key.
 *
 * @c down_ticks counts how long the key has been continuously down, which is
 * what separates HELD from LONG_HELD. It is reset on every transition rather
 * than accumulated across presses.
 */
typedef struct
{
    uint16_t        down_ticks;
    bool            was_down; /**< Level at the previous tick, for edges. */
    bool            toggle;   /**< Flips on each press.                   */
    DEV_Key_State_e state;
} DEV_Remote_Key_s;

struct DEV_Remote_s
{
    UART_Instance_s* uart;

    /* Receive path. rx_buf is handed to the UART backend and written by DMA/IT;
     * pending holds the frame decoded in the ISR, awaiting publication.
     *
     * pending_seq is a seqlock: the ISR increments it to an odd value before
     * writing pending and back to even after, so a reader can tell whether its
     * copy was taken while a write was in progress. This keeps the handoff
     * lock-free, which matters because the device layer has no access to a
     * critical-section primitive (that would be a vendor intrinsic). */
    uint8_t                     rx_buf[DEV_REMOTE_RX_BUF_SIZE];
    volatile DEV_Remote_Input_s pending;
    volatile uint32_t           pending_seq;
    volatile bool               has_pending;

    /* Frame reassembly window, written only by the RX callback. Two frames wide so
     * that sliding it after a corrupted frame never throws away a byte that could
     * still start a good one. See remote_rx_cb for why bytes must be accumulated
     * rather than decoded where they land. */
    uint8_t  win[DEV_REMOTE_FRAME_LEN * 2u];
    uint16_t win_len;

    /* Published snapshot: only Tick writes it, so readers need no locking. */
    DEV_Remote_Input_s input;

    DEV_Remote_Key_s keys[DEV_KEY_COUNT];

    uint16_t short_ticks;
    uint16_t long_ticks;
    uint16_t lost_ticks;
    uint16_t idle_ticks; /**< Ticks since the last accepted frame. */
    bool     link_lost;

    volatile uint32_t frame_count;
    volatile uint32_t error_count;
};

/* ========================================================================= */
/*  Frame decoding (interrupt context)                                       */
/* ========================================================================= */

/**
 * @brief Clamp a centred channel to the transmitter's physical range.
 */
static int16_t clamp_channel(int32_t value)
{
    if (value > DEV_REMOTE_CH_MAX)
    {
        return (int16_t) DEV_REMOTE_CH_MAX;
    }
    if (value < -DEV_REMOTE_CH_MAX)
    {
        return (int16_t) -DEV_REMOTE_CH_MAX;
    }
    return (int16_t) value;
}

/**
 * @brief Decode one 18-byte DBUS frame.
 *
 * The channel fields are packed across byte boundaries at 11 bits each, so each
 * one is assembled from two or three source bytes.
 *
 * @param buf  Frame bytes (at least DEV_REMOTE_FRAME_LEN).
 * @param out  Destination for the decoded values.
 * @return true if the frame passed validation, false if it looked corrupted.
 */
static bool decode_frame(const uint8_t* buf, DEV_Remote_Input_s* out)
{
    int32_t ch[5];

    ch[0] = (int32_t) (((uint32_t) buf[0] | ((uint32_t) buf[1] << 8)) & RC_CH_MASK);
    ch[1] = (int32_t) ((((uint32_t) buf[1] >> 3) | ((uint32_t) buf[2] << 5)) & RC_CH_MASK);
    ch[2] = (int32_t) ((((uint32_t) buf[2] >> 6) | ((uint32_t) buf[3] << 2) |
                        ((uint32_t) buf[4] << 10)) &
                       RC_CH_MASK);
    ch[3] = (int32_t) ((((uint32_t) buf[4] >> 1) | ((uint32_t) buf[5] << 7)) & RC_CH_MASK);
    ch[4] = (int32_t) (((uint32_t) buf[16] | ((uint32_t) buf[17] << 8)) & RC_CH_MASK);

    uint8_t sw_left  = (uint8_t) (((buf[5] >> 4) & 0x03u));
    uint8_t sw_right = (uint8_t) (((buf[5] >> 4) & 0x0Cu) >> 2);

    /* A switch reads 1..3; zero means the frame did not come from a transmitter
     * in a known state, so reject rather than publish a bogus mode. This is the
     * only integrity check DBUS affords — the protocol carries no checksum. */
    if (sw_left == 0u || sw_right == 0u)
    {
        return false;
    }

    int16_t mouse_x = (int16_t) ((uint16_t) buf[6] | ((uint16_t) buf[7] << 8));
    int16_t mouse_y = (int16_t) ((uint16_t) buf[8] | ((uint16_t) buf[9] << 8));
    int16_t mouse_z = (int16_t) ((uint16_t) buf[10] | ((uint16_t) buf[11] << 8));

    if (mouse_x > RC_MOUSE_MAX || mouse_x < -RC_MOUSE_MAX || mouse_y > RC_MOUSE_MAX ||
        mouse_y < -RC_MOUSE_MAX)
    {
        return false;
    }

    for (uint8_t i = 0; i < 5u; i++)
    {
        out->ch[i] = clamp_channel(ch[i] - RC_CH_CENTRE);
    }

    out->sw[0]    = sw_left;
    out->sw[1]    = sw_right;
    out->mouse_x  = mouse_x;
    out->mouse_y  = mouse_y;
    out->mouse_z  = mouse_z;
    out->mouse_l  = (buf[12] != 0u);
    out->mouse_r  = (buf[13] != 0u);
    out->key_bits = (uint16_t) ((uint16_t) buf[14] | ((uint16_t) buf[15] << 8));

    return true;
}

/**
 * @brief Try to decode a frame ending at the tail of the reassembly window.
 *
 * @param dev  Device whose window to decode from.
 * @return true if a frame was decoded and published.
 */
static bool publish_from_window(DEV_Remote_s* dev)
{
    if (dev->win_len < DEV_REMOTE_FRAME_LEN)
    {
        return false;
    }

    /* The newest frame is the tail: a run may carry more than one frame when an idle
     * gap was missed, and only the latest transmitter state is worth acting on. */
    const uint8_t* frame = dev->win + (dev->win_len - DEV_REMOTE_FRAME_LEN);

    DEV_Remote_Input_s decoded;
    if (!decode_frame(frame, &decoded))
    {
        return false;
    }

    /* Publish under the seqlock: odd while writing, even when settled. Tick reads
     * the counter either side of its copy to detect a write that landed in the
     * middle of it. */
    dev->pending_seq++;
    dev->pending = decoded;
    dev->pending_seq++;

    dev->has_pending = true;
    dev->frame_count++;
    return true;
}

/**
 * @brief UART frame callback: accumulate bytes and decode whole frames.
 *
 * Runs in interrupt context and does no more than decode — debounce and timeout
 * belong to Tick, where the timebase is well defined. An unread pending frame is
 * overwritten on purpose: the newest transmitter state is the only one worth
 * acting on, and queueing stale stick positions would add latency.
 *
 * @par Why the bytes are accumulated rather than decoded in place
 * One call is not one frame. The UART contract says so explicitly: idle framing
 * merges two frames when the gap between them is missed, and a backend streaming
 * into a circular buffer splits a run that wraps the end of that buffer into two
 * calls. So a single delivery can be a partial frame, and decoding in place would
 * discard it — the transmitter would appear to drop roughly one frame per buffer
 * lap, which reads as an intermittent link rather than as a driver assumption.
 * A small sliding window costs 36 bytes and removes the assumption.
 */
static void remote_rx_cb(UART_Instance_s* uart, const uint8_t* data, uint16_t len)
{
    DEV_Remote_s* dev = uart->id;

    if (dev == NULL || data == NULL || len == 0u)
    {
        if (dev != NULL)
        {
            dev->error_count++;
        }
        return;
    }

    for (uint16_t i = 0u; i < len; i++)
    {
        if (dev->win_len == sizeof dev->win)
        {
            /* Window full with nothing decodable in it: drop the oldest byte and
             * slide. This is how the decoder recovers byte alignment after a
             * corrupted or truncated frame — without it, one bad byte would
             * misalign every subsequent frame for as long as the link stayed up.
             * The window holds two frames, so a slide never discards a byte that
             * could still begin a complete frame.
             *
             * Counted, because this is what a malformed line looks like from here:
             * bytes arriving that never form a frame the decoder accepts. Counting
             * the slide rather than each failed decode attempt is deliberate — a
             * window slid one byte at a time would otherwise report eighteen errors
             * for one bad frame. */
            memmove(dev->win, dev->win + 1, sizeof dev->win - 1u);
            dev->win_len--;
            dev->error_count++;
        }

        dev->win[dev->win_len++] = data[i];

        if (dev->win_len >= DEV_REMOTE_FRAME_LEN && publish_from_window(dev))
        {
            /* Consumed: start the next frame clean rather than leaving bytes that
             * would be decoded a second time. */
            dev->win_len = 0u;
        }
    }
}

/* ========================================================================= */
/*  Key debounce (task context, from Tick)                                   */
/* ========================================================================= */

/**
 * @brief Extract one key's raw level from a published frame.
 */
static bool key_raw_level(const DEV_Remote_Input_s* in, DEV_Remote_Key_e key)
{
    if (key < DEV_KEY_MOUSE_L)
    {
        return ((in->key_bits >> (uint16_t) key) & 1u) != 0u;
    }
    return (key == DEV_KEY_MOUSE_L) ? in->mouse_l : in->mouse_r;
}

/**
 * @brief Advance one key's debounce state by a tick.
 *
 * Edges (PRESSED, RELEASED) are reported for exactly one tick; levels (HELD,
 * LONG_HELD) persist. The tick counter resets on every transition, so the
 * thresholds always measure the current press rather than accumulated activity.
 */
static void key_update(DEV_Remote_Key_s* k, bool down, uint16_t short_ticks, uint16_t long_ticks)
{
    if (down != k->was_down)
    {
        k->was_down   = down;
        k->down_ticks = 0;

        if (down)
        {
            k->state  = DEV_KEY_STATE_PRESSED;
            k->toggle = !k->toggle;
        }
        else
        {
            k->state = DEV_KEY_STATE_RELEASED;
        }
        return;
    }

    if (!down)
    {
        /* Steady up: consumes the one-tick RELEASED edge. */
        k->state = DEV_KEY_STATE_UP;
        return;
    }

    if (k->down_ticks < long_ticks)
    {
        k->down_ticks++;
    }

    if (k->down_ticks >= long_ticks)
    {
        k->state = DEV_KEY_STATE_LONG_HELD;
    }
    else if (k->down_ticks >= short_ticks)
    {
        k->state = DEV_KEY_STATE_HELD;
    }
    else
    {
        /* Down, but not yet past the short threshold. Distinct from HELD so that
         * short_ticks actually means something to a caller. */
        k->state = DEV_KEY_STATE_DOWN;
    }
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

DEV_Remote_s* DEV_Remote_Create(UART_Instance_s* uart, uint16_t short_ticks, uint16_t long_ticks,
                                uint16_t lost_ticks)
{
    if (uart == NULL || lost_ticks == 0u || long_ticks <= short_ticks)
    {
        return NULL;
    }

    DEV_Remote_s* dev = PLAT_malloc(sizeof(DEV_Remote_s));
    if (dev == NULL)
    {
        return NULL;
    }

    memset(dev, 0, sizeof(DEV_Remote_s));

    dev->uart        = uart;
    dev->short_ticks = short_ticks;
    dev->long_ticks  = long_ticks;
    dev->lost_ticks  = lost_ticks;

    /* Start out lost: nothing has been received yet, and reporting a live link
     * before the first frame would let a consumer act on centred-but-unconfirmed
     * inputs. */
    dev->link_lost  = true;
    dev->idle_ticks = lost_ticks;

    /* The callback recovers the device from uart->id, so this must be set before
     * reception starts. */
    uart->id = dev;
    PLAT_UART_OnReceive(uart, remote_rx_cb);

    if (!PLAT_UART_StartReceive(uart, dev->rx_buf, (uint16_t) sizeof(dev->rx_buf)))
    {
        uart->id = NULL;
        PLAT_UART_OnReceive(uart, NULL);
        PLAT_free(dev);
        return NULL;
    }

    return dev;
}

void DEV_Remote_Tick(DEV_Remote_s* dev)
{
    if (dev == NULL)
    {
        return;
    }

    /* Claim the frame the ISR most recently decoded, without masking interrupts —
     * that would need a CMSIS intrinsic, and the device layer may not reach past
     * the platform layer for one.
     *
     * Instead the ISR stamps a sequence number before and after writing the
     * pending frame. Reading the stamps either side of the copy detects a write
     * that landed mid-copy: seq_end changing, or an odd seq_begin, both mean the
     * copy may be torn, so it is discarded and retried on the next tick. The
     * frame rate (~14 ms) is far below the tick rate, so a collision costs at
     * most one tick of latency and cannot repeat indefinitely. */
    bool               got   = false;
    DEV_Remote_Input_s fresh = {0};

    if (dev->has_pending)
    {
        uint32_t seq_before = dev->pending_seq;

        fresh = *(const DEV_Remote_Input_s*) &dev->pending;

        /* An even sequence means no write was in flight; an unchanged sequence
         * means none completed during the copy. */
        if (((seq_before & 1u) == 0u) && (dev->pending_seq == seq_before))
        {
            dev->has_pending = false;
            got              = true;
        }
    }

    if (got)
    {
        dev->input      = fresh;
        dev->idle_ticks = 0;
        dev->link_lost  = false;
    }
    else if (dev->idle_ticks < dev->lost_ticks)
    {
        dev->idle_ticks++;
    }

    if (dev->idle_ticks >= dev->lost_ticks && !dev->link_lost)
    {
        dev->link_lost = true;
    }

    /* With the link down every input reads neutral, so a consumer that never
     * checks IsLinkLost coasts to a stop instead of holding the last deflection.
     * Toggle latches survive, since they represent operator intent rather than
     * a live stick position. */
    if (dev->link_lost)
    {
        memset(&dev->input, 0, sizeof(dev->input));
    }

    for (uint8_t i = 0; i < (uint8_t) DEV_KEY_COUNT; i++)
    {
        bool down = dev->link_lost ? false : key_raw_level(&dev->input, (DEV_Remote_Key_e) i);
        key_update(&dev->keys[i], down, dev->short_ticks, dev->long_ticks);
    }
}

const DEV_Remote_Input_s* DEV_Remote_GetInput(const DEV_Remote_s* dev) { return &dev->input; }

bool DEV_Remote_IsLinkLost(const DEV_Remote_s* dev) { return dev->link_lost; }

DEV_Key_State_e DEV_Remote_GetKeyState(const DEV_Remote_s* dev, DEV_Remote_Key_e key)
{
    if (key >= DEV_KEY_COUNT)
    {
        return DEV_KEY_STATE_UP;
    }
    return dev->keys[key].state;
}

bool DEV_Remote_IsKeyDown(const DEV_Remote_s* dev, DEV_Remote_Key_e key)
{
    DEV_Key_State_e s = DEV_Remote_GetKeyState(dev, key);
    return s == DEV_KEY_STATE_PRESSED || s == DEV_KEY_STATE_DOWN || s == DEV_KEY_STATE_HELD ||
           s == DEV_KEY_STATE_LONG_HELD;
}

bool DEV_Remote_IsKeyPressed(const DEV_Remote_s* dev, DEV_Remote_Key_e key)
{
    return DEV_Remote_GetKeyState(dev, key) == DEV_KEY_STATE_PRESSED;
}

bool DEV_Remote_IsKeyReleased(const DEV_Remote_s* dev, DEV_Remote_Key_e key)
{
    return DEV_Remote_GetKeyState(dev, key) == DEV_KEY_STATE_RELEASED;
}

bool DEV_Remote_GetKeyToggle(const DEV_Remote_s* dev, DEV_Remote_Key_e key)
{
    if (key >= DEV_KEY_COUNT)
    {
        return false;
    }
    return dev->keys[key].toggle;
}

void DEV_Remote_SetKeyToggle(DEV_Remote_s* dev, DEV_Remote_Key_e key, bool value)
{
    if (key < DEV_KEY_COUNT)
    {
        dev->keys[key].toggle = value;
    }
}

uint32_t DEV_Remote_GetFrameCount(const DEV_Remote_s* dev) { return dev->frame_count; }

uint32_t DEV_Remote_GetErrorCount(const DEV_Remote_s* dev) { return dev->error_count; }
